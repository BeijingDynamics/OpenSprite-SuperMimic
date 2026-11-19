"""CPU closed-loop rate sensitivity; no hardware access or training changes.

Uses the exported actor and robot XML. The flat CPU scene is a diagnostic
reconstruction, not a bitwise reproduction of mjlab/MuJoCo Warp training.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT.parent / '.analysis_deps'))
import onnxruntime as ort

BUNDLE = ROOT / 'outputs/deploy-20261116/live-bundle10000'
CONTRACT = json.loads((BUNDLE / 'contract.json').read_text())
NAMES = CONTRACT['joint_names']
KP, KD, CAPS, OFFSET, SCALE, DEFAULT = [np.array(CONTRACT[k]) for k in
    ['stiffness', 'damping', 'effort_limit', 'action_offset', 'action_scale', 'default_joint_pos']]
XML = ROOT / 'assets/opensprite/opensprite.xml'
DTYPE = np.dtype([('obs','<f4',(993,)),('raw','<f4',(31,)),('action','<f8',(31,)),('target','<f8',(31,))])
RUNS = ['xbox-powered-20261116-152917-5629', 'xbox-powered-20261116-153129-5687']
BASE = ROOT / 'outputs/deploy-20261116/live-evidence/supermimic-pm01-xbox30-1116'
PROFILES = {'pd500_dt2':(.002,2), 'pd500_dt1':(.001,2), 'direct1000_dt1':(.001,1)}

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def root_quat(gravity):
    roll=np.arctan2(-gravity[1],-gravity[2])
    pitch=np.arctan2(gravity[0],np.hypot(gravity[1],gravity[2]))
    sr,cr,sp,cp=np.sin(roll),np.cos(roll),np.sin(pitch),np.cos(pitch)
    quat=np.empty(4)
    mujoco.mju_mat2Quat(quat,np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]]).ravel())
    return quat

def scene(dt):
    spec=mujoco.MjSpec.from_file(str(XML))
    spec.worldbody.add_geom(name='diagnostic_ground',type=mujoco.mjtGeom.mjGEOM_PLANE,
                            size=[0,0,.1],contype=1,conaffinity=1,friction=[.8,.005,.0001])
    model=spec.compile()
    model.opt.timestep=dt
    model.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    model.opt.iterations=50;model.opt.ls_iterations=25
    for g in range(model.ngeom):
        if model.geom_contype[g] and g != model.geom('diagnostic_ground').id:
            model.geom_priority[g]=1;model.geom_condim[g]=3;model.geom_friction[g,0]=.8
    assert model.nu==0
    return model,mujoco.MjData(model)

def metrics(model,data,base,torso,feet):
    R=data.xmat[base].reshape(3,3);T=data.xmat[torso].reshape(3,3)
    yaw=np.arctan2(R[1,0],R[0,0]);lat=np.array([-np.sin(yaw),np.cos(yaw),0])
    delta=data.geom_xpos[feet[0]]-data.geom_xpos[feet[1]]
    ra,rb=[data.geom_xmat[g].reshape(3,3) for g in feet]
    v=[*ra.T,*rb.T,*np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)]
    axes=np.array([a/np.linalg.norm(a) for a in v if np.linalg.norm(a)>1e-8])
    gap=np.max(abs(axes@delta)-abs(axes@ra)@model.geom_size[feet[0]]-abs(axes@rb)@model.geom_size[feet[1]])
    return [delta@lat,gap,np.degrees(np.arctan2(T[2,1],T[2,2])),
            np.degrees(np.arctan2(-T[2,0],np.hypot(T[2,1],T[2,2]))),
            np.degrees(np.arccos(np.clip(R[2,2],-1,1))),
            np.cos(yaw)*data.qvel[0]+np.sin(yaw)*data.qvel[1]]

def run(session,source,profile,args):
    dt,direct_ms=PROFILES[profile];m,d=scene(dt)
    qi=np.array([int(m.joint(n).qposadr[0]) for n in NAMES]);vi=np.array([int(m.joint(n).dofadr[0]) for n in NAMES])
    base=m.body('pelvis_link').id;torso=m.body('waist_yaw_link').id
    feet=[m.geom(s+'_foot_collision').id for s in ('left','right')]
    records=np.fromfile(BASE/source/'native.bin',DTYPE)
    inputs=np.fromfile(BASE/source/'native.bin.inputs','<f8').reshape(-1,71)
    start=1000;initial=inputs[start]
    d.qpos[:3]=[0,0,.5];d.qpos[3:7]=root_quat(initial[65:68]);d.qpos[qi]=initial[:31]
    d.qvel[vi]=initial[31:62];d.qvel[3:6]=initial[62:65]
    mujoco.mj_forward(m,d)
    bottoms=[d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g] for g in feet]
    d.qpos[2]+=.001-min(bottoms);mujoco.mj_forward(m,d)
    history=records['obs'][start,:990].copy();previous=history[620:930].reshape(10,31)[-1].copy()
    target=OFFSET+SCALE*previous;held_torque=np.zeros(31)
    ankle=np.array(['ankle_' in n for n in NAMES]);direct=~ankle
    pending=[];rows=[];failure=None;clock_start=time.monotonic()
    ns=int(round(.02/dt));direct_ticks=int(round(direct_ms*.001/dt));ankle_ticks=int(round(.002/dt))
    rng=np.random.default_rng(101);delay_ms=0
    for step in range(min(len(inputs)-start,round(args.seconds/.02))):
        mujoco.mj_forward(m,d)
        twist=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,base,twist,1)
        parts=[d.qpos[qi]-DEFAULT,d.qvel[vi],previous,twist[:3],-d.xmat[base].reshape(3,3)[2]]
        if step:
            cursor=0
            for p in parts:
                width=len(p);h=history[cursor:cursor+width*10].reshape(10,width)
                h[:-1]=h[1:];h[-1]=p;cursor+=width*10
        else:
            # Preserve observed pre-start memory; verify the reconstructed latest state.
            assert np.max(abs(twist[:3]-initial[62:65]))<1e-9
            assert np.max(abs(parts[-1]-initial[65:68]))<1e-5
        cmd=inputs[start+step,68:].copy()
        if args.no_turn:cmd[2]=0
        obs=np.r_[history,cmd].astype(np.float32)
        action=session.run(None,{'obs':obs[None]})[0][0]
        previous=action.copy();requested=OFFSET+SCALE*action
        # Paired stochastic target-delay hypothesis, refreshed every 20 ms.
        # Its 0/2 ms values stay fixed in physical time for all integration rates.
        if rng.random()>.9:delay_ms=int(rng.integers(0,2))*2
        lag_ms=args.delay_ms if args.delay_ms is not None else delay_ms
        pending.append((step*ns+int(round(lag_ms*.001/dt)),requested.copy()))
        q_before=d.qpos[qi].copy();qd_before=d.qvel[vi].copy()
        rows.append((step*.02,cmd.copy(),d.qpos.copy(),q_before,qd_before,requested.copy(),np.array(metrics(m,d,base,torso,feet))))
        for sub in range(ns):
            tick=step*ns+sub
            while pending and pending[0][0]<=tick:target=pending.pop(0)[1]
            raw=KP*(target-d.qpos[qi])-KD*d.qvel[vi]
            if tick%direct_ticks==0:held_torque[direct]=np.clip(raw[direct],-CAPS[direct],CAPS[direct])
            if tick%ankle_ticks==0:held_torque[ankle]=np.clip(raw[ankle],-CAPS[ankle],CAPS[ankle])
            d.qfrc_applied[:]=0;d.qfrc_applied[vi]=held_torque
            mujoco.mj_step(m,d)
        mujoco.mj_forward(m,d)
        if not np.isfinite(d.qpos).all() or d.qpos[2]<.25 or d.xmat[base].reshape(3,3)[2,2]<.5:
            failure=(step+1)*.02;break
    label=f'{source}_{profile}';values={k:np.array([row[i] for row in rows]) for i,k in enumerate(['time','command','qpos','q','qd','target','metrics'])}
    np.savez_compressed(args.output/(label+'.npz'),**values)
    v=values['metrics'];walk=values['command'][:,0]>.15;late=values['time']>=20
    def summary(mask):
        if not mask.any():return None
        a=v[mask];wr=NAMES.index('waist_roll_joint')
        return dict(width_median_mm=float(np.median(a[:,0])*1000),minimum_sat_gap_mm=float(a[:,1].min()*1000),
            torso_roll_median_deg=float(np.median(a[:,2])),torso_pitch_median_deg=float(np.median(a[:,3])),
            vx_mean_m_s=float(a[:,5].mean()),waist_roll_median_deg=float(np.degrees(np.median(values['q'][mask,wr]))),
            waist_target_median_deg=float(np.degrees(np.median(values['target'][mask,wr]))))
    result=dict(source=source,profile=profile,physics_dt=dt,direct_pd_hz=1000/direct_ms,ankle_pd_hz=500,
        duration_s=len(rows)*.02,fall_at_s=failure,wall_s=time.monotonic()-clock_start,
        forward_samples=summary(walk),late_samples=summary(late),initial_sole_height_difference_m=max(bottoms)-min(bottoms))
    print(json.dumps(result),flush=True)
    return result

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=34);p.add_argument('--profiles',nargs='+',choices=list(PROFILES),default=list(PROFILES))
    p.add_argument('--runs',type=int,nargs='+',default=[0,1]);p.add_argument('--no-turn',action='store_true');p.add_argument('--delay-ms',type=float)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    assert sha(XML)==CONTRACT['asset_mjcf_sha256'];assert sha(BUNDLE/'policy.onnx')==CONTRACT['policy_onnx_sha256']
    assert np.all(SCALE==.25)
    opts=ort.SessionOptions();opts.intra_op_num_threads=2;opts.inter_op_num_threads=1
    session=ort.InferenceSession(str(BUNDLE/'policy.onnx'),sess_options=opts,providers=['CPUExecutionProvider'])
    errors=[]
    for source in RUNS:
        all_records=np.fromfile(BASE/source/'native.bin',DTYPE)
        # Frame zero is the explicit zero-action lifecycle handshake, not inference.
        assert np.all(all_records['raw'][0]==0)
        records=all_records[1::20]
        pred=session.run(None,{'obs':records['obs'].copy()})[0]
        error=float(abs(pred-records['raw']).max());errors.append(error)
        np.testing.assert_allclose(pred,records['raw'],atol=1e-5,rtol=1e-4)
    report=dict(actor_parity_max_errors=errors,model_sha256=sha(BUNDLE/'policy.onnx'),asset_sha256=sha(XML),
        mujoco=mujoco.__version__,onnxruntime=ort.__version__,no_turn=args.no_turn,delay_ms=args.delay_ms,
        scope='CPU serial-joint PD rate sensitivity, starting from measured state/history at policy time 20 s; closed loop actor, recorded commands.',
        limitations=['Not a qualified hardware emulator or bitwise mjlab/Warp reproduction.',
            'Root translation velocity is unavailable and initialized to zero; height inferred from sole boxes.',
            'No sling, physical parallel transmission, motor-space protection, sensor filtering or measured transport delay.',
            'Serial effort caps are unchanged training caps, not runtime motor protection thresholds.',
            'Uniform stochastic 0/2 ms target delay is a paired diagnostic approximation, not exact training buffer replay.'],results=[])
    print('Recorded actor parity passed',errors,flush=True)
    for source in [RUNS[i] for i in args.runs]:
        for profile in args.profiles:
            report['results'].append(run(session,source,profile,args))
            (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')

if __name__=='__main__':main()
