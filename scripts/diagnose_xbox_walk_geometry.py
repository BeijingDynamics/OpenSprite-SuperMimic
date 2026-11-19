"""Reconstruct foot/torso geometry from recorded joint feedback, without simulation."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/deploy-20261116/walking-diagnosis'
OUT.mkdir(exist_ok=True)
CONTRACT = json.loads((ROOT/'outputs/deploy-20261116/live-bundle10000/contract.json').read_text())
XML = ROOT/'assets/opensprite/opensprite.xml'
assert hashlib.sha256(XML.read_bytes()).hexdigest() == CONTRACT['asset_mjcf_sha256']
MODEL = mujoco.MjModel.from_xml_path(str(XML))
STATE = mujoco.MjData(MODEL)
NAMES = CONTRACT['joint_names']
QIDS = [int(MODEL.joint(n).qposadr[0]) for n in NAMES]
assert QIDS == list(range(7, 38)), 'Saved simulation qpos must match contract joint order'
FEET = [MODEL.geom(s+'_foot_collision').id for s in ('left','right')]
TORSO = MODEL.body('waist_yaw_link').id
DTYPE = np.dtype([('obs','<f4',(993,)),('raw','<f4',(31,)),('action','<f8',(31,)),('target','<f8',(31,))])

def root_rotation(g):
    roll = np.arctan2(-g[1], -g[2])
    pitch = np.arctan2(g[0], np.hypot(g[1], g[2]))
    sr,cr,sp,cp=np.sin(roll),np.cos(roll),np.sin(pitch),np.cos(pitch)
    return np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])

def geometry(q, gravity=None, root_q=None):
    values=[]
    for i, pose in enumerate(q):
        STATE.qpos[:3]=0
        if root_q is None:
            R=root_rotation(gravity[i]);quat=np.empty(4)
            mujoco.mju_mat2Quat(quat,R.ravel());STATE.qpos[3:7]=quat
        else:
            STATE.qpos[3:7]=root_q[i]
        STATE.qpos[QIDS]=pose
        mujoco.mj_kinematics(MODEL,STATE)
        pelvis=STATE.xmat[MODEL.body('pelvis_link').id].reshape(3,3)
        yaw=np.arctan2(pelvis[1,0],pelvis[0,0])
        lateral=np.array([-np.sin(yaw),np.cos(yaw),0.])
        foot_delta=STATE.geom_xpos[FEET[0]]-STATE.geom_xpos[FEET[1]]
        sep=float(foot_delta@lateral)
        half=[];head=[]
        for gid in FEET:
            R=STATE.geom_xmat[gid].reshape(3,3)
            half.append(float(np.abs(lateral@R)@MODEL.geom_size[gid]))
            angle=np.arctan2(R[1,0],R[0,0])-yaw
            head.append(np.degrees(np.arctan2(np.sin(angle),np.cos(angle))))
        # Independent OBB separating-axis margin. Positive values are a lower
        # bound on Euclidean separation; negative values certify box overlap.
        ra,rb=[STATE.geom_xmat[g].reshape(3,3) for g in FEET]
        candidates=[*ra.T,*rb.T,*np.cross(ra.T[:,None,:],rb.T[None,:,:]).reshape(-1,3)]
        axes=np.asarray([v/np.linalg.norm(v) for v in candidates if np.linalg.norm(v)>1e-8])
        radii=np.abs(axes@ra)@MODEL.geom_size[FEET[0]]+np.abs(axes@rb)@MODEL.geom_size[FEET[1]]
        gap=float(np.max(np.abs(axes@foot_delta)-radii))
        upper=STATE.xmat[TORSO].reshape(3,3)
        tilt=np.degrees(np.arccos(np.clip(upper[2,2],-1,1)))
        pitch=np.degrees(np.arctan2(-upper[2,0],np.hypot(upper[2,1],upper[2,2])))
        roll=np.degrees(np.arctan2(upper[2,1],upper[2,2]))
        values.append([sep,sep-sum(half),gap,tilt,pitch,roll,*head])
    return np.array(values)

def stats(v):
    return dict(min=float(np.min(v)),p05=float(np.quantile(v,.05)),median=float(np.median(v)),p95=float(np.quantile(v,.95)),max=float(np.max(v)))

report={'method':'Forward kinematics of feedback and policy targets using the exact training MJCF; no dynamics replay.',
        'limitations':['No measured global translation, sole forces or external contact sensor.',
        'Sole-box separating-axis gap uses training geometry (104 x 44 x 10 mm); it does not prove physical contact.',
        'Positive 3D SAT gap is a lower bound on Euclidean separation; negative certifies modeled box overlap.',
        'Signed lateral edge gap can be negative when feet pass fore/aft or at different heights; use 3D SAT too.',
        'Target geometry is the policy target before motor-space protection, not an executed pose.',
        'Gravity defines root roll/pitch; global heading is unavailable and set to zero.'], 'runs':[]}
fig,axes=plt.subplots(4,2,figsize=(13,12),sharex='col',layout='constrained')
for col,run in enumerate(['xbox-powered-20261116-152917-5629','xbox-powered-20261116-153129-5687']):
    p=ROOT/'outputs/deploy-20261116/live-evidence/supermimic-pm01-xbox30-1116'/run
    inputs=np.fromfile(p/'native.bin.inputs',dtype='<f8').reshape(-1,71)
    records=np.fromfile(p/'native.bin',dtype=DTYPE)
    assert len(inputs)==len(records)
    actual=geometry(inputs[:,:31],inputs[:,65:68]);target=geometry(records['target'],inputs[:,65:68])
    t=np.arange(len(inputs))*.02
    can=json.loads((p/'can.json').read_text())
    result={'run':run,'frames':len(t),'operator_report':'Sling slack, no pushing or support in second run.',
            'can_errors':can['errors'],'disabled':sum(v['final_disabled'] for v in can['motors'].values()),'segments':{}}
    for label,mask in [('pre_window',(t>=10)&(t<20)),('early_control',(t>=22)&(t<32)),('late_control',(t>=40)&(t<50)),('stop',(t>=50))]:
        a=actual[mask];b=target[mask]
        result['segments'][label]={
            'commands_mean':inputs[mask,68:].mean(0).tolist(),'commands_min':inputs[mask,68:].min(0).tolist(),'commands_max':inputs[mask,68:].max(0).tolist(),
            'center_separation_m':stats(a[:,0]),'lateral_edge_gap_m':stats(a[:,1]),'sole_box_sat_gap_m':stats(a[:,2]),
            'target_center_separation_m':stats(b[:,0]),'target_sole_box_sat_gap_m':stats(b[:,2]),
            'sat_gap_below_5mm_fraction':float(np.mean(a[:,2]<.005)),
            'box_overlap_frames':int(np.sum(a[:,2]<-1e-6)),
            'torso_tilt_deg':stats(a[:,3]),'torso_pitch_deg':stats(a[:,4]),'torso_roll_deg':stats(a[:,5]),
            'pelvis_tilt_deg':stats(np.degrees(np.arccos(np.clip(-inputs[mask,67],-1,1)))),
            'waist_roll_deg':stats(np.degrees(inputs[mask,NAMES.index('waist_roll_joint')])),
            'waist_yaw_deg':stats(np.degrees(inputs[mask,NAMES.index('waist_yaw_joint')])),
            'foot_heading_median_deg':np.median(a[:,6:8],axis=0).tolist()}
    result['motors']=[dict(name=k,**v) for k,v in can['motors'].items()]
    walking=(t>=20)&(t<50)&(inputs[:,68]>.15)
    hips=[NAMES.index(s+'_hip_roll_joint') for s in ('left','right')]
    hybrid_q=inputs[:,:31].copy()
    hybrid_q[:,hips]=records['target'][:,hips]
    hybrid=geometry(hybrid_q,inputs[:,65:68])
    result['forward_command_samples']={
        'count':int(walking.sum()),
        'measured_width_mm':stats(actual[walking,0]*1000),
        'policy_target_width_mm':stats(target[walking,0]*1000),
        'hip_roll_target_only_width_mm':stats(hybrid[walking,0]*1000),
        'hip_roll_target_only_width_change_mm':stats((hybrid[walking,0]-actual[walking,0])*1000),
        'modeled_overlap_frames':int((actual[walking,2]<-1e-6).sum()),
        'minimum_sat_gap_mm':float(actual[walking,2].min()*1000),
        'hip_roll_target_minus_measured_mean_deg':np.degrees((records['target'][walking][:,hips]-inputs[walking,:31][:,hips]).mean(0)).tolist(),
        'limitation':'Replacing hip roll targets is an instantaneous geometry sensitivity, not a dynamically achievable correction.'}
    waist=NAMES.index('waist_roll_joint')
    for label,mask in [('early_control',(t>=22)&(t<32)),('late_control',(t>=40)&(t<50))]:
        result['segments'][label]['policy_waist_roll_deg']=stats(np.degrees(records['target'][mask,waist]))
        result['segments'][label]['policy_target_torso_roll_deg']=stats(target[mask,5])
    report['runs'].append(result)
    np.savez_compressed(OUT/(run+'.npz'),time=t,inputs=inputs,measured=actual,policy_target=target)
    axes[0,col].plot(t,inputs[:,68],label='Forward (m/s)');axes[0,col].plot(t,inputs[:,70],label='Yaw (rad/s)')
    axes[0,col].set_title('Run '+str(col+1));axes[0,col].legend()
    axes[1,col].plot(t,actual[:,0]*1000,label='Measured FK center width');axes[1,col].plot(t,target[:,0]*1000,alpha=.55,label='Policy target FK')
    axes[1,col].set_ylabel('Foot center width (mm)');axes[1,col].legend()
    axes[2,col].plot(t,actual[:,2]*1000,label='3D sole-box SAT gap');axes[2,col].axhline(5,color='r',ls='--',label='5 mm margin')
    axes[2,col].set_ylabel('Separating-axis gap (mm)');axes[2,col].legend()
    axes[3,col].plot(t,actual[:,4],label='Torso pitch');axes[3,col].plot(t,actual[:,5],label='Torso roll')
    axes[3,col].plot(t,np.degrees(np.arccos(np.clip(-inputs[:,67],-1,1))),alpha=.65,label='Pelvis tilt')
    axes[3,col].set_ylabel('Angle (deg)');axes[3,col].set_xlabel('Policy time (s)');axes[3,col].legend()
    for ax in axes[:,col]:ax.axvspan(20.04,50.04,color='green',alpha=.05);ax.grid(alpha=.2)

sim=np.load(ROOT/'outputs/deploy-20261116/seed45-quarter10000/rollout.npz')
report['saved_simulation']=[]
for env in [0,1,3,4]:
    q=sim['qpos'][150:,env]
    g=geometry(q[:,7:],root_q=q[:,3:7])
    report['saved_simulation'].append({'case':str(sim['case_names'][env]),'center_separation_m':stats(g[:,0]),'sole_box_sat_gap_m':stats(g[:,2]),'torso_tilt_deg':stats(g[:,3]),'torso_pitch_deg':stats(g[:,4]),'torso_roll_deg':stats(g[:,5])})
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
fig.savefig(OUT/'walking_geometry.png',dpi=150)
print(OUT/'report.json')
