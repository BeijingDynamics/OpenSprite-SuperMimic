"""Reconstruct R1 lateral lean from pelvis gravity and measured joint geometry."""
import hashlib
import argparse
import json
from pathlib import Path
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/deploy-sm-walk-r1-xbox120-20261119'
SOURCE=BASE/'original'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source',type=Path,default=SOURCE)
parser.add_argument('--output',type=Path,default=BASE/'roll-analysis')
args=parser.parse_args();SOURCE=args.source;OUT=args.output
OUT.mkdir(parents=True,exist_ok=True)
contract=json.loads((SOURCE/'bundle/contract.json').read_text())
xml=ROOT/'assets/opensprite/opensprite.xml'
assert hashlib.sha256(xml.read_bytes()).hexdigest()==contract['asset_mjcf_sha256']
model=mujoco.MjModel.from_xml_path(str(xml));state=mujoco.MjData(model)
names=contract['joint_names'];qids=[int(model.joint(n).qposadr[0]) for n in names]
torso=model.body('waist_yaw_link').id;waist=names.index('waist_roll_joint')
dtype=np.dtype([('obs','<f4',(993,)),('raw','<f4',(31,)),('action','<f8',(31,)),('target','<f8',(31,))])


def rotation(g):
    r=np.arctan2(-g[1],-g[2]);p=np.arctan2(g[0],np.hypot(g[1],g[2]))
    sr,cr,sp,cp=np.sin(r),np.cos(r),np.sin(p),np.cos(p)
    return np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])


def geometry(q,gravity):
    rows=[]
    for pose,g in zip(q,gravity):
        R=rotation(g);quat=np.empty(4);mujoco.mju_mat2Quat(quat,R.ravel())
        state.qpos[:3]=0;state.qpos[3:7]=quat;state.qpos[qids]=pose;mujoco.mj_kinematics(model,state)
        upper=state.xmat[torso].reshape(3,3)
        rows.append([np.degrees(np.arctan2(R[1,2],R[2,2])),
                     np.degrees(np.arctan2(upper[1,2],upper[2,2])),
                     np.degrees(np.arctan2(upper[0,2],upper[2,2]))])
    return np.array(rows)


def stats(v):
    return dict(mean=float(v.mean()),median=float(np.median(v)),p05=float(np.quantile(v,.05)),p95=float(np.quantile(v,.95))) if len(v) else None


runs=[];full=[]
for path in sorted((SOURCE/'powered').glob('xbox-powered-*')):
    report=json.loads((path/'commissioning.json').read_text())
    row=dict(run=path.name,passed=report['passed'],failure=report.get('failure'),
        final_disabled_count=report['final_disabled_count'],motor_enable_attempts=report.get('motor_enable_attempts'))
    if not (path/'native.bin.inputs').exists():runs.append(row);continue
    x=np.fromfile(path/'native.bin.inputs',dtype='<f8').reshape(-1,71)
    record=np.fromfile(path/'native.bin',dtype=dtype)
    if not len(x) or len(x)!=len(record):row['analysis_error']='Incomplete paired record';runs.append(row);continue
    actual=geometry(x[:,:31],x[:,65:68]);target=geometry(record['target'],x[:,65:68]);t=np.arange(len(x))*.02
    straight=(t>=20)&(x[:,68]>.08)&(np.abs(x[:,70])<.03)
    segments={'supported_zero_10_20':(t>=10)&(t<20),'commanded_straight':straight,
        'straight_full_025':(t>=20)&(x[:,68]>=.249)&(np.abs(x[:,70])<.03),
        'straight_fast':(t>=20)&(x[:,68]>=.30)&(np.abs(x[:,70])<.03),
        'left_turn':(t>=20)&(x[:,70]>.05),'right_turn':(t>=20)&(x[:,70]<-.05),
        'late_zero':(t>=25)&(abs(x[:,68])<.01)&(abs(x[:,70])<.01)}
    row['frames']=len(x);row['segments']={}
    for label,mask in segments.items():
        row['segments'][label]=dict(samples=int(mask.sum()),pelvis_left_lean_deg=stats(actual[mask,0]),
            torso_left_lean_deg=stats(actual[mask,1]),torso_forward_lean_deg=stats(actual[mask,2]),
            waist_roll_feedback_deg=stats(np.degrees(x[mask,waist])),
            waist_roll_policy_target_deg=stats(np.degrees(record['target'][mask,waist])),
            waist_roll_target_error_deg=stats(np.degrees(record['target'][mask,waist]-x[mask,waist])),
            target_torso_left_lean_deg=stats(target[mask,1]),
            forward_command_m_s=stats(x[mask,68]),yaw_command_rad_s=stats(x[mask,70]))
    runs.append(row)
    if report['passed']:full.append((path.name,t,x,actual,target))
report=dict(runs=runs,sign_convention='Positive lean means the body-up axis points left (+pelvis-y) or forward (+pelvis-x). This is not the signed Euler roll convention.',
    method='Measured pelvis projected gravity and measured joint FK; global yaw set to zero.',
    limitations=['No measured foot forces, global velocity or sling load.',
        'Policy target geometry uses the current measured pelvis orientation and pre-protection joint targets; it is not a predicted future pose.',
        'Commanded straight samples can include turns ending or disturbances; correlation does not establish causation.'])
(OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
if full:
    fig,axes=plt.subplots(3,len(full),figsize=(7*len(full),9),squeeze=False,layout='constrained')
    for col,(name,t,x,a,b) in enumerate(full):
        axes[0,col].plot(t,x[:,68],label='Forward command (m/s)');axes[0,col].plot(t,x[:,70],label='Yaw command (rad/s)')
        axes[0,col].set_title(name)
        axes[1,col].plot(t,a[:,0],label='Pelvis left lean');axes[1,col].plot(t,a[:,1],label='Torso left lean')
        axes[1,col].plot(t,b[:,1],alpha=.5,label='Policy target FK, measured pelvis')
        axes[1,col].set_ylabel('Degrees; positive = left')
        axes[2,col].plot(t,np.degrees(x[:,waist]),label='Waist roll feedback')
        axes[2,col].plot(t,np.degrees(np.fromfile(SOURCE/'powered'/name/'native.bin',dtype=dtype)['target'][:,waist]),label='Policy waist roll target')
        axes[2,col].set_ylabel('Joint angle (deg)');axes[2,col].set_xlabel('Policy time (s)')
        for ax in axes[:,col]:ax.grid(alpha=.25);ax.legend();ax.axvspan(20.04,50.04,alpha=.05,color='green')
    fig.savefig(OUT/'roll-traces.png',dpi=140);plt.close(fig)
print(json.dumps([dict(run=r['run'],passed=r['passed'],segments=r.get('segments')) for r in runs],indent=2))
