"""Offline reward and reference audit; no policy or robot changes."""
import json
import sys
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ROBOT_XML,BODY_JOINT_NAMES

def audit():
    model=mujoco.MjModel.from_xml_path(str(ROBOT_XML));data=mujoco.MjData(model)
    ids=[int(model.joint(n).qposadr[0]) for n in BODY_JOINT_NAMES]
    feet=[model.geom(side+'_foot_collision').id for side in ['left','right']]
    pelvis=model.body('pelvis_link').id;torso=model.body('waist_yaw_link').id
    def geometry(poses):
        rows=[]
        for pose in poses:
            data.qpos[:7]=pose[:7];data.qpos[ids]=pose[7:];mujoco.mj_kinematics(model,data)
            R=data.xmat[pelvis].reshape(3,3);T=data.xmat[torso].reshape(3,3)
            yaw=np.arctan2(R[1,0],R[0,0]);front=np.array([np.cos(yaw),np.sin(yaw),0])
            pitch=np.arctan2(front@T[:,2],T[2,2])
            heights=[data.geom_xpos[g,2]-np.abs(data.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g] for g in feet]
            rows.append([pitch,*heights])
        return np.array(rows)
    ref=np.load(ROOT/'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz')
    r=geometry(ref['qpos']);speed=np.linalg.norm(ref['body_lin_vel_w'][:,0,:2],axis=-1);quiet=int(speed.argmin())
    result={'reference':{'quiet_frame':quiet,'quiet_torso_pitch_deg':float(np.rad2deg(r[quiet,0])),
        'torso_pitch_p05_median_p95_deg':np.rad2deg(np.quantile(r[:,0],[.05,.5,.95])).tolist(),
        'sole_height_p95_m':np.quantile(r[:,1:],.95,axis=0).tolist()},'rollouts':{}}
    base=ROOT/'outputs/posture-width-experiment'
    for name in ['baseline','eval-5000','eval-10000']:
        z=np.load(base/name/'rollout.npz');report=json.loads((base/name/'summary.json').read_text());rows=[]
        for case in report['results']:
            i=case['env'];g=geometry(z['qpos'][150:,i]);a=z['policy_action'][149:,i]
            old=8*np.maximum(np.abs(np.sin(g[:,0]))-np.sin(np.deg2rad(2)),0)**2
            new=32*np.maximum(np.abs(np.sin(g[:,0]))-np.sin(np.deg2rad(1)),0)**2
            smooth=.3*np.sum(np.diff(a,axis=0)**2,axis=-1)
            rows.append({'case':case['case_name'],'mean_old_pitch_cost':float(old.mean()),
                'mean_proposed_pitch_cost':float(new.mean()),'mean_action_smooth_cost':float(smooth.mean()),
                'sole_height_p95_m':np.quantile(g[:,1:],.95,axis=0).tolist()})
        result['rollouts'][name]=rows
    return result

if __name__=='__main__':
    result=audit();out=ROOT/'outputs/targeted-upright-clearance-20261118';out.mkdir(exist_ok=True)
    (out/'diagnosis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
