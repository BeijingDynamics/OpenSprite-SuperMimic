"""Measure actual policy foot lifts/slip from recorded physical qpos, not rewards."""
import argparse,json,sys
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ROBOT_XML,BODY_JOINT_NAMES,JOINT_CONTROL

def events(height):
    planted=False;air=0;count=0
    for h in height:
        if h<.008:planted=True;air=0
        elif h>.02 and planted:
            air+=1
            if air>=2:count+=1;planted=False;air=0
        else:air=0
    return count

def analyze(args):
    with np.load(args.rollout,allow_pickle=False) as archive:trace={k:archive[k] for k in archive.files}
    poses=trace['qpos'];fps=float(trace['fps'])
    m=mujoco.MjModel.from_xml_path(str(ROBOT_XML));d=mujoco.MjData(m)
    qids=[int(m.joint(n).qposadr[0]) for n in BODY_JOINT_NAMES]
    feet=[m.geom(side+'_foot_collision').id for side in ['left','right']]
    rows=[];start=round(3*fps)
    for env in (args.envs if args.envs else range(poses.shape[1])):
        positions=[];heights=[];tilts=[]
        for pose in poses[:,env]:
            d.qpos[:7]=pose[:7];d.qpos[qids]=pose[7:];mujoco.mj_forward(m,d)
            positions.append(d.geom_xpos[feet].copy())
            heights.append([d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g] for g in feet])
            tilts.append(float(np.arccos(np.clip(d.xmat[m.body('waist_yaw_link').id].reshape(3,3)[2,2],-1,1))))
        positions=np.asarray(positions);heights=np.asarray(heights)
        speed=np.linalg.norm(np.diff(positions[...,:2],axis=0)*fps,axis=-1)
        stable=(heights[1:]<.008)&(heights[:-1]<.008);stable[:start]=False
        selected=heights[start:];slip=float(np.sqrt(np.mean(speed[stable]**2))) if stable.any() else None
        row=dict(env=env,command=trace['commands'][-1,env].tolist(),lift_events_over_2cm=[events(selected[:,i]) for i in range(2)],
                 maximum_sole_height_m=selected.max(0).tolist(),p95_sole_height_m=np.quantile(selected,.95,axis=0).tolist(),
                 geometric_stance_foot_rms_speed_m_s=slip,near_floor_fraction=(selected<.008).mean(0).tolist(),
                 body_distance_m=float(np.linalg.norm(poses[start+1:,env,:2]-poses[start:-1,env,:2],axis=-1).sum()))
        row['case_name']=str(trace['case_names'][env]) if 'case_names' in trace else str(env)
        row['torso_tilt_p95_deg']=float(np.quantile(tilts[start:],.95)*180/np.pi)
        if 'raw_torque_peak' in trace:
            limits=np.array([JOINT_CONTROL[n].command_effort_nm for n in BODY_JOINT_NAMES])
            row['requested_effort_over_limit_fraction']=float((trace['raw_torque_peak'][start:,env]>limits).mean())
        rows.append(row)
    moving=[row for row in rows if np.linalg.norm(row['command'])>.05]
    for row in rows:
        row['walking_accepted']=(min(row['lift_events_over_2cm'])>=2 and row['geometric_stance_foot_rms_speed_m_s'] is not None
                                 and row['geometric_stance_foot_rms_speed_m_s']<=.12)
    report=dict(source=str(args.rollout),source_shapes={k:list(trace[k].shape) for k in trace},start_seconds=3.,
                geometric_contact_proxy='sole height below8mm at adjacent frames; sole-center XY speed; rolling contact may contribute',results=rows,
                moving_command_walking_accepted=bool(moving) and all(row['walking_accepted'] for row in moving),
                assessment_limit='Legacy2cm-clearance heuristic only. Failure is not a definitive walking/sliding classification, especially at low speed; review loaded material-point velocities.')
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--rollout',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--envs',type=int,nargs='+');analyze(p.parse_args())
