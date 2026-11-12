"""Add physical foot geometry to every command case without scoring reset frames."""
import argparse,json,sys
from pathlib import Path
import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import BODY_JOINT_NAMES,ROBOT_XML
from audit_sole_tracking import geometry,measure,load


def headings(poses):
    model=mujoco.MjModel.from_xml_path(str(ROBOT_XML));state=mujoco.MjData(model)
    ids=[int(model.joint(name).qposadr[0]) for name in BODY_JOINT_NAMES]
    feet=[model.geom(side+'_foot_collision').id for side in ('left','right')]
    pelvis=model.body('pelvis_link').id;torso=model.body('waist_yaw_link').id
    foot_yaw=[];torso_yaw=[];tilt=[]
    for pose in poses:
        state.qpos[:7]=pose[:7];state.qpos[ids]=pose[7:];mujoco.mj_forward(model,state)
        base=state.xmat[pelvis].reshape(3,3);upper=state.xmat[torso].reshape(3,3)
        yaw=np.arctan2(base[1,0],base[0,0])
        relative=[]
        for g in feet:
            rotation=state.geom_xmat[g].reshape(3,3)
            difference=np.arctan2(rotation[1,0],rotation[0,0])-yaw
            relative.append(np.arctan2(np.sin(difference),np.cos(difference)))
        difference=np.arctan2(upper[1,0],upper[0,0])-yaw
        torso_yaw.append(np.arctan2(np.sin(difference),np.cos(difference)))
        foot_yaw.append(relative);tilt.append(np.arccos(np.clip(upper[2,2],-1,1)))
    return np.rad2deg(foot_yaw),np.rad2deg(torso_yaw),np.rad2deg(tilt)


def summarize(directory,start_seconds=3.):
    summary=json.loads((directory/'summary.json').read_text());trace=load(directory/'rollout.npz')
    fps=float(trace['fps']);rows=[]
    for case in summary['results']:
        env=case['env'];failed=np.flatnonzero(trace['failed'][:,env])
        end=int(failed[0]) if len(failed) else len(trace['qpos']);begin=round(start_seconds*fps)
        row={key:case.get(key) for key in ['env','case_name','label','survived','first_failure_seconds','mean_vx_m_s','mean_yaw_rad_s','velocity_error_m_s','yaw_error_rad_s','last5s_mean_velocity_m_s']}
        if end<=begin+1:
            row['geometry_unavailable']='Failure before scoring window';rows.append(row);continue
        poses=trace['qpos'][begin:end,env];forces=trace['contact_force_mean_n'][begin:end,env]
        metric=measure(geometry(poses),fps,forces)
        # Keep individual events in the full audit; this table records their distribution.
        events=metric.pop('clearance_events')
        metric['event_peak_median_m']=[float(np.median([e['peak_m'] for e in side])) if side else None for side in events]
        metric['event_peak_min_m']=[min(e['peak_m'] for e in side) if side else None for side in events]
        yaw,torso,tilt=headings(poses)
        metric['loaded_foot_heading_relative_pelvis_median_deg']=[float(np.median(yaw[forces[:,j]>5,j])) if (forces[:,j]>5).any() else None for j in range(2)]
        metric['torso_heading_relative_pelvis_median_deg']=float(np.median(torso))
        metric['torso_tilt_p95_deg']=float(np.quantile(tilt,.95))
        row.update(scoring_start_seconds=begin/fps,scoring_end_seconds=end/fps,geometry=metric);rows.append(row)
    return dict(checkpoint_sha256=summary['checkpoint_sha256'],iteration=summary['iteration'],seed=summary['seed'],results=rows,
                limits='Physical simulation only. Sole events use >5mm for at least two50Hz samples. Loaded corner slip and heading use measured mean vertical force >5N; they are not exact contact-point trajectories. Survival alone is not gait quality.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--evaluation',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=summarize(args.evaluation)
    with args.output.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(iteration=report['iteration'],seed=report['seed'],cases=len(report['results']),output=str(args.output))))
