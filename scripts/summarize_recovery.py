"""Summarize physical gait and recovery without hiding post-disturbance falls."""
import argparse,json,sys
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ROBOT_XML,BODY_JOINT_NAMES


def summarize(directory):
    report=json.loads((directory/'summary.json').read_text())
    with np.load(directory/'rollout.npz',allow_pickle=False) as data:z={key:data[key] for key in data.files}
    dt=1/float(z['fps']);model=mujoco.MjModel.from_xml_path(str(ROBOT_XML));state=mujoco.MjData(model)
    qids=[int(model.joint(n).qposadr[0]) for n in BODY_JOINT_NAMES]
    feet=[model.geom(side+'_foot_collision').id for side in ('left','right')]
    rows=[]
    for i,result in enumerate(report['results']):
        failed=z['failed'][:,i];end=int(np.flatnonzero(failed)[0]) if failed.any() else len(failed)
        velocity_error=np.linalg.norm(z['velocity'][:,i,:2]-z['commands'][:,i,:2],axis=-1)
        yaw_error=abs(z['yaw_rate'][:,i]-z['commands'][:,i,2])
        quat=z['qpos'][:,i,3:7]
        tilt=np.arccos(np.clip(1-2*(quat[:,1]**2+quat[:,2]**2),-1,1))
        stable=(velocity_error<.10)&(yaw_error<.25)&(tilt<np.deg2rad(12))&~failed
        events=[]
        for event in result.get('forces',[]):
            begin=round(event['at_seconds']/dt);release=begin+round(event['duration_seconds']/dt)
            recovered=None
            for first in range(release,min(end,len(failed))-49):
                if stable[first:first+50].all():recovered=(first-release)*dt;break
            events.append(dict(event,alive_at_force_start=bool(end>begin),survived_to_end=bool(not failed.any()),
                               recovery_seconds=recovered,recovered_for_1s=recovered is not None))
        row=dict(case_name=result['case_name'],survived=result['survived'],first_failure_seconds=result['first_failure_seconds'],
                 velocity_error_m_s=result['velocity_error_m_s'],yaw_error_rad_s=result['yaw_error_rad_s'],force_events=events)
        start=round(3/dt)
        if end>start+1:
            positions=[];heights=[];heading=[]
            for pose in z['qpos'][start:end,i]:
                state.qpos[:7]=pose[:7];state.qpos[qids]=pose[7:];mujoco.mj_forward(model,state)
                positions.append(state.geom_xpos[feet].copy())
                heights.append([state.geom_xpos[g,2]-abs(state.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g] for g in feet])
                rootR=state.xmat[model.body('pelvis_link').id].reshape(3,3)
                rootyaw=np.arctan2(rootR[1,0],rootR[0,0])
                heading.append([np.arctan2(state.geom_xmat[g].reshape(3,3)[1,0],state.geom_xmat[g].reshape(3,3)[0,0])-rootyaw for g in feet])
            positions=np.asarray(positions);heights=np.asarray(heights);heading=np.asarray(heading)
            heading=np.arctan2(np.sin(heading),np.cos(heading))
            loaded=z['contact_force_mean_n'][start:end,i]>5
            mask=loaded[:-1]&loaded[1:]
            speed=np.linalg.norm(np.diff(positions[...,:2],axis=0)/dt,axis=-1)
            row.update(pre_failure_samples=end-start,foot_heading_median_deg=np.rad2deg(np.median(heading,axis=0)).tolist(),
                sole_height_p95_m=np.quantile(heights,.95,axis=0).tolist(),
                maximum_sole_height_m=heights.max(0).tolist(),
                loaded_foot_centre_rms_speed_m_s=[float(np.sqrt(np.mean(speed[mask[:,j],j]**2))) if mask[:,j].any() else None for j in range(2)],
                root_tilt_p95_deg=float(np.rad2deg(np.quantile(tilt[start:end],.95))))
        rows.append(row)
    summary=dict(iteration=report['iteration'],seed=report['seed'],results=rows,
        recovery_definition='After force release: velocity error <0.10m/s, yaw-rate error <0.25rad/s and pelvis tilt <12deg for 1s; no failure reset.',
        foot_metric='Loaded means measured physical mean vertical foot force >5N at both endpoints. Foot-centre speed includes rolling; no universal 2cm clearance pass/fail.',
        hardware_qualified=False)
    (directory/'recovery_quality.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(iteration=report['iteration'],seed=report['seed'],survivors=sum(x['survived'] for x in rows),cases=len(rows))))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--evaluation',type=Path,required=True);summarize(p.parse_args().evaluation)
