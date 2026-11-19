"""Score physical rollout posture and signed, orientation-aware sole spacing."""
import json,sys
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ROBOT_XML,BODY_JOINT_NAMES

def score(directory):
    directory=Path(directory);z=np.load(directory/'rollout.npz');summary=json.loads((directory/'summary.json').read_text())
    m=mujoco.MjModel.from_xml_path(str(ROBOT_XML));d=mujoco.MjData(m)
    ids=[int(m.joint(n).qposadr[0]) for n in BODY_JOINT_NAMES]
    feet=[m.geom(s+'_foot_collision').id for s in ('left','right')]
    pelvis=m.body('pelvis_link').id;torso=m.body('waist_yaw_link').id;rows=[]
    for case in summary['results']:
        env=case['env'];values=[]
        for t in range(150,len(z['qpos'])):
            if z['failed'][t,env]:break
            pose=z['qpos'][t,env];d.qpos[:7]=pose[:7];d.qpos[ids]=pose[7:];mujoco.mj_kinematics(m,d)
            R=d.xmat[pelvis].reshape(3,3);U=d.xmat[torso].reshape(3,3);yaw=np.arctan2(R[1,0],R[0,0])
            front=np.array([np.cos(yaw),np.sin(yaw),0]);lat=np.array([-np.sin(yaw),np.cos(yaw),0])
            pitch=np.degrees(np.arctan2(front@U[:,2],U[2,2]));sep=(d.geom_xpos[feet[0]]-d.geom_xpos[feet[1]])@lat
            extent=sum(np.abs(lat@d.geom_xmat[g].reshape(3,3))@m.geom_size[g] for g in feet)
            values.append([pitch,sep,sep-extent])
        row={k:case.get(k) for k in ['case_name','survived','mean_vx_m_s','mean_yaw_rad_s','yaw_error_rad_s','last5s_mean_velocity_m_s']}
        if values:
            a=np.array(values);row.update(pitch_median_deg=float(np.median(a[:,0])),pitch_abs_p95_deg=float(np.quantile(abs(a[:,0]),.95)),
                 center_median_m=float(np.median(a[:,1])),center_p05_m=float(np.quantile(a[:,1],.05)),
                 edge_gap_p05_m=float(np.quantile(a[:,2],.05)),crossed_fraction=float(np.mean(a[:,1]<0)))
        rows.append(row)
    result=dict(checkpoint_sha256=summary['checkpoint_sha256'],results=rows,
        limitation='Lateral edge projection is conservative during fore/aft staggering; it is not a full 3D collision test.')
    (directory/'posture-width.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':print(json.dumps(score(sys.argv[1]),indent=2))
