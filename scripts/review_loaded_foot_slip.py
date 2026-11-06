"""Recheck low-speed gait using loaded near-floor material-point velocities.

The earlier sole-center/2cm-lift proxy cannot distinguish rolling or low shuffles.
Uses saved pre-action qd/gyro and prior post-state root velocity, without replaying policy.
"""
import argparse,json,sys,itertools,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
ROBOT_XML=ROOT/'assets/opensprite/opensprite.xml'
BODY_JOINT_NAMES=[j.get('name') for j in ET.parse(ROOT/'assets/opensprite/vendor/sprite0825_v5/sprite0825_float.urdf').getroot().findall('joint') if j.get('type')!='fixed']
CONTROL_DEFAULT=json.loads((ROOT/'data/command_default_pose.json').read_text())
DEFAULT=np.array([CONTROL_DEFAULT[n] for n in BODY_JOINT_NAMES],np.float32)

def main(args):
    with np.load(args.rollout,allow_pickle=False) as archive:trace={k:archive[k] for k in archive.files}
    m=mujoco.MjModel.from_xml_path(str(ROBOT_XML));d=mujoco.MjData(m)
    qids=[int(m.joint(n).qposadr[0]) for n in BODY_JOINT_NAMES];vids=[int(m.joint(n).dofadr[0]) for n in BODY_JOINT_NAMES]
    geoms=[m.geom(s+'_foot_collision').id for s in ['left','right']];report=[]
    for env in range(trace['qpos'].shape[1]):
        values=[[],[]];forces=[[],[]]
        for t in range(150,len(trace['qpos']),2):
            pose=trace['qpos'][t-1,env];obs=trace['proprioception'][t,env]
            d.qpos[:7]=pose[:7];d.qpos[qids]=obs[:31]+DEFAULT
            R=Rotation.from_quat(pose[[4,5,6,3]]).as_matrix();yaw=np.arctan2(R[1,0],R[0,0])
            H=Rotation.from_euler('z',yaw).as_matrix();d.qvel[:3]=H@trace['velocity'][t-1,env]
            d.qvel[3:6]=obs[62:65];d.qvel[vids]=obs[31:62]
            mujoco.mj_kinematics(m,d);mujoco.mj_comPos(m,d)
            for side,g in enumerate(geoms):
                force=float(trace['contact_force_mean_n'][t-1,env,side]);forces[side].append(force)
                if force<5.:continue
                rot=d.geom_xmat[g].reshape(3,3)
                corners=np.array([d.geom_xpos[g]+rot@(m.geom_size[g]*np.array([x,y,-1])) for x,y in itertools.product([-1,1],repeat=2)])
                low=corners[corners[:,2]<=min(.003,float(corners[:,2].min())+.001)]
                speeds=[]
                for point in low:
                    J=np.zeros((3,m.nv));Jr=np.zeros_like(J);mujoco.mj_jac(m,d,J,Jr,point,int(m.geom_bodyid[g]))
                    speeds.append(float(np.linalg.norm((J@d.qvel)[:2])))
                if speeds:values[side].append(min(speeds))
        feet=[]
        for side in range(2):
            speed=np.asarray(values[side]);force=np.asarray(forces[side])
            feet.append(dict(side=['left','right'][side],loaded_near_floor_samples=len(speed),
                optimistic_contact_point_speed_median_m_s=float(np.median(speed)) if len(speed) else None,
                optimistic_contact_point_speed_rms_m_s=float(np.sqrt(np.mean(speed**2))) if len(speed) else None,
                fraction_loaded_samples_over_0_05_m_s=float(np.mean(speed>.05)) if len(speed) else None,
                mean_normal_force_n=float(force.mean()),fraction_force_below_5n=float((force<5).mean())))
        report.append(dict(env=env,command=trace['commands'][-1,env].tolist(),feet=feet))
    bank_path=ROOT/'data/motions/contact_v5_v11/v5_locomotion.npz'
    if not bank_path.exists():bank_path=ROOT/'results/data/motions/contact_v5_v11/v5_locomotion.npz'
    bank=np.load(bank_path,allow_pickle=False);pelvis=bank['body_names'].astype(str).tolist().index('pelvis_link')
    joint_ids=[bank['joint_names'].astype(str).tolist().index(n) for n in BODY_JOINT_NAMES]
    root_v=bank['body_lin_vel_w'][:,pelvis];quiet=int(np.linalg.norm(root_v[:,:2],axis=-1).argmin())
    q_all=bank['joint_pos'][:,joint_ids];initial_q=trace['proprioception'][0,0,:31]+DEFAULT
    matched=int(np.max(abs(q_all-initial_q),axis=1).argmin());quiet=matched
    clip=int(np.searchsorted(bank['motion_starts'],quiet,side='right')-1);q=q_all[quiet]
    selected=['left_hip_pitch_joint','right_hip_pitch_joint','left_knee_joint','right_knee_joint','left_elbow_joint','right_elbow_joint']
    initial=dict(clip=str(bank['motion_names'][clip]),packed_frame=quiet,clip_frame=quiet-int(bank['motion_starts'][clip]),
        root_position_reference_m=bank['body_pos_w'][quiet,pelvis].tolist(),root_planar_speed_before_zeroing_m_s=float(np.linalg.norm(root_v[quiet,:2])),
        selected_joint_angles_deg={name:float(q[BODY_JOINT_NAMES.index(name)]*180/np.pi) for name in selected},
        matched_initial_joint_error_rad=float(abs(q-initial_q).max()),
        maximum_difference_from_urdf_zero_rad=float(abs(q).max()),maximum_difference_from_controller_default_rad=float(abs(q-DEFAULT).max()))
    output=dict(method='Optimistic minimum XY speed among near-floor sole corners, only when measured per-foot normal force >=5N; qvel reconstructed from recorded instantaneous proprioception',
        limitations='Contact forces are means over previous10 physics substeps; contact points are reconstructed geometric candidates, not logged individual contact manifolds. No2cm clearance requirement.',
        results=report,initialization=initial)
    args.output.write_text(json.dumps(output,indent=2));print(json.dumps(output,indent=2),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--rollout',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
