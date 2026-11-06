"""Fail-closed v5 reference QA: source identity, FK, ground, stepping and slip.

These are kinematic/contact-proxy checks, not proof of dynamic feasibility.
"""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.kinematics import OpenSpriteKinematics
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.rotations import mat_to_quat
from opensprite_tracker.robot import ASSET_REVISION,URDF,JOINT_CONTROL

def validate(bank):
    source_manifest=json.loads((ROOT/'data/sources/lafan1-resolved/selected_segments.json').read_text())
    expected={row['name']:row['category'] for row in source_manifest}
    m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/opensprite.xml'));d=mujoco.MjData(m)
    kin=OpenSpriteKinematics('cpu',active_collision_only=True)
    motion=MotionLib(bank,kin,'cpu')
    with np.load(bank,allow_pickle=False) as original:payload={k:original[k] for k in original.files}
    if str(payload.get('asset_revision'))!=ASSET_REVISION:raise ValueError('Reference bank not bound to current v5 asset')
    if str(payload['retarget_method'])!='umr_native_nr_surface_vector_v1':raise ValueError('References are not native UMR')
    if set(motion.clip_names)!=set(expected):raise ValueError('Motion bank does not contain every selected genuine source')
    # Canonicalize cached body velocities to the same analytic FK as training.
    payload.update(body_pos_w=motion.body_p.numpy(),body_quat_w=mat_to_quat(motion.body_R).numpy(),
                   body_lin_vel_w=motion.body_v.numpy(),body_ang_vel_w=motion.body_w.numpy())
    np.savez_compressed(bank,**payload)
    qpos=np.concatenate((motion.root_p.numpy(),mat_to_quat(motion.root_R).numpy(),motion.q.numpy()),1)
    qids=[int(m.joint(n).qposadr[0]) for n in kin.joint_names]
    feet=[m.geom(side+'_foot_collision').id for side in ['left','right']]
    rows=[];all_errors=[]
    for name,begin,end in zip(motion.clip_names,motion.clip_start.tolist(),motion.clip_end.tolist()):
        category=expected[name];positions=[];height=[]
        for pose in qpos[begin:end+1]:
            d.qpos[:7]=pose[:7];d.qpos[qids]=pose[7:];mujoco.mj_forward(m,d)
            positions.append(d.geom_xpos[feet].copy())
            height.append([d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g] for g in feet])
        positions=np.array(positions);height=np.array(height)
        speed=np.linalg.norm(np.diff(positions[...,:2],axis=0)*50,axis=-1)
        contact=height<.008;stable=contact[:-1]&contact[1:];lift=height>.02
        lifts=((~lift[:-1])&lift[1:]).sum(0)
        slip=float(np.sqrt(np.mean(speed[stable]**2))) if stable.any() else None
        rootR=motion.root_R[begin:end+1].numpy();yaw=np.unwrap(np.arctan2(rootR[:,1,0],rootR[:,0,0]));net=float(yaw[-1]-yaw[0])
        qd=motion.qd[begin:end+1].numpy()
        root_positions=motion.root_p[begin:end+1].numpy()
        root_speed=np.abs(np.diff(root_positions,axis=0)*50).max(0)
        root_acceleration=np.abs(np.diff(root_positions,n=2,axis=0)*2500).max(0)
        import xml.etree.ElementTree as ET
        urdf=ET.parse(URDF).getroot()
        limits={j.get('name'):float(j.find('limit').get('velocity')) for j in urdf.findall('joint') if j.get('type')!='fixed'}
        velocity=np.array([limits[n] for n in kin.joint_names])
        ratio=float((abs(qd)/velocity).max())
        errors=[]
        source_qa=bank.parent/(name+'.json')
        if not source_qa.exists():errors.append('missing native conversion geometry report')
        else:
            native=json.loads(source_qa.read_text())['final_geometry']
            if native['self_minimum_signed_distance_m']<-.01:errors.append('self-penetration exceeds 1cm')
            if native['minimum_collision_z_m']<-.002:errors.append('non-foot collision penetrates ground')
        if float(height.min())<-.002:errors.append('foot penetrates ground by more than 2mm')
        if category in ('straight','left','right') and min(lifts)<2:errors.append('insufficient actual bilateral foot lifts')
        if slip is None or slip>.08:errors.append('stance-foot RMS sliding exceeds 0.08m/s or no support detected')
        if ratio>1.01:errors.append('joint velocity exceeds official limits')
        if (root_speed>np.array([1.,1.,.5])+1e-3).any():errors.append('reference root velocity exceeds preprocessing envelope')
        if (root_acceleration>8.01).any():errors.append('reference root acceleration exceeds preprocessing envelope')
        if category=='left' and net<.2:errors.append('no actual left turn after v5 retarget')
        if category=='right' and net>-.2:errors.append('no actual right turn after v5 retarget')
        if motion.root_p[begin:end+1,2].min()<.25:errors.append('locomotion pelvis height is implausibly low')
        rows.append(dict(name=name,category=category,frames=end-begin+1,lift_events=lifts.tolist(),
                         min_sole_height_m=float(height.min()),stance_foot_rms_speed_m_s=slip,net_yaw_rad=net,
                         max_joint_velocity_ratio=ratio,accepted=not errors,errors=errors))
        rows[-1].update(root_component_speed_max_m_s=root_speed.tolist(),root_component_acceleration_max_m_s2=root_acceleration.tolist())
        threshold=np.array([JOINT_CONTROL[n].urdf_velocity_rad_s for n in kin.joint_names])
        rows[-1]['max_training_velocity_cost_threshold_ratio']=float((abs(qd)/threshold).max())
        all_errors.extend([name+': '+e for e in errors])
    report=dict(accepted_for_training=not all_errors,asset_revision=ASSET_REVISION,
                motion_bank_sha256=hashlib.sha256(bank.read_bytes()).hexdigest(),references=rows,
                source_manifest_sha256=hashlib.sha256((ROOT/'data/sources/lafan1-resolved/selected_segments.json').read_bytes()).hexdigest(),
                errors=all_errors,limitations='Kinematic ground/contact/velocity checks only; not dynamic feasibility or hardware admission')
    bank.with_suffix('.qa.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    if all_errors:raise RuntimeError('Reference bank rejected; do not train')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--motion',type=Path,required=True);validate(p.parse_args().motion)
