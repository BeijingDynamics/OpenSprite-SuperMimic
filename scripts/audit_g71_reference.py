"""Recompute the pinned G71 walk on the current v5 geometry without pose edits."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ASSET_REVISION,ROBOT_XML,JOINT_CONTROL
from opensprite_tracker.supermimic.robot import make_kinematics
from opensprite_tracker.supermimic.rotations import quat_to_mat,mat_to_quat,yaw_mat
from retarget.processing import kinematics,geometry_quality

EXPECTED='97ec26f45460316e55845137b2adeb83958bd2676967af166085df085b509256'


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if hashlib.sha256(args.source.read_bytes()).hexdigest()!=EXPECTED:
        raise ValueError('Not the pinned G71 V20 whole-body reference')
    args.output.mkdir(parents=True,exist_ok=False)
    with np.load(args.source,allow_pickle=False) as z:
        source={key:z[key] for key in z.files}
    if float(source['fps'])!=100.:raise ValueError('Expected original 100 Hz data')
    model=mujoco.MjModel.from_xml_path(str(ROBOT_XML));data=mujoco.MjData(model)
    kin=make_kinematics('cpu',True)
    joint_names=list(kin.joint_names);body_names=list(kin.tracked_names)
    source_joints=source['joint_names'].tolist();source_bodies=source['body_names'].tolist()
    jids=[source_joints.index(name) for name in joint_names]
    root=source_bodies.index('pelvis_link');indices=np.arange(0,len(source['joint_pos']),2)
    poses=np.zeros((len(indices),model.nq))
    poses[:,:3]=source['body_pos_w'][indices,root]
    poses[:,3:7]=source['body_quat_w'][indices,root]
    q=source['joint_pos'][indices][:,jids]
    qids=[int(model.joint(name).qposadr[0]) for name in joint_names]
    poses[:,qids]=q
    derived=kinematics(model,poses,50.)
    tensor=lambda x:torch.as_tensor(x,dtype=torch.float32)
    native_names=[model.joint(i).name for i in range(1,model.njnt)]
    order=[native_names.index(name) for name in joint_names]
    qd=derived['joint_vel'][:,order]
    source_R=quat_to_mat(tensor(poses[:,3:7]))
    native_bodies=[model.body(i).name for i in range(1,model.nbody)]
    r=native_bodies.index('pelvis_link')
    state=(tensor(poses[:,:3]),source_R,tensor(derived['body_lin_vel_w'][:,r]),
           tensor(derived['body_ang_vel_w'][:,r]),tensor(q),tensor(qd))
    bp,bR,bv,bw=kin.forward(*state)
    common=[name for name in body_names if name in source_bodies]
    oldp=source['body_pos_w'][indices][:,[source_bodies.index(n) for n in common]]
    newp=bp[:,[body_names.index(n) for n in common]].numpy()
    oldR=quat_to_mat(tensor(source['body_quat_w'][indices][:,[source_bodies.index(n) for n in common]]))
    newR=bR[:,[body_names.index(n) for n in common]]
    geometry=geometry_quality(model,poses,50.)
    feet=[model.geom(side+'_foot_collision').id for side in ('left','right')]
    foot_height=[];foot_heading=[];foot_position=[];corners=[]
    for frame,pose in enumerate(poses):
        data.qpos[:]=pose;mujoco.mj_forward(model,data)
        heading=yaw_mat(source_R[frame]).numpy()
        foot_height.append([data.geom_xpos[g,2]-np.abs(data.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g] for g in feet])
        foot_position.append(data.geom_xpos[feet].copy())
        frame_corners=[]
        for g in feet:
            half=model.geom_size[g]
            local_corners=np.array([[sx*half[0],sy*half[1],-half[2]] for sx in (-1,1) for sy in (-1,1)])
            frame_corners.append(local_corners@data.geom_xmat[g].reshape(3,3).T+data.geom_xpos[g])
        corners.append(frame_corners)
        local=[heading.T@data.geom_xmat[g].reshape(3,3)[:,0] for g in feet]
        foot_heading.append([np.arctan2(v[1],v[0]) for v in local])
    height=np.asarray(foot_height);position=np.asarray(foot_position)
    contact=height<.008;stable=contact[:-1]&contact[1:]
    speed=np.linalg.norm(np.diff(position[...,:2],axis=0)*50,axis=-1)
    slip=float(np.sqrt(np.mean(speed[stable]**2))) if stable.any() else None
    corners=np.asarray(corners)
    corner_speed=np.linalg.norm(np.diff(corners[...,:2],axis=0)*50,axis=-1)
    near=(corners[:-1,...,2]<.008)&(corners[1:,...,2]<.008)
    geometric_corner_slip=float(np.sqrt(np.mean(corner_speed[near]**2))) if near.any() else None
    support_proxy=((corners[:-1,...,2]<.025)&(corners[1:,...,2]<.025)&(corner_speed<.08)).any(-1)
    lift=height>.02;lifts=((~lift[:-1])&lift[1:]).sum(0)
    limits=np.stack((kin.q_lo.numpy(),kin.q_hi.numpy()),-1)
    limit_excess=float(np.maximum(limits[:,0]-q,q-limits[:,1]).clip(0).max())
    localv=(yaw_mat(source_R).transpose(-1,-2)@state[2][...,None]).squeeze(-1).numpy()
    errors=[]
    if limit_excess>1e-6:errors.append('Joint hard-limit violation')
    if geometry['minimum_collision_z_m']<-.002:errors.append('Active collider penetrates ground by more than 2mm')
    if geometry['self_minimum_signed_distance_m']<-.01:errors.append('Self penetration exceeds 1cm')
    if min(lifts)<2:errors.append('Insufficient bilateral foot lifts over 2cm')
    if slip is None or slip>.08:errors.append('Stance-foot RMS slip exceeds 0.08m/s or support is absent')
    bank=args.output/'g71_walk_v5_50hz.npz'
    np.savez_compressed(bank,fps=np.array(50.,np.float32),joint_names=np.array(joint_names),body_names=np.array(body_names),
        joint_pos=q,joint_vel=qd,body_pos_w=bp.numpy(),body_quat_w=mat_to_quat(bR).numpy(),
        body_lin_vel_w=bv.numpy(),body_ang_vel_w=bw.numpy(),qpos=poses.astype(np.float32),
        clip_starts=np.array([0]),clip_lengths=np.array([len(poses)]),clip_names=np.array(['g71_straight']),
        asset_revision=np.array(ASSET_REVISION),retarget_method=np.array('g71_pinned_pose_current_v5_fk_v1'))
    report=dict(accepted_for_training=False,visual_review_passed=False,numeric_candidate_passed=not errors,
        source_sha256=EXPECTED,motion_bank_sha256=hashlib.sha256(bank.read_bytes()).hexdigest(),asset_revision=ASSET_REVISION,
        frames=len(poses),duration_s=(len(poses)-1)/50.,
        pose_processing='Every second original sample, no pose/root edits; velocities and all-body FK recomputed at 50 Hz',
        source_body_position_max_difference_m=float(abs(oldp-newp).max()),
        source_body_rotation_matrix_max_difference=float((oldR-newR).abs().max()),
        max_hard_joint_limit_excess_rad=limit_excess,
        final_geometry=geometry,mean_local_root_velocity_m_s=localv.mean(0).tolist(),
        mean_foot_heading_deg=np.rad2deg(foot_heading).mean(0).tolist(),
        foot_heading_abs_p90_deg=np.quantile(abs(np.rad2deg(foot_heading)),.9,axis=0).tolist(),
        min_sole_height_m=height.min(0).tolist(),max_sole_height_m=height.max(0).tolist(),
        foot_lift_events=lifts.tolist(),stance_foot_rms_speed_m_s=slip,
        near_ground_corner_rms_speed_m_s=geometric_corner_slip,
        v20_style_low_motion_support_fraction=support_proxy.mean(0).tolist(),
        support_proxy_note='Height below 25mm and planar corner speed below 0.08m/s. This velocity-conditioned proxy does not independently prove low slip or physical contact.',
        maximum_training_velocity_threshold_ratio=float((abs(qd)/np.array([JOINT_CONTROL[n].urdf_velocity_rad_s for n in joint_names])).max()),
        errors=errors,limitations='Source-pose compatibility audit only. Review rendering and current-model dynamics before admitting training.')
    (args.output/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
