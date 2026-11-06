"""Accept a saved SQP iterate only if the unchanged physical QA gates pass.

The optimizer need not converge every arm-pose cost after contact feasibility.
The complete bank still requires independent QA and rendered visual review.
"""
import argparse,json,sys
from pathlib import Path
import numpy as np,mujoco
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from refine_reference_contacts import source_targets,write_result,measured_foot
from retarget.processing import robot_contract,geometry_quality,kinematics

def finalize(name):
    source=ROOT/'data/sources/lafan1-resolved'/(name+'.bvh')
    src=ROOT/'data/motions/native_v5_v3'/(name+'.npz');output=ROOT/'data/motions/contact_v5_v11'
    with np.load(src,allow_pickle=False) as data:payload={k:data[k] for k in data.files}
    with np.load(ROOT/'outputs/refinement-debug'/(name+'.npz'),allow_pickle=False) as data:
        qpos=data['qpos'].copy();iteration=int(data['iteration'])+1
    targets,provenance,adapted=source_targets(source,payload['qpos'],ROOT/'outputs/umr-v5-v3'/name/'native.npz')
    actual_rotation=Rotation.from_quat(qpos[:,[4,5,6,3]])
    expected_rotation=Rotation.from_quat(adapted[:,[4,5,6,3]])
    root_rotation_error=float((actual_rotation*expected_rotation.inv()).magnitude().max())
    m,joints,limits,speeds=robot_contract();d=mujoco.MjData(m);feet=[m.geom(s+'_foot_collision').id for s in ['left','right']]
    positions=[];heights=[];plant=[]
    for frame,pose in enumerate(qpos):
        d.qpos[:]=pose;mujoco.mj_forward(m,d);positions.append(d.geom_xpos[feet].copy())
        heights.append([d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g] for g in feet])
        plant.extend([float(np.linalg.norm(measured_foot(m,d,g)[0]-targets[side][0][frame])) for side,g in enumerate(feet) if targets[side][2][frame]])
    positions=np.array(positions);heights=np.array(heights);contact=heights<.008;stable=contact[1:]&contact[:-1]
    speed=np.linalg.norm(np.diff(positions[...,:2],axis=0)*50,axis=-1)
    slip=float(np.sqrt(np.mean(speed[stable]**2))) if stable.any() else float('inf')
    lift=heights>.02;events=((~lift[:-1])&lift[1:]).sum(0)
    quality=geometry_quality(m,qpos);velocity=float((abs(kinematics(m,qpos)['joint_vel'])/speeds).max())
    report=dict(motion=name,iteration=iteration,plant_max_m=max(plant or [0]),stance_slip_m_s=slip,lift_events=events.tolist(),
                velocity_ratio=velocity,geometry=quality)
    source_contacts=np.stack([target[2] for target in targets],1)
    source_stable=source_contacts[1:]&source_contacts[:-1]
    false_ground=stable&~source_stable
    report.update(source_stance_rms_m_s=float(np.sqrt(np.mean(speed[source_stable]**2))) if source_stable.any() else None,
                  false_near_ground_transition_count=int(false_ground.sum()),
                  false_near_ground_rms_m_s=float(np.sqrt(np.mean(speed[false_ground]**2))) if false_ground.any() else None,
                  root_height_range_m=[float(qpos[:,2].min()),float(qpos[:,2].max())])
    root_speed=np.abs(np.diff(qpos[:,:3],axis=0)*50).max(0)
    root_acceleration=np.abs(np.diff(qpos[:,:3],n=2,axis=0)*2500).max(0)
    report.update(root_component_speed_max_m_s=root_speed.tolist(),root_component_acceleration_max_m_s2=root_acceleration.tolist(),maximum_source_root_rotation_error_rad=root_rotation_error)
    diagnostics=ROOT/'outputs/preparation-contact-v11';diagnostics.mkdir(exist_ok=True)
    (diagnostics/(name+'-candidate-qa.json')).write_text(json.dumps(report,indent=2))
    print(json.dumps(report),flush=True)
    accepted=(report['plant_max_m']<=.002 and slip<=.08 and velocity<=1.01 and heights.min()>=-.002
              and quality['minimum_collision_z_m']>=-.002 and quality['self_minimum_signed_distance_m']>=-.01
              and qpos[:,2].min()>=.25 and root_rotation_error<1e-4 and (root_speed<=np.array([1.,1.,.5])+1e-3).all() and (root_acceleration<=8.01).all()
              and (name not in ['v5_straight','v5_left','v5_right'] or min(events)>=2))
    if not accepted:raise RuntimeError('Candidate did not pass unchanged contact/geometry/lift/velocity gates')
    provenance['termination_rule']='all kinematic quality gates satisfied at saved SQP iterate; full-bank and visual QA still required'
    provenance['optimizer_fully_converged']=False
    write_result(name,src,source,output,m,qpos,payload,limits,provenance,max(plant or [0]),iteration)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--name',required=True);finalize(parser.parse_args().name)
