"""Reference preprocessing must preserve support and avoid artificial foot turns."""
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from refine_neutral_contacts import stabilize_support_yaw,continuous_rigid_sole_swing,unit_direction_and_jacobian,retime_source_tasks,pelvis_height_from_leg_geometry,source_to_robot_bone_offset


def test_projected_toe_spin_does_not_create_a_full_robot_foot_turn():
    contact=np.r_[np.ones(5,bool),np.zeros(21,bool),np.ones(5,bool)]
    yaw=np.deg2rad(np.r_[np.full(5,170.),np.linspace(-120,120,21),np.full(5,-170.)])
    stable=stabilize_support_yaw(yaw,contact)
    assert np.max(np.abs(np.diff(stable))) < np.deg2rad(2)
    assert stable[-1]-stable[0] == pytest.approx(np.deg2rad(20))
    assert np.max(np.abs(np.angle(np.exp(1j*(stable[contact]-yaw[contact]))))) < 1e-12


def test_unsupported_entire_clip_is_not_given_an_invented_heading():
    with pytest.raises(ValueError,match='no source support'):
        stabilize_support_yaw(np.zeros(20),np.zeros(20,bool))


def test_complete_swing_keeps_support_anchors_source_peak_and_phase():
    contact=np.r_[np.ones(6,bool),np.zeros(19,bool),np.ones(6,bool)]
    path=np.zeros((31,3));path[:,2]=.0005;path[25:,:2]=[.25,.03]
    path[6:25,0]=np.linspace(.01,.24,19);path[6:25,2]=.0305
    smooth=continuous_rigid_sole_swing(path,contact)
    assert np.array_equal(smooth[contact],path[contact])
    assert smooth[:,2].max() == pytest.approx(path[:,2].max())
    assert np.all(smooth[~contact,2]>.0005)
    assert np.linalg.norm(smooth[6,:2]-smooth[5,:2]) < .001
    assert smooth[6,2]-smooth[5,2] < .001


def test_partial_boundary_swings_and_standing_are_not_reinvented():
    path=np.c_[np.arange(15)*.01,np.zeros(15),np.full(15,.02)]
    contact=np.r_[np.zeros(4,bool),np.ones(7,bool),np.zeros(4,bool)]
    assert np.array_equal(continuous_rigid_sole_swing(path,contact),path)
    assert np.array_equal(continuous_rigid_sole_swing(path,np.ones(15,bool)),path)


def test_limb_direction_derivative_and_length_invariance():
    vector=np.array([.1,.2,-.3]);jacobian=np.array([[.4,.2],[-.1,.5],[.8,-.2]])
    direction,analytic=unit_direction_and_jacobian(vector,jacobian)
    eps=1e-6
    for j in range(2):
        plus=vector+eps*jacobian[:,j];minus=vector-eps*jacobian[:,j]
        numerical=(plus/np.linalg.norm(plus)-minus/np.linalg.norm(minus))/(2*eps)
        np.testing.assert_allclose(analytic[:,j],numerical,atol=1e-9)
    np.testing.assert_allclose(unit_direction_and_jacobian(vector*2,jacobian*2)[0],direction)
    with pytest.raises(ValueError,match='Degenerate'):
        unit_direction_and_jacobian(np.zeros(3),jacobian)


def test_source_time_scaling_preserves_original_tasks_and_contacts():
    n=12;prior=np.zeros((n,38));prior[:,3]=1;prior[:,0]=np.arange(n)/50
    contact=np.r_[np.ones(4,bool),np.zeros(4,bool),np.ones(4,bool)]
    points=np.c_[np.arange(n)*.01,np.zeros(n),np.full(n,.02)]
    targets=[(points,np.zeros(n),contact)]*2
    provenance=dict(source_torso_orientation_world=np.tile(np.eye(3),(n,1,1)).tolist(),
                    source_head_orientation_world=np.tile(np.eye(3),(n,1,1)).tolist(),
                    source_upper_body_vectors=[dict(start_body='a',end_body='b',directions=np.tile([0.,0.,-1.],(n,1)).tolist())],
                    source_arm_orientations=[dict(body='arm',rotations=np.tile(np.eye(3),(n,1,1)).tolist())],
                    source_contacts=[dict(side='Left'),dict(side='Right')])
    retimed,tasks,metadata,times=retime_source_tasks(prior,targets,provenance,.5,None)
    assert len(retimed)==2*n-1
    np.testing.assert_allclose(retimed[::2],prior,atol=1e-12)
    np.testing.assert_allclose(retimed[:,0],times*.5,atol=1e-12)
    np.testing.assert_allclose(tasks[0][0][::2],points,atol=1e-12)
    np.testing.assert_array_equal(tasks[0][2][::2],contact)
    assert len(metadata['source_head_orientation_world'])==len(retimed)
    np.testing.assert_allclose(metadata['source_arm_orientations'][0]['rotations'][::2],provenance['source_arm_orientations'][0]['rotations'],atol=1e-12)


def test_source_root_height_preserves_vertical_motion_and_robot_offsets():
    from scipy.spatial.transform import Rotation
    rotation=Rotation.from_euler('y',np.array([0.,.1,-.1])[:,None]).as_matrix()
    source_hips=np.array([.85,.86,.84]);offset=np.array([.01,0,.014])
    result=pelvis_height_from_leg_geometry(source_hips,.09,.5675,.039,offset,rotation)
    robot_hip=result+np.einsum('tij,j->ti',rotation,offset)[:,2]
    np.testing.assert_allclose(robot_hip-.039,(source_hips-.09)*.5675,atol=1e-12)
    shifted=pelvis_height_from_leg_geometry(source_hips+1,1.09,.5675,.039,offset,rotation)
    np.testing.assert_allclose(shifted,result,atol=1e-12)


def test_arm_axis_mapping_uses_actual_bind_rotation_and_preserves_world_motion():
    from scipy.spatial.transform import Rotation
    from audit_geno_source_frame import NATIVE_Y_UP_TO_Z_UP as U
    source=Rotation.from_euler('xyz',[.2,-.3,1.5]).as_matrix()
    robot=Rotation.from_euler('xyz',[-.1,.6,-1.4]).as_matrix()
    offset=source_to_robot_bone_offset(source,robot)
    np.testing.assert_allclose(U@source@offset,robot,atol=1e-12)
    world_delta=Rotation.from_rotvec([.1,-.2,.3]).as_matrix()
    np.testing.assert_allclose(U@(world_delta@source)@offset,(U@world_delta@U.T)@robot,atol=1e-12)
