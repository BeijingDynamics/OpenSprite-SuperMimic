"""Reference preprocessing must preserve support and avoid artificial foot turns."""
from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from refine_neutral_contacts import stabilize_support_yaw,continuous_rigid_sole_swing


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
