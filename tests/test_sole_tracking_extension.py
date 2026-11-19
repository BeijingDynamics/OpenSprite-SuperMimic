"""Geometry reward direction, old-checkpoint parity and unchanged actor inputs."""
from types import SimpleNamespace
import pytest
import torch

from opensprite_tracker.supermimic.sole_tracking import make_motion_mdp,sole_height_penalty,REVISION
from opensprite_tracker.supermimic.joint_interfaces import MotionTrackingMDP
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits,CONTEXT_DIM


EXTENSION={'revision':REVISION,'std_m':.015,'weight':.25}


def test_geometry_gradient_lifts_the_swing_foot_and_lowers_the_support_foot():
    actual=torch.tensor([[0.,.01]],requires_grad=True)
    target=torch.tensor([[.025,0.]])
    value=sole_height_penalty(actual,target,std_m=.015,weight=.25)
    value.sum().backward()
    assert actual.grad[0,0]>0
    assert actual.grad[0,1]<0
    assert -.25 <= value.item() <= 0
    assert sole_height_penalty(target,target,std_m=.015,weight=.25).item()==0
    assert torch.isfinite(sole_height_penalty(actual*1e10,target,std_m=.015,weight=.25)).all()


@pytest.fixture
def inputs():
    kin=make_kinematics('cpu',True)
    q=default_joint_pos('cpu')[None].repeat(2,1)
    p=torch.tensor([[0.,0.,.5]]).repeat(2,1)
    R=torch.eye(3).repeat(2,1,1)
    state=(p,R,torch.zeros(2,3),torch.zeros(2,3),q,torch.zeros_like(q))
    bp,bR,bv,bw=kin.forward(*state)
    motion=SimpleNamespace(root_p=p[:1],root_R=R[:1],body_p=bp[:1],body_R=bR[:1],
                           body_v=bv[:1],body_w=bw[:1],q=q[:1])
    return (kin,motion,default_joint_pos('cpu'),velocity_limits('cpu')),state


def test_old_configuration_is_exact_and_extension_keeps_input_contract(inputs):
    args,state=inputs;frame=torch.zeros(2,dtype=torch.long);action=torch.zeros(2,31)
    original=MotionTrackingMDP(*args)
    plain=make_motion_mdp({},*args)
    extended=make_motion_mdp({'motion_reward_extension':EXTENSION},*args)
    assert type(plain) is MotionTrackingMDP
    assert torch.equal(original.reward(state,frame,action,action),plain.reward(state,frame,action,action))
    xi=torch.zeros(2,CONTEXT_DIM)
    assert torch.equal(original.obs(state,action,xi,frame),extended.obs(state,action,xi,frame))
    assert extended.obs(state,action,xi,frame).shape==(2,99)
    assert original.reward_scale==extended.reward_scale
    assert torch.equal(original.termination(state,frame),extended.termination(state,frame))


def test_full_geometry_term_is_nonpositive_and_reported_consistently(inputs):
    args,state=inputs;frame=torch.zeros(2,dtype=torch.long);action=torch.zeros(2,31)
    mdp=make_motion_mdp({'motion_reward_extension':EXTENSION},*args)
    raised=list(state);raised[0]=state[0]+torch.tensor([0.,0.,.01])
    raised=tuple(raised)
    original=MotionTrackingMDP(*args)
    base=original.reward(raised,frame,action,action)
    reward,detail=mdp.reward(raised,frame,action,action,terms=True)
    assert (reward<base).all()
    assert (base-reward<=.25*.02+1e-7).all()
    torch.testing.assert_close(reward-base,.02*detail['per_reward/sole_height'])


@pytest.mark.parametrize('extension',[{},dict(EXTENSION,revision='unknown'),dict(EXTENSION,std_m=0),dict(EXTENSION,weight=float('nan'))])
def test_unknown_or_unphysical_settings_fail_closed(inputs,extension):
    with pytest.raises(ValueError):make_motion_mdp({'motion_reward_extension':extension},*inputs[0])
