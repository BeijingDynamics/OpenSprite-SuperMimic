"""Reward direction, orientation invariance, geometry and legacy compatibility."""
from types import SimpleNamespace
import math
import pytest
import torch
from opensprite_tracker.supermimic.posture_width import DEFAULTS,posture_width_cost,make_velocity_mdp
from opensprite_tracker.supermimic.joint_interfaces import VelocityMDP
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits,CONTEXT_DIM

def test_cost_corrects_both_pitch_signs_and_does_not_demand_a_fixed_pitch():
    theta=torch.tensor([-.15,0.,.15],requires_grad=True)
    up=torch.stack([theta.sin(),torch.zeros_like(theta),theta.cos()],-1)
    cost=posture_width_cost(up,torch.full_like(theta,.13),torch.full_like(theta,.06),DEFAULTS)
    cost.sum().backward()
    assert cost[1]==0 and theta.grad[0]<0 and theta.grad[2]>0
    assert torch.allclose(cost[0],cost[2])

def test_width_cost_preserves_left_right_order_and_discourages_excessive_width():
    sep=torch.tensor([-.12,.07,.13,.23],requires_grad=True)
    gap=sep-.05;up=torch.tensor([[0.,0.,1.]]).repeat(4,1)
    cost=posture_width_cost(up,sep,gap,DEFAULTS);cost.sum().backward()
    assert cost[0]>cost[1]>cost[2] and cost[2]==0 and cost[3]>0
    assert sep.grad[0]<0 and sep.grad[1]<0 and sep.grad[3]>0

@pytest.fixture
def state():
    kin=make_kinematics('cpu',True);q=default_joint_pos()[None].repeat(2,1)
    S=(torch.zeros(2,3),torch.eye(3).repeat(2,1,1),torch.zeros(2,3),torch.zeros(2,3),q,torch.zeros_like(q))
    return (kin,SimpleNamespace(),default_joint_pos(),velocity_limits()),S

def test_observations_termination_and_legacy_reward_unchanged(state):
    args,S=state;old=VelocityMDP(*args);plain=make_velocity_mdp({},*args)
    new=make_velocity_mdp({'velocity_reward_extension':DEFAULTS},*args)
    a=torch.zeros(2,31);cmd=torch.zeros(2,3);xi=torch.zeros(2,CONTEXT_DIM);f=torch.zeros(2,dtype=torch.long)
    assert torch.equal(old.reward_for_command(S,cmd,a,a),plain.reward_for_command(S,cmd,a,a))
    assert torch.equal(old.obs(S,a,xi,f),new.obs(S,a,xi,f))
    assert torch.equal(old.termination(S,f),new.termination(S,f))
    assert (new.reward_for_command(S,cmd,a,a)<=old.reward_for_command(S,cmd,a,a)).all()

def test_world_yaw_does_not_change_cost_or_width(state):
    args,S=state;mdp=make_velocity_mdp({'velocity_reward_extension':DEFAULTS},*args)
    angle=.9;c,s=math.cos(angle),math.sin(angle);Y=torch.tensor([[c,-s,0.],[s,c,0.],[0.,0.,1.]])
    rotated=(S[0],Y@S[1],S[2],S[3],S[4],S[5])
    a,b=mdp.posture_geometry(S),mdp.posture_geometry(rotated)
    for x,y in zip(a,b):torch.testing.assert_close(x,y,atol=2e-7,rtol=2e-6)

def test_foot_box_projection_matches_explicit_corners(state):
    args,S=state;mdp=make_velocity_mdp({'velocity_reward_extension':DEFAULTS},*args)
    up,sep,gap=mdp.posture_geometry(S)
    p,R,_,_=mdp.kin.forward(*S[:6]);fr=R[...,mdp.feet,:,:]
    center=p[...,mdp.feet,:]+(fr@mdp.kin.sole_centers[...,None]).squeeze(-1)
    signs=torch.tensor([[x,y,z] for x in [-1.,1.] for y in [-1.,1.] for z in [-1.,1.]])
    corners=center[...,None,:]+torch.einsum('bfij,fcj->bfci',fr@mdp.kin.sole_R,signs[None]*mdp.kin.sole_half[:,None,:])
    explicit=corners[:,0,:,1].amin(-1)-corners[:,1,:,1].amax(-1)
    torch.testing.assert_close(gap,explicit)

def test_bad_config_rejected(state):
    args,_=state
    with pytest.raises(ValueError):make_velocity_mdp({'velocity_reward_extension':dict(DEFAULTS,minimum_center_m=.3)},*args)
