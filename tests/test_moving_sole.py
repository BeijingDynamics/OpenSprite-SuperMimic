"""Check reward geometry, symmetry, gradients, and the zero-command boundary."""
import torch
from opensprite_tracker.supermimic.posture_width import moving_sole_cost,SWING_DEFAULTS
from test_posture_width import state
from opensprite_tracker.supermimic.posture_width import make_velocity_mdp,DEFAULTS
from opensprite_tracker.supermimic.robot import CONTEXT_DIM

def test_low_moving_foot_has_upward_gradient_but_stationary_stance_is_free():
    h=torch.tensor([[.005,0.]],requires_grad=True);v=torch.tensor([[[.2,0.],[0.,0.]]])
    cost=moving_sole_cost(h,v,torch.tensor([[.25,0.,0.]]),SWING_DEFAULTS);cost.sum().backward()
    assert cost.item()>0 and h.grad[0,0]<0 and h.grad[0,1]==0

def test_zero_command_and_sufficient_clearance_have_no_cost():
    v=torch.ones(2,2,2);h=torch.tensor([[0.,0.],[.025,.025]])
    cmd=torch.tensor([[0.,0.,0.],[.25,0.,0.]])
    assert torch.equal(moving_sole_cost(h,v,cmd,SWING_DEFAULTS),torch.zeros(2))

def test_swapping_feet_or_rotating_world_heading_preserves_cost():
    h=torch.tensor([[.006,.014]]);v=torch.tensor([[[.2,.1],[-.1,.3]]]);cmd=torch.tensor([[.2,0.,.2]])
    expected=moving_sole_cost(h,v,cmd,SWING_DEFAULTS)
    torch.testing.assert_close(expected,moving_sole_cost(h.flip(-1),v.flip(-2),cmd,SWING_DEFAULTS))
    rot=torch.tensor([[0.,-1.],[1.,0.]])
    torch.testing.assert_close(expected,moving_sole_cost(h,v@rot,cmd,SWING_DEFAULTS))

def test_turn_in_place_also_has_clearance_cost():
    cost=moving_sole_cost(torch.zeros(1,2),torch.ones(1,2,2)*.1,torch.tensor([[0.,0.,.2]]),SWING_DEFAULTS)
    assert cost.item()>0

def test_full_factory_preserves_observations_and_zero_weight_reward(state):
    args,S=state
    config=dict(SWING_DEFAULTS,swing_weight=0.)
    plain=make_velocity_mdp({'velocity_reward_extension':dict(DEFAULTS,pitch_weight=32.,pitch_deadband_deg=1.)},*args)
    new=make_velocity_mdp({'velocity_reward_extension':config},*args)
    a=torch.zeros(2,31);xi=torch.zeros(2,CONTEXT_DIM);f=torch.zeros(2,dtype=torch.long);cmd=torch.full((2,3),.1)
    torch.testing.assert_close(new.reward_for_command(S,cmd,a,a),plain.reward_for_command(S,cmd,a,a),rtol=0,atol=0)
    assert torch.equal(plain.obs(S,a,xi,f),new.obs(S,a,xi,f))
    assert torch.equal(plain.termination(S,f),new.termination(S,f))
