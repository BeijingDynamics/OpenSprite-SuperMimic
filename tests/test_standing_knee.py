"""Test soft standing shaping without constraining walking or locking knees."""
import math
import torch
from test_posture_width import state
from opensprite_tracker.supermimic.posture_width import standing_knee_cost,STAND_DEFAULTS,SWING_DEFAULTS,make_velocity_mdp

def test_only_excess_flexion_is_penalized_with_correct_gradient():
    q=torch.tensor([[math.radians(38),-math.radians(15)]],requires_grad=True)
    c=standing_knee_cost(q,torch.zeros(1,3),STAND_DEFAULTS);c.sum().backward()
    assert c.item()>0 and q.grad[0,0]>0 and q.grad[0,1]==0
    assert standing_knee_cost(torch.tensor([[.1,-.1]]),torch.zeros(1,3),STAND_DEFAULTS).item()==0

def test_mirrored_angles_match_and_walking_turning_gate_is_negligible():
    q=torch.tensor([[.7,-.6]]);zero=torch.zeros(1,3)
    base=standing_knee_cost(q,zero,STAND_DEFAULTS)
    torch.testing.assert_close(base,standing_knee_cost(-q,zero,STAND_DEFAULTS))
    assert standing_knee_cost(q,torch.tensor([[.15,0.,0.]]),STAND_DEFAULTS).item()<base.item()*1e-9
    assert standing_knee_cost(q,torch.tensor([[0.,0.,.2]]),STAND_DEFAULTS).item()<base.item()*1e-6

def test_zero_weight_factory_preserves_previous_reward_exactly(state):
    args,S=state;old=make_velocity_mdp({'velocity_reward_extension':SWING_DEFAULTS},*args)
    new=make_velocity_mdp({'velocity_reward_extension':dict(STAND_DEFAULTS,standing_knee_weight=0.)},*args)
    a=torch.zeros(2,31);cmd=torch.zeros(2,3)
    torch.testing.assert_close(old.reward_for_command(S,cmd,a,a),new.reward_for_command(S,cmd,a,a),rtol=0,atol=0)
