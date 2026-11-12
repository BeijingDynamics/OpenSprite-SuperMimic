"""Physical integration, observation semantics and v2 compatibility guards."""
import copy
from types import SimpleNamespace

import pytest
import torch
from torch import nn

from opensprite_tracker.supermimic.networks import WorldModel
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.rotations import so3_exp, mat_to_6d
from opensprite_tracker.supermimic.joint_interfaces import SharedController, MotionTrackingMDP
from opensprite_tracker.supermimic.revisions import revision_contract
from opensprite_tracker.supermimic.engine import Learner
from opensprite_tracker.supermimic.joint_learner import VelocityLearner
from test_engine_microbatch import make_learner, assert_modules_equal


def state(batch=2, memory=0):
    return (torch.zeros(batch, 3), torch.eye(3).expand(batch, 3, 3).clone(),
            torch.ones(batch, 3), torch.zeros(batch, 3), torch.zeros(batch, 31),
            torch.ones(batch, 31), torch.zeros(batch, memory))


def test_analytic_integration_matches_ten_semi_implicit_substeps():
    wm = WorldModel(hidden=(8,), memory_dim=2, dynamics_mode='acceleration_v2')
    s = state(memory=2)
    # Constant acceleration in each world/joint axis; fixed z rotation axis.
    delta = torch.cat((torch.tensor([.2, -.1, .3, 0., 0., .4]),
                       torch.full((31,), .6), torch.tensor([.1, -.2])))
    wm.out_norm.mean.copy_(delta)
    s1 = wm.step(s, torch.zeros(2, 31))
    p, v, q, qd = s[0].clone(), s[2].clone(), s[4].clone(), s[5].clone()
    rotation = s[1].clone()
    w = s[3].clone()
    for _ in range(10):
        v += delta[:3] / 10
        w += delta[3:6] / 10
        qd += delta[6:37] / 10
        p += v * .002
        q += qd * .002
        rotation = so3_exp(w * .002) @ rotation
    for expected, actual in zip((p, rotation, v, w, q, qd, s[6]+delta[37:]), s1):
        torch.testing.assert_close(actual, expected, rtol=1e-5, atol=1e-7)
    torch.testing.assert_close(wm.targets(s, s1), delta.expand(2, -1))


def test_zero_increment_preserves_velocity_instead_of_freezing_position():
    wm = WorldModel(hidden=(8,), dynamics_mode='acceleration_v2')
    s = state()
    s1 = wm.step(s, torch.zeros(2, 31))
    torch.testing.assert_close(s1[0], s[0] + .02*s[2])
    torch.testing.assert_close(s1[4], s[4] + .02*s[5])
    assert wm.out_dim == 37


def test_physical_loss_supervises_root_velocity_and_memory():
    wm = WorldModel(hidden=(8,), memory_dim=2, dynamics_mode='acceleration_v2')
    truth = state(memory=2)
    predicted = tuple(t.clone() for t in truth)
    predicted[2][:, 0] += .1
    predicted[6][:, 0] += .1
    error = wm.weighted_sq_error(predicted, truth)
    torch.testing.assert_close(error.sum(-1), torch.full((2,), 2.))
    assert wm.nominal_state_scale().numel() == 76


def fake_motion():
    m = object.__new__(MotionLib)
    m.state_dependent_goal=True
    m.goal_stride=5
    m.clip_id=torch.cat((torch.zeros(60,dtype=torch.long),torch.ones(10,dtype=torch.long)))
    m.clip_end=torch.tensor([59,69])
    m.root_R=torch.eye(3).expand(70,3,3).clone()
    m.root_p=torch.arange(70.).unsqueeze(-1).expand(70,3)
    m.q=torch.zeros(70,31)
    m.qd=torch.ones(70,31)
    m.root_v_local=torch.ones(70,3)
    m.root_w_local=torch.ones(70,3)
    return m


def test_goals_use_current_robot_orientation_and_do_not_cross_clips():
    m=fake_motion()
    s=state(batch=1)
    s[1].copy_(so3_exp(torch.tensor([[0.,0.,.4]])))
    f=torch.tensor([50])
    goal=m.goal(f,10,state=s).reshape(1,10,77)
    assert m.future_frames(f,10).tolist()==[[51,56,59,59,59,59,59,59,59,59]]
    torch.testing.assert_close(goal[0,0,-6:],mat_to_6d(s[1].transpose(-1,-2))[0])
    assert torch.count_nonzero(goal[:,2:,31:62])==0
    assert torch.count_nonzero(goal[:,2:,65:71])==0
    with pytest.raises(ValueError,match='actual current state'):
        m.goal(f,10)


def test_teacher_depends_on_current_pose_and_has_no_gradient():
    learner=object.__new__(VelocityLearner)
    learner.motion=fake_motion()
    learner.args=SimpleNamespace(goal_frames=10)
    learner.pi=SimpleNamespace(shared=SharedController(hidden=(16,),normalized_gelu=True))
    s=state(batch=1)
    frame=torch.tensor([0])
    original=learner.alignment_target(s,frame)
    s=list(s)
    s[1]=so3_exp(torch.tensor([[0.,0.,.5]],requires_grad=True))
    changed=learner.alignment_target(tuple(s),frame)
    assert not original.requires_grad and not changed.requires_grad
    assert not torch.allclose(original,changed,atol=1e-6,rtol=0)
    assert all(p.grad is None for p in learner.pi.shared.motion_encoder.parameters())


def test_imu_site_observation_keeps_measured_joints_and_raw_previous_action():
    mdp=object.__new__(MotionTrackingMDP)
    mdp.imu_frame='imu_site'
    mdp.default_q=torch.zeros(31)
    gyro=torch.tensor([[.1,.2,.3]])
    gravity=torch.tensor([[.2,.1,-.97]])
    mdp.kin=SimpleNamespace(imu=lambda *args:(None,gyro,gravity))
    s=state(batch=1)
    previous=torch.full((1,31),.7)
    from opensprite_tracker.supermimic.robot import CONTEXT_DIM
    result=mdp.obs(s,previous,torch.zeros(1,CONTEXT_DIM),torch.zeros(1,dtype=torch.long))
    torch.testing.assert_close(result[:,:31],s[4])
    torch.testing.assert_close(result[:,31:62],s[5])
    torch.testing.assert_close(result[:,62:65],gyro)
    torch.testing.assert_close(result[:,65:68],gravity)
    torch.testing.assert_close(result[:,68:],previous)


def test_normalized_gelu_velocity_gradient_crosses_frozen_decoder():
    shared=SharedController(hidden=(16,8),normalized_gelu=True)
    history=torch.randn(2,10,99,requires_grad=True)
    for p in shared.decoder.parameters():p.requires_grad_(False)
    action=shared.decode(history,shared.velocity_latent(history,torch.ones(2,3)))
    action.square().sum().backward()
    assert history.grad.abs().sum()>0
    assert all(p.grad is None for p in shared.decoder.parameters())
    assert any(p.grad is not None and p.grad.abs().sum()>0 for p in shared.velocity_encoder.parameters())
    assert any(isinstance(layer,nn.LayerNorm) for layer in shared.decoder)


def test_new_world_model_loss_preserves_global_valid_denominator():
    whole=make_learner(False)
    torch.manual_seed(33)
    whole.wm=WorldModel(hidden=(16,16,16),context_dim=2,dynamics_mode='acceleration_v2')
    whole.opt_wm=torch.optim.Adam(whole.wm.parameters(),lr=whole.args.lr)
    partitioned=copy.deepcopy(whole)
    partitioned.args.wm_microbatch=3
    a=whole.wm_update()
    b=partitioned.wm_update()
    assert_modules_equal(whole.wm,partitioned.wm)
    for x,y in zip(a,b):torch.testing.assert_close(x,y)


def test_critic_bound_retains_negative_values():
    learner=object.__new__(Learner)
    learner.args=SimpleNamespace(critic_upper_bound=True,gamma=.99)
    learner.mdp=SimpleNamespace(reward_scale=.08)
    torch.testing.assert_close(learner._bounded_value(torch.tensor([-12.,2.,20.])),torch.tensor([-12.,2.,8.]))


def test_revision_requires_explicit_semantics_not_just_input_width():
    cfg=dict(algorithm_revision='cz_official_aligned_rl_v2')
    with pytest.raises(ValueError,match='IMU frame'):revision_contract(cfg)
    cfg['imu_frame']='root'
    with pytest.raises(ValueError,match='Incomplete v2'):revision_contract(cfg)
    assert revision_contract({})['schema']=='sprite_v5_joint_supermimic_v1'
