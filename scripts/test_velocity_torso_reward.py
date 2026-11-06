"""Independent A.6 regressions: moving the waist must affect torso terms."""
import math,sys,torch
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits,JOINT_NAMES
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.joint_interfaces import VelocityMDP

kin=make_kinematics('cpu',True);motion=MotionLib(ROOT/'data/motions/contact_v5_v11/v5_locomotion.npz',kin,'cpu')
mdp=VelocityMDP(kin,motion,default_joint_pos(),velocity_limits())
q=default_joint_pos()[None];zero=torch.zeros(1,3);R=torch.eye(3)[None];p=torch.tensor([[0.,0.,.46]])
state=(p,R,zero.clone(),zero.clone(),q.clone(),torch.zeros_like(q));action=torch.zeros_like(q);command=zero.clone()
reward=mdp.reward_for_command(state,command,action,action)
rotating=list(state);rotating[3]=torch.tensor([[0.,0.,.5]])
rotation_reward=mdp.reward_for_command(tuple(rotating),command,action,action)
assert torch.allclose(reward-rotation_reward,torch.tensor([2*(1-math.exp(-.25/.5))*.02]),atol=1e-6)
waist=JOINT_NAMES.index('waist_roll_joint');waist_motion=list(state);waist_motion[5]=state[5].clone();waist_motion[5][:,waist]=1.
waist_rate_reward=mdp.reward_for_command(tuple(waist_motion),command,action,action)
assert abs(float(reward-waist_rate_reward)-.05*.02)<1e-6,'Torso rate must use waist motion even when pelvis gyro is zero'
lean=list(state);lean[4]=state[4].clone();lean[4][:,waist]=.1
assert mdp.reward_for_command(tuple(lean),command,action,action)<reward,'Torso lean must include waist articulation'
print('A6_TORSO_HEADING_AND_YAW_SCALE_PASSED',flush=True)
