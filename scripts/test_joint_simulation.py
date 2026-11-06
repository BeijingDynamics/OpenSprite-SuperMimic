"""Small real-physics dual-interface gradient test using an explicitly invalid reference fixture.

This fixture is never accepted by reference QA and never launches a long run.
"""
import sys,json
from pathlib import Path
from types import SimpleNamespace
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from train_joint import MixedReplay,reference_commands
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits,CONTEXT_DIM,MEM_DIM
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.env import TrackingSim
from opensprite_tracker.supermimic.networks import WorldModel,Critic
from opensprite_tracker.supermimic.buffer import ReplayBuffer
from opensprite_tracker.supermimic.engine import Learner
from opensprite_tracker.supermimic.state import unflatten_state
from opensprite_tracker.supermimic.joint_interfaces import SharedController,MotionActor,VelocityActor,MotionTrackingMDP,VelocityMDP
from opensprite_tracker.supermimic.joint_learner import VelocityLearner,joint_policy_update

def check():
    torch.set_num_threads(4);torch.manual_seed(42);device='cuda:0';N=16
    kin=make_kinematics(device,True)
    fixture=MotionLib(ROOT/'data/motions/v5_command_seed.npz',kin,device)
    fixture.reference_commands=reference_commands(fixture)
    mdps=[MotionTrackingMDP(kin,fixture,default_joint_pos(device),velocity_limits(device)),VelocityMDP(kin,fixture,default_joint_pos(device),velocity_limits(device))]
    shared=SharedController(hidden=(64,32)).to(device);actors=[MotionActor(shared),VelocityActor(shared)]
    world=WorldModel(hidden=(128,128,128),context_dim=CONTEXT_DIM,memory_dim=MEM_DIM).to(device)
    args=SimpleNamespace(lr=.0002,compile=False,memory_dim=MEM_DIM,pi_batch=8,pi_microbatch=4,hist=10,horizon=4,goal_frames=10,
        gamma=.99,lam=.95,wm_window=8,wm_batch=8,wm_microbatch=4,wm_loss_scale=1.,failure_replay_fraction=0.,
        critic_storage='cuda',critic_microbatch=32,critic_epochs=5,critic_minibatches=4,command_ranges=[[-.3,.6],[-.2,.2],[-.6,.6]])
    learners=[];envs=[];buffers=[]
    for i in range(2):
        env=TrackingSim(N,fixture,mdps[i],device,seed=42+i,randomize=False,reset_noise=False,full_body_collision=True,torque_lag_enabled=False);envs.append(env)
        extras={'command':(3,torch.float32),'reference_command':((),torch.bool),'command_age':((),torch.int32)} if i else None
        buffer=ReplayBuffer(64,N,device,state_dim=75+MEM_DIM,context_dim=CONTEXT_DIM,extra_fields=extras,storage_device='cuda');buffers.append(buffer)
        dim=mdps[i].critic_features(env.state(),env.f,env.normalized_xi).shape[-1]+(3 if i else 770)
        learner=(VelocityLearner if i else Learner)(world,actors[i],Critic(dim).to(device),mdps[i],fixture,buffer,env.normalize_xi,args);learners.append(learner)
        with torch.no_grad():
            for t in range(20):
                obs=mdps[i].obs(env.state(),env.a_prev,env.xi,env.f);shared.obs_norm.update(obs)
                goal=fixture.reference_commands[env.f] if i else fixture.goal(env.f,10)
                (shared.command_norm if i else shared.motion_norm).update(goal)
                if i:mdps[i].active_command=goal
                out=env.step(torch.randn(N,31,device=device)*.1)
                if i:out.update(command=goal.clone(),reference_command=torch.ones(N,dtype=torch.bool,device=device),command_age=torch.full((N,),t,dtype=torch.int32,device=device))
                buffer.add(out)
                before,after=unflatten_state(out['s']),unflatten_state(out['s1'])
                world.in_norm.update(world.inputs(before,out['a'],env.normalize_xi(out['xi'])))
                world.out_norm.update(world.targets(before,after))
    learners[0].buffer=MixedReplay(*buffers);wm_loss,_=learners[0].wm_update();learners[0].buffer=buffers[0]
    assert torch.isfinite(wm_loss)
    metrics=joint_policy_update(*learners,shared)
    assert all(torch.isfinite(torch.tensor(v)) for v in metrics.values())
    assert len(learners[0].opt_pi.state)>0 and len(learners[1].opt_pi.state)>0
    for env in envs:env.close()
    report=dict(passed=True,world_model_shared=True,mixed_replay_wm_step=True,both_interfaces_updated=True,
                both_critics_updated=True,fixture_is_valid_training_reference=False,metrics=metrics)
    (ROOT/'joint_simulation_test.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
if __name__=='__main__':check()
