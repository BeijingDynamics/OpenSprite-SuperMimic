"""Closed-loop command evaluation from rest; no resets after failure."""
import argparse,json,sys,time
from pathlib import Path
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.velocity import VelocityMDP,VelocityRewardCfg
from opensprite_tracker.supermimic.env import TrackingSim
from opensprite_tracker.supermimic.networks import Policy
from opensprite_tracker.supermimic.rotations import mat_to_quat,yaw_mat

def load_policy(path,device):
    ck=torch.load(path,map_location=device,weights_only=False)
    args=ck['args']
    policy=Policy(99,3,hist=args['hist'],hidden=args['policy_hidden']).to(device)
    policy.load_state_dict(ck['pi']);policy.eval()
    return policy,ck

@torch.no_grad()
def evaluate(args):
    torch.set_num_threads(4);torch.manual_seed(args.seed)
    policy,ck=load_policy(args.checkpoint,args.device)
    kin=make_kinematics(args.device,True)
    motion=MotionLib(ROOT/'data/motions/v5_command_seed.npz',kin,args.device)
    mdp=VelocityMDP(kin,motion,default_joint_pos(args.device),velocity_limits(args.device),VelocityRewardCfg(**{k:v for k,v in ck['reward_config'].items() if k!='profile'}))
    N=7
    env=TrackingSim(N,motion,mdp,args.device,seed=args.seed,eval_mode=True,reset_noise=False,randomize=False,auto_reset=False,full_body_collision=True,torque_lag_enabled=False)
    # All environments begin in the same zero-velocity stand pose, not a moving reference reset.
    env.reset(frames=torch.zeros(N,dtype=torch.long,device=args.device))
    f0=motion.clip_start
    history=mdp.obs(env.state(),env.a_prev,env.xi,env.f)[:,None].expand(-1,policy.hist,-1).clone()
    first=torch.full((N,),-1,device=args.device,dtype=torch.long)
    velocity_sum=torch.zeros(N,device=args.device);yaw_sum=torch.zeros_like(velocity_sum);counts=torch.zeros_like(velocity_sum)
    frames=[];commands=[];velocities=[];yaws=[];failures=[]
    steps=round(args.seconds/.02)
    start=time.monotonic()
    for t in range(steps):
        # Initial 2s stand; then individual straight/turn commands, preserving physical continuity.
        env.f.copy_(torch.zeros_like(f0) if t<100 else f0+min(t-100,900))
        command=motion.goal(env.f)
        obs=mdp.obs(env.state(),env.a_prev,env.xi,env.f)
        if t:history=torch.cat((history[:,1:],obs[:,None]),1)
        action=policy(history,command).clamp(-20,20)
        out=env.step(action);S=out['S1']
        failed=mdp.termination(S,env.f)
        first=torch.where((first<0)&failed,torch.full_like(first,t),first)
        local=(yaw_mat(S[1]).transpose(-1,-2)@S[2][...,None]).squeeze(-1)
        valid=(first<0)&(t>=150)
        velocity_sum+=valid*(local[:,:2]-command[:,:2]).norm(dim=-1)
        yaw_sum+=valid*(S[3][:,2]-command[:,2]).abs();counts+=valid
        frames.append(torch.cat((S[0],mat_to_quat(S[1]),S[4]),-1).cpu().numpy())
        commands.append(command.cpu().numpy());velocities.append(local.cpu().numpy());yaws.append(S[3][:,2].cpu().numpy());failures.append((first>=0).cpu().numpy())
        if t%250==0:print('COMMAND_EVAL',t,steps,'alive',int((first<0).sum()),flush=True)
        if not torch.isfinite(out['s1']).all():raise RuntimeError('Nonfinite physical evaluation state')
    args.output.mkdir(parents=True,exist_ok=False)
    per=[]
    for i in range(N):
        per.append(dict(command=motion.commands[motion.clip_start[i]].cpu().tolist(),survived=bool(first[i]<0),
                        first_failure_seconds=None if first[i]<0 else float((first[i]+1)*.02),
                        velocity_error_m_s=None if counts[i]==0 else float(velocity_sum[i]/counts[i]),
                        yaw_error_rad_s=None if counts[i]==0 else float(yaw_sum[i]/counts[i]),
                        scored_steps=int(counts[i])))
    summary=dict(checkpoint=str(args.checkpoint),iteration=ck['iteration'],seed=args.seed,seconds=args.seconds,
                 steps=steps,per_command=per,wall_seconds=time.monotonic()-start,
                 protocol='2s stand then command step; from rest; deterministic physics; no reset after failure; errors score pre-failure only',
                 hardware_qualified=False)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    np.savez_compressed(args.output/'rollout.npz',qpos=np.array(frames),commands=np.array(commands),
                        velocity=np.array(velocities),yaw_rate=np.array(yaws),failed=np.array(failures),fps=np.array(50.))
    print(json.dumps(summary,indent=2),flush=True);env.close()

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=20);p.add_argument('--seed',type=int,default=101);p.add_argument('--device',default='cuda:0')
    evaluate(p.parse_args())
