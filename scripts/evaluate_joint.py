"""Physical evaluation of the jointly trained interfaces, with no failure resets."""
import argparse,json,sys,time,hashlib
from pathlib import Path
import numpy as np,torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from train_joint import reference_commands
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits,physical_contract
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.env import TrackingSim
from opensprite_tracker.supermimic.joint_interfaces import SharedController,MotionActor,VelocityActor,MotionTrackingMDP,VelocityMDP
from opensprite_tracker.supermimic.rotations import mat_to_quat,yaw_mat

@torch.no_grad()
def evaluate(args):
    torch.set_num_threads(4);torch.manual_seed(args.seed)
    ck=torch.load(args.checkpoint,map_location=args.device,weights_only=False)
    if ck.get('schema')!='sprite_v5_joint_supermimic_v1':raise ValueError('Not a joint-interface policy')
    if ck['motion_sha256']!=hashlib.sha256(args.motion.read_bytes()).hexdigest():raise ValueError('Reference bank changed')
    if ck['physical_contract']!=physical_contract(torque_lag_enabled=False):raise ValueError('Physics contract changed')
    shared=SharedController(hidden=tuple(ck['config']['network_hidden'])).to(args.device);shared.load_state_dict(ck['shared']);shared.eval()
    velocity=args.interface=='velocity'
    actor=VelocityActor(shared) if velocity else MotionActor(shared)
    kin=make_kinematics(args.device,True);motion=MotionLib(args.motion,kin,args.device);motion.reference_commands=reference_commands(motion)
    mdp=(VelocityMDP if velocity else MotionTrackingMDP)(kin,motion,default_joint_pos(args.device),velocity_limits(args.device))
    default_commands=[[0,0,0],[.15,0,0],[.3,0,0],[.15,0,.15],[.15,0,-.15],[.3,0,.15],[.3,0,-.15]]
    cases=json.loads(args.commands_file.read_text()) if args.commands_file else [dict(name='case_'+str(i),command=c) for i,c in enumerate(default_commands)]
    if not isinstance(cases,list) or not cases:raise ValueError('Command cases must be a nonempty list')
    commands=torch.tensor([c['command'] for c in cases],device=args.device,dtype=torch.float32)
    if commands.shape!=(len(cases),3) or not torch.isfinite(commands).all():raise ValueError('Each command must be finite [vx,vy,yaw_rate]')
    stages=[]
    for c in cases:
        rows=c.get('stages',[])
        if rows and (rows[0]['at_seconds']!=2 or any(b['at_seconds']<=a['at_seconds'] for a,b in zip(rows,rows[1:]))):raise ValueError('Stages must start at2seconds and increase')
        stages.append([(round(row['at_seconds']/.02),torch.tensor(row['command'],device=args.device,dtype=torch.float32)) for row in rows])
    N=len(commands) if velocity else motion.num_clips
    env=TrackingSim(N,motion,mdp,args.device,seed=args.seed,eval_mode=True,auto_reset=False,randomize=False,reset_noise=False,full_body_collision=True,torque_lag_enabled=False)
    if velocity:
        # A real quiet reference pose, with physical velocities zeroed, not a moving reset.
        quiet=int(motion.root_v[:,:2].norm(dim=-1).argmin())
        env.reset(frames=torch.full((N,),quiet,device=args.device,dtype=torch.long))
        S=env.state();initial_p=S[0].clone();initial_R=S[1].clone();initial_q=S[4].clone()
        if args.initial_pose!='reference-quiet':
            initial_q=torch.zeros_like(S[4]) if args.initial_pose=='urdf-zero' else default_joint_pos(args.device)[None].expand_as(S[4]).clone()
            initial_p=torch.zeros_like(S[0]);initial_R=torch.eye(3,device=args.device)[None].expand_as(S[1]).clone()
            bottom=kin.ground_clearance(initial_p,initial_R,torch.zeros_like(S[2]),torch.zeros_like(S[3]),initial_q,torch.zeros_like(S[5]))
            initial_p[:,2]=.001-bottom
        root=torch.cat((initial_p+env.scene.env_origins,mat_to_quat(initial_R),torch.zeros(N,6,device=args.device)),-1)
        env.robot.write_root_state_to_sim(root)
        env.robot.write_joint_state_to_sim(initial_q,torch.zeros_like(S[5]),joint_ids=env.joint_ids)
        env.refresh()
        initial_description=dict(mode=args.initial_pose,reference_frame=quiet if args.initial_pose=='reference-quiet' else None,
            root_position_m=initial_p[0].cpu().tolist(),root_quaternion_wxyz=mat_to_quat(initial_R)[0].cpu().tolist(),joint_position_rad=initial_q[0].cpu().tolist(),
            ground_placement='root translated vertically until minimum active collider is1mm above flat floor' if args.initial_pose!='reference-quiet' else 'reference pose with reset ground-clearance correction')
        steps=round(args.seconds/.02)
    else:
        env.reset(frames=motion.clip_start);steps=int(motion.clip_len.max())-1
        initial_description=dict(mode='reference-clip-first-frame')
    hist=mdp.obs(env.state(),env.a_prev,env.xi,env.f)[:,None].expand(-1,10,-1).clone()
    first=torch.full((N,),-1,device=args.device,dtype=torch.long)
    totals=torch.zeros(N,4,device=args.device);counts=torch.zeros(N,device=args.device)
    saved={k:[] for k in ['qpos','commands','velocity','yaw_rate','failed','contact_force_mean_n','applied_torque_peak','raw_torque_peak','proprioception','policy_action']}
    started=time.monotonic()
    for t in range(steps):
        obs=mdp.obs(env.state(),env.a_prev,env.xi,env.f)
        if t:hist=torch.cat((hist[:,1:],obs[:,None]),1)
        if velocity:
            goal=torch.zeros_like(commands) if t<100 else commands
            if t>=100 and any(stages):
                goal=commands.clone()
                for i,rows in enumerate(stages):
                    for begin,value in rows:
                        if t>=begin:goal[i]=value
            mdp.active_command=goal
        else:goal=motion.goal(env.f,10)
        action=actor(hist,goal).clamp(-20,20)
        out=env.step(action);S=out['S1']
        active=torch.ones(N,dtype=torch.bool,device=args.device) if velocity else t<motion.clip_len-1
        failure=out['term']&active
        first=torch.where((first<0)&failure,torch.full_like(first,t),first)
        local=(yaw_mat(S[1]).transpose(-1,-2)@S[2][...,None]).squeeze(-1)
        cmd=goal if velocity else motion.reference_commands[out['f1']]
        valid=active&(first<0)&(t>=150 if velocity else True)
        if velocity:
            values=torch.cat(((local[:,:2]-cmd[:,:2]).norm(dim=-1,keepdim=True),(S[3][:,2:3]-cmd[:,2:3]).abs(),local[:,:1],S[3][:,2:3]),1)
        else:
            joint=(S[4]-motion.q[out['f1']]).square().mean(-1).sqrt()
            root=(S[0]-motion.root_p[out['f1']]).norm(dim=-1)
            values=torch.stack((joint,root,local[:,0],S[3][:,2]),-1)
        totals+=valid[:,None]*values;counts+=valid
        fields=[torch.cat((S[0],mat_to_quat(S[1]),S[4]),-1),cmd,local,S[3][:,2],first>=0,out['contact_mean_n'],out['applied_torque_peak'],out['raw_torque_peak'],obs,action]
        for key,value in zip(saved,fields):saved[key].append(value.cpu().numpy())
        if t%250==0:print('JOINT_EVAL',args.interface,t,steps,'alive',int((first<0).sum()),flush=True)
        if not torch.isfinite(out['s1']).all():raise RuntimeError('Nonfinite physical state')
    args.output.mkdir(parents=True,exist_ok=False)
    per=[]
    for i in range(N):
        item=dict(env=i,label=commands[i].cpu().tolist() if velocity else motion.clip_names[i],survived=bool(first[i]<0),
                  first_failure_seconds=None if first[i]<0 else float((first[i]+1)*.02),scored_steps=int(counts[i]))
        if velocity:
            ranges=ck['config']['command_ranges'];command=commands[i].cpu().tolist()
            item.update(case_name=cases[i]['name'],command_category=cases[i].get('category','unspecified'),
                within_training_random_command_range=all(lo-1e-6<=value<=hi+1e-6 for value,(lo,hi) in zip(command,ranges)),stages=cases[i].get('stages',[]))
        keys=['velocity_error_m_s','yaw_error_rad_s','mean_vx_m_s','mean_yaw_rad_s'] if velocity else ['joint_rmse_rad','root_error_m','mean_vx_m_s','mean_yaw_rad_s']
        for k,v in zip(keys,totals[i]):item[k]=float(v/counts[i]) if counts[i]>0 else None
        if velocity:
            valid_end=min(steps,int(first[i]) if first[i]>=0 else steps)
            tail_begin=max(150,valid_end-250)
            if valid_end>tail_begin:
                measured=np.asarray(saved['velocity'])[tail_begin:valid_end,i];target=np.asarray(saved['commands'])[tail_begin:valid_end,i]
                yaw=np.asarray(saved['yaw_rate'])[tail_begin:valid_end,i]
                item.update(last5s_mean_velocity_m_s=measured.mean(0).tolist(),last5s_mean_yaw_rad_s=float(yaw.mean()),
                    last5s_velocity_error_m_s=float(np.linalg.norm(measured[:,:2]-target[:,:2],axis=-1).mean()),last5s_yaw_error_rad_s=float(abs(yaw-target[:,2]).mean()))
        per.append(item)
    report=dict(interface=args.interface,iteration=ck['iteration'],seed=args.seed,steps=steps,seconds=steps*.02,results=per,
                initialization=initial_description,
                training_random_command_ranges=ck['config']['command_ranges'],
                policy_dt=.02,physics_dt=.002,history=10,checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
                protocol='No failure reset; velocity starts from physical rest and steps command after 2s; motion starts at each source first frame; pre-failure scoring only',
                hardware_qualified=False,wall_seconds=time.monotonic()-started)
    (args.output/'summary.json').write_text(json.dumps(report,indent=2));np.savez_compressed(args.output/'rollout.npz',**{k:np.array(v) for k,v in saved.items()},fps=np.array(50.),case_names=np.array([c['name'] for c in cases]) if velocity else np.array(motion.clip_names))
    env.close();print(json.dumps(report,indent=2),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--motion',type=Path,required=True)
    p.add_argument('--interface',choices=['motion','velocity'],required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=30);p.add_argument('--seed',type=int,default=101);p.add_argument('--device',default='cuda:0')
    p.add_argument('--initial-pose',choices=['reference-quiet','urdf-zero','controller-default'],default='reference-quiet')
    p.add_argument('--commands-file',type=Path);evaluate(p.parse_args())
