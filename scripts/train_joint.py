"""Joint SuperMimic training, gated by real v5 reference motion QA."""
import argparse,csv,hashlib,json,math,os,random,signal,sys,time
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import torch
from scipy.ndimage import uniform_filter1d
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits,CONTEXT_DIM,MEM_DIM,physical_contract
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.env import TrackingSim
from opensprite_tracker.supermimic.networks import WorldModel,Critic
from opensprite_tracker.supermimic.buffer import ReplayBuffer
from opensprite_tracker.supermimic.engine import Learner
from opensprite_tracker.supermimic.joint_interfaces import SharedController,MotionActor,VelocityActor,MotionTrackingMDP,VelocityMDP
from opensprite_tracker.supermimic.joint_learner import VelocityLearner,joint_policy_update,draw_commands
from opensprite_tracker.supermimic.state import unflatten_state
from opensprite_tracker.supermimic.rotations import yaw_mat
from opensprite_tracker.supermimic.sampling import ReturnSampler

class MixedReplay:
    def __init__(self,a,b):self.buffers=[a,b]
    def sample_windows(self,B,k,**kw):
        halves=[b.sample_windows(B//2,k,**kw) for b in self.buffers]
        keys=halves[0].keys()&halves[1].keys()
        return {key:torch.cat([h[key] for h in halves],0) for key in keys}

def reference_commands(motion):
    heading=yaw_mat(motion.root_R).transpose(-1,-2)
    local=(heading@motion.root_v[...,None]).squeeze(-1)
    raw=torch.cat((local[:,:2],motion.root_w[:,2:3]),-1).cpu().numpy()
    smoothed=np.empty_like(raw)
    for begin,end in zip(motion.clip_start.tolist(),motion.clip_end.tolist()):
        smoothed[begin:end+1]=uniform_filter1d(raw[begin:end+1],size=25,axis=0,mode='nearest')
    return torch.as_tensor(smoothed,device=motion.q.device)

def train(cli):
    cfg=json.loads(cli.config.read_text())
    if cfg['training_pd_hz']!=500 or cfg['physics_dt']!=.002 or cfg['policy_dt']!=.02 or cfg['decimation']!=10:
        raise ValueError('Run002 requires explicit500Hz PD/50Hz policy/10substeps')
    from opensprite_tracker.robot import CONTROL_PROFILE_SHA256,ASSET_REVISION
    if cfg['control_profile_sha256']!=CONTROL_PROFILE_SHA256 or cfg['asset_revision']!=ASSET_REVISION:
        raise ValueError('Run002 asset/control profile changed')
    if cfg['velocity_fall_height_m']!=VelocityMDP.fall_height:raise ValueError('Velocity fall-height configuration/implementation mismatch')
    if cfg['velocity_reward_revision']!=VelocityMDP.reward_revision:raise ValueError('Velocity reward revision mismatch')
    if cli.smoke:cfg.update(num_envs=64,wm_batch=64,wm_microbatch=64,pi_batch_total=64,pi_microbatch=32,iterations=3,compile=False,warmup_steps=32,max_wall_seconds=600)
    report=json.loads(cli.quality.read_text())
    if report.get('accepted_for_training') is not True:raise ValueError('Reference motion QA has not accepted the bank')
    if not cli.smoke and report.get('visual_review_passed') is not True:
        raise ValueError('Long training requires rendered reference review in addition to numeric QA')
    digest=hashlib.sha256(cli.motion.read_bytes()).hexdigest()
    if report['asset_revision']!=ASSET_REVISION:raise ValueError('Reference QA belongs to a different asset')
    if digest!=report['motion_bank_sha256']:raise ValueError('Reference changed after QA')
    cli.output.mkdir(parents=True,exist_ok=False)
    (cli.output/'config.json').write_text(json.dumps(cfg,indent=2))
    (cli.output/'reference_qa.json').write_text(json.dumps(report,indent=2))
    torch.set_num_threads(4);torch.manual_seed(cfg['seed']);np.random.seed(cfg['seed']);random.seed(cfg['seed'])
    torch.set_float32_matmul_precision('highest');torch.backends.cuda.matmul.allow_tf32=False
    device=cli.device;N=cfg['num_envs']//2
    args=SimpleNamespace(**{k:v for k,v in cfg.items() if k not in ('pi_batch_total',)})
    args.pi_batch=cfg['pi_batch_total']//2;args.memory_dim=MEM_DIM;args.critic_storage='cuda';args.failure_replay_fraction=0.;args.wm_loss_scale=1.
    kin=make_kinematics(device,True);motion=MotionLib(cli.motion,kin,device)
    if max(v['max_abs'] for k,v in motion.source_fk_error.items() if k in ('body_p','body_R'))>1e-4:raise ValueError('Source v5 FK geometry mismatch')
    motion.reference_commands=reference_commands(motion)
    mdps=[MotionTrackingMDP(kin,motion,default_joint_pos(device),velocity_limits(device)),VelocityMDP(kin,motion,default_joint_pos(device),velocity_limits(device))]
    envs=[TrackingSim(N,motion,mdps[i],device,seed=cfg['seed']+i,randomize=True,reset_noise=True,full_body_collision=True,torque_lag_enabled=False) for i in range(2)]
    samplers=[ReturnSampler(motion,args.gamma,reward_scale=m.reward_scale) for m in mdps]
    for env,sampler in zip(envs,samplers):env.frame_sampler=sampler;env.clip_sampler=None
    shared=SharedController(hidden=tuple(cfg['network_hidden'])).to(device)
    actors=[MotionActor(shared),VelocityActor(shared)]
    world=WorldModel(hidden=(cfg['wm_width'],)*3,context_dim=CONTEXT_DIM,memory_dim=MEM_DIM).to(device)
    buffers=[];learners=[]
    for i in range(2):
        extras={'command':(3,torch.float32),'reference_command':((),torch.bool),'command_age':((),torch.int32)} if i else None
        buf=ReplayBuffer(cfg['buffer_steps'],N,device,state_dim=75+MEM_DIM,context_dim=CONTEXT_DIM,extra_fields=extras,storage_device='cuda')
        buffers.append(buf)
        goal=motion.goal(envs[i].f,10) if not i else motion.reference_commands[envs[i].f]
        dim=mdps[i].critic_features(envs[i].state(),envs[i].f,envs[i].normalized_xi).shape[-1]+goal.shape[-1]
        critic=Critic(dim).to(device)
        cls=Learner if not i else VelocityLearner
        learners.append(cls(world,actors[i],critic,mdps[i],motion,buf,envs[i].normalize_xi,args))
    mixed=MixedReplay(*buffers)
    start_iteration=0;wall_offset=0.;sim_steps=0
    if cli.resume is not None:
        old=torch.load(cli.resume,map_location=device,weights_only=False)
        if old.get('schema')!='sprite_v5_joint_supermimic_v1' or old['motion_sha256']!=digest:
            raise ValueError('Resume checkpoint method/reference mismatch')
        if old['config'].get('velocity_reward_revision','a6_root_legacy_v1')!=cfg['velocity_reward_revision']:
            raise ValueError('Reward changed; ordinary resume is invalid. Use a separately documented transfer experiment.')
        if old['physical_contract']!=physical_contract(torque_lag_enabled=False):
            raise ValueError('Resume robot or actuator contract changed')
        for key in ['num_envs','hist','goal_frames','horizon','wm_width','network_hidden','command_ranges']:
            if cfg[key]!=old['config'][key]:raise ValueError('Resume configuration changed: '+key)
        shared.load_state_dict(old['shared']);world.load_state_dict(old['wm'])
        learners[0].opt_wm.load_state_dict(old['opt_wm'])
        learners[0].opt_pi.load_state_dict(old['opt_motion']);learners[1].opt_pi.load_state_dict(old['opt_velocity'])
        for i,learner in enumerate(learners):
            learner.critic.load_state_dict(old['critics'][i]);learner.opt_v.load_state_dict(old['opt_critics'][i])
            samplers[i].returns.copy_(old['sampling'][i]['returns']);samplers[i].past.copy_(old['sampling'][i]['past'])
        start_iteration=int(old['iteration']);wall_offset=float(old['wall_seconds']);sim_steps=int(old['sim_steps'])
        (cli.output/'resume.json').write_text(json.dumps(dict(parent=str(cli.resume),parent_sha256=hashlib.sha256(cli.resume.read_bytes()).hexdigest(),
            prior_wall_seconds=wall_offset,start_iteration=start_iteration,rebuilds=['replay','live simulator','RNG']),indent=2))
    histories=[torch.zeros(N,10,99,device=device) for _ in range(2)]
    command=torch.zeros(N,3,device=device);source=torch.zeros(N,dtype=torch.bool,device=device);age=torch.zeros(N,dtype=torch.int32,device=device)
    mdps[1].active_command=command
    stats={}
    def metric(key,x):stats.setdefault(key,[]).append(float(x))
    @torch.no_grad()
    def collect(count):
        nonlocal sim_steps,command,source,age
        for _ in range(count):
            for i,(env,mdp,actor,buf,sampler) in enumerate(zip(envs,mdps,actors,buffers,samplers)):
                obs=mdp.add_noise(mdp.obs(env.state(),env.a_prev,env.xi,env.f))
                histories[i]=torch.where((env.ep_step==0)[:,None,None],obs[:,None].expand_as(histories[i]),torch.cat((histories[i][:,1:],obs[:,None]),1))
                shared.obs_norm.update(obs)
                if i:
                    resample=(env.ep_step==0)|(age>=250)
                    source.copy_(torch.where(resample,torch.rand(N,device=device)<.5,source))
                    random_cmd=draw_commands(N,args.command_ranges,device)
                    command.copy_(torch.where(resample[:,None],random_cmd,command))
                    command.copy_(torch.where(source[:,None],motion.reference_commands[env.f],command))
                    age.copy_(torch.where(resample,torch.zeros_like(age),age))
                    goal=command;shared.command_norm.update(goal);mdp.active_command=command
                else:goal=motion.goal(env.f,10);shared.motion_norm.update(goal)
                action=(actor(histories[i],goal)+cfg['action_noise']*torch.randn_like(env.a_prev)).clamp(-20,20)
                out=env.step(action)
                if i:out.update(command=command.clone(),reference_command=source.clone(),command_age=age.clone());age+=1
                sampler.update(out);buf.add(out)
                before,after=unflatten_state(out['s']),unflatten_state(out['s1'])
                world.in_norm.update(world.inputs(before,out['a'],env.normalize_xi(out['xi'])))
                world.out_norm.update(world.targets(before,after));sim_steps+=N
                label='velocity' if i else 'motion'
                metric(label+'/reward',out['r'].mean()/mdp.reward_scale)
                metric(label+'/terminated',out['term'].float().mean())
                if 'ep_len' in out:metric(label+'/episode_length',out['ep_len'].float().mean())
    stop=False
    def stopping(signum,frame):
        nonlocal stop;stop=True
    signal.signal(signal.SIGTERM,stopping);signal.signal(signal.SIGINT,stopping)
    started=time.monotonic();last_save=started;last_log=started;last_it=start_iteration;step_acc=0.
    collect(cfg['warmup_steps'])
    # Recording this contract does not grant hardware admission.
    (cli.output/'interface.json').write_text(json.dumps(dict(history=10,proprioception=99,velocity_actor_input=993,motion_goal=770,latent=32,
        split_envs=[N,N],split_imagined_batch=[args.pi_batch,args.pi_batch],shared_world_model=True,shared_decoder=True,
        velocity_decoder_weight_gradient=False,reference_latent_alignment_weight=.1,physics=physical_contract(torque_lag_enabled=False),
        reference_commands='centered 25-frame mean, per clip',reference_sha256=digest,hardware_qualified=False),indent=2))
    def save(iteration):
        payload=dict(schema='sprite_v5_joint_supermimic_v1',config=cfg,iteration=iteration,sim_steps=sim_steps,wall_seconds=time.monotonic()-started+wall_offset,
                     shared=shared.state_dict(),wm=world.state_dict(),critics=[x.critic.state_dict() for x in learners],
                     opt_wm=learners[0].opt_wm.state_dict(),opt_motion=learners[0].opt_pi.state_dict(),opt_velocity=learners[1].opt_pi.state_dict(),
                     opt_critics=[x.opt_v.state_dict() for x in learners],sampling=[dict(returns=x.returns,past=x.past) for x in samplers],
                     motion_sha256=digest,physical_contract=physical_contract(torque_lag_enabled=False),
                     resume_semantics='Replay, RNG and live simulator state not serialized; no bitwise resume claim')
        dest=cli.output/f'model_{iteration:06d}.pt';tmp=dest.with_suffix('.pt.tmp');torch.save(payload,tmp);tmp.replace(dest)
        print('SAVED',dest,flush=True)
        # Local synchronization runs every five minutes. Keep forty minutes of
        # recent checkpoints plus the first; hourly audits retain best policies.
        checkpoints=sorted(cli.output.glob('model_*.pt'))
        for obsolete in checkpoints[1:-4]:obsolete.unlink()
    with (cli.output/'metrics.jsonl').open('a',buffering=1) as log:
        for it in range(start_iteration,cfg['iterations']):
            lr=args.lr*(.03+.97*.5*(1+math.cos(math.pi*min(it,250000)/250000)))
            for opt in [learners[0].opt_wm,learners[0].opt_pi,learners[1].opt_pi,*[x.opt_v for x in learners]]:
                for group in opt.param_groups:group['lr']=lr
            step_acc+=1/args.utd;count=int(step_acc);step_acc-=count;collect(count)
            old=learners[0].buffer;learners[0].buffer=mixed
            try:
                for _ in range(args.wm_batches):
                    loss,error=learners[0].wm_update();metric('wm/loss',loss);metric('wm/error_last',error)
            finally:learners[0].buffer=old
            for key,value in joint_policy_update(*learners,shared).items():metric(key,value)
            now=time.monotonic();finished=(it+1==cfg['iterations']) or stop or now-started+wall_offset>=cfg['max_wall_seconds']
            if it==0 or (it+1)%cfg['log_every']==0 or finished:
                row={k:sum(v)/len(v) for k,v in stats.items()};stats.clear()
                row.update(iteration=it+1,wall_seconds=now-started+wall_offset,iterations_per_second=(it+1-last_it)/(now-last_log),sim_steps=sim_steps,peak_gpu_gib=torch.cuda.max_memory_allocated()/2**30)
                bad=[k for k,v in row.items() if isinstance(v,(int,float)) and not math.isfinite(v)]
                if bad:
                    (cli.output/'numerical_failure.json').write_text(json.dumps(dict(iteration=it+1,nonfinite=bad),indent=2))
                    raise FloatingPointError('Nonfinite training telemetry: '+str(bad))
                log.write(json.dumps(row)+'\n');print('JOINT_UPDATE',json.dumps(row),flush=True);last_log=now;last_it=it+1
            if it==0 or now-last_save>=cfg['save_seconds'] or finished:save(it+1);last_save=now
            if finished:break
    for env in envs:env.close()
    print('JOINT_TRAINING_COMPLETE',it+1,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--config',type=Path,default=ROOT/'configs/joint-paper.json');p.add_argument('--motion',type=Path,required=True)
    p.add_argument('--quality',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--device',default='cuda:0');p.add_argument('--smoke',action='store_true');p.add_argument('--resume',type=Path)
    train(p.parse_args())
