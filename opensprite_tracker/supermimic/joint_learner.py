"""Velocity imagined rollouts with stored commands and reference-only alignment."""
import torch
from torch import nn
from .engine import Learner,actor_return,set_grad
from .state import unflatten_state

def draw_commands(count,ranges,device):
    lo=torch.tensor([r[0] for r in ranges],device=device)
    hi=torch.tensor([r[1] for r in ranges],device=device)
    commands=lo+torch.rand(count,3,device=device)*(hi-lo)
    return torch.where((torch.rand(count,device=device)<.1)[:,None],torch.zeros_like(commands),commands)

class VelocityLearner(Learner):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.velocity_step=torch.compile(self._velocity_step) if self.args.compile else self._velocity_step

    def _velocity_step(self,state,history,previous,frame,xi,xi_n,command):
        shared=self.pi.shared
        latent=shared.velocity_latent(history,command)
        action=shared.decode(history,latent).clamp(-20,20)
        next_state=self.wm.step(state,action,xi_n)
        next_frame=torch.minimum(frame+1,self.motion.clip_end[self.motion.clip_id[frame]])
        reward=self.mdp.reward_for_command(next_state,command,action,previous)
        death=self.mdp.soft_termination(next_state,next_frame)
        obs=self.mdp.add_noise(self.mdp.obs(next_state,action,xi,next_frame))
        return next_state,torch.cat((history[:,1:],obs[:,None]),1),action,next_frame,reward,death,latent

    def _policy_rollout(self,start,history):
        args=self.args;shared=self.pi.shared
        state=initial_state=unflatten_state(start['s'])
        xi=start['xi'];xi_n=self.normalize_xi(xi)
        frame=start['f'];previous=start['a_prev']
        obs_history=self.mdp.add_noise(self.mdp.obs(unflatten_state(history['s']),history['a_prev'],history['xi'],history['f']))
        command=initial_goal=start['command']
        source=start['reference_command'].bool()
        age=start['command_age'].long()
        initial_frame=frame
        states=[];rewards=[];deaths=[];frames=[];goals=[];actions=[];alignment=[]
        for _ in range(args.horizon):
            next_state,next_history,action,next_frame,reward,death,latent=self.velocity_step(state,obs_history,previous,frame,xi,xi_n,command)
            target=self.teacher_latents[frame]
            alignment.append((latent-target).square().mean(-1)*source.float())
            age=age+1;resample=age>=250
            next_source=torch.where(resample,torch.rand_like(age.float())<.5,source)
            random_command=draw_commands(len(frame),args.command_ranges,frame.device)
            next_command=torch.where(resample[:,None],random_command,command)
            next_command=torch.where(next_source[:,None],self.motion.reference_commands[next_frame],next_command)
            age=torch.where(resample,torch.zeros_like(age),age)
            obs_history=next_history
            states.append(next_state);rewards.append(reward);deaths.append(death);frames.append(next_frame);goals.append(next_command);actions.append(action)
            state,frame,previous,command,source=next_state,next_frame,action,next_command,next_source
        rewards=torch.stack(rewards);deaths=torch.stack(deaths)
        tail=self.critic(self.critic_input(state,frame,xi_n,command))
        objective,alive_now,alive_next=actor_return(rewards,deaths,tail,args.gamma)
        # Weight 0.1; no gradient into the motion encoder or shared decoder weights.
        objective=objective-.1*torch.stack(alignment).mean(0)
        with torch.no_grad():
            combined=tuple(torch.cat((initial_state[i][None],torch.stack([s[i].detach() for s in states]))).flatten(0,1) for i in range(len(state)))
            all_frames=torch.cat((initial_frame[None],torch.stack(frames))).flatten()
            all_goals=torch.cat((initial_goal[None],torch.stack(goals))).flatten(0,1)
            context=xi_n[None].expand(args.horizon+1,-1,-1).flatten(0,1)
            features=self.critic_input(combined,all_frames,context,all_goals).view(args.horizon+1,len(frame),-1)
        return objective,alive_now,alive_next,rewards,deaths,features,torch.stack(actions).detach().abs().amax()

def joint_policy_update(motion_learner,velocity_learner,shared):
    """Both halves imagine with the same pre-update weights, then one actor step.

    Separate optimizer parameter groups are disjoint; only their union is
    gradient-clipped. Critic targets come from these same pre-update rollouts.
    """
    motion_learner.opt_pi.zero_grad(set_to_none=True)
    velocity_learner.opt_pi.zero_grad(set_to_none=True)
    frozen=[motion_learner.wm,motion_learner.critic,velocity_learner.critic]
    # Exact cache: teacher depends only on recorded reference frame and the
    # current pre-update encoder. It is never reused across optimizer steps.
    with torch.no_grad():
        frames=torch.arange(motion_learner.motion.num_frames,device=next(shared.parameters()).device)
        velocity_learner.teacher_latents=shared.motion_latent(motion_learner.motion.goal(frames,10))
    for module in frozen:set_grad(module,False)
    critic_batches=[];metrics={}
    try:
        for label,learner in [('motion',motion_learner),('velocity',velocity_learner)]:
            if label=='velocity':
                set_grad(shared.decoder,False);set_grad(shared.motion_encoder,False)
            start,history=learner.sample_valid_starts();B=learner.args.pi_batch
            X=R=D=None;loss_sum=torch.zeros((),device=start['s'].device)
            reward_sum=loss_sum.clone();survival_sum=loss_sum.clone()
            for chunk in learner._chunks(B,learner.args.pi_microbatch):
                part={k:v[chunk] for k,v in start.items()};hist={k:v[chunk] for k,v in history.items()}
                objective,alive,alive_next,rewards,deaths,features,peak=learner._policy_rollout(part,hist)
                loss=-.5*objective.sum()/B
                loss.backward();loss_sum+=loss.detach()
                reward_sum+=(alive*rewards).detach().sum()/(learner.args.horizon*B)
                survival_sum+=alive_next[-1].detach().sum()/B
                if X is None:
                    X=torch.empty(learner.args.horizon+1,B,features.shape[-1],device=features.device)
                    R=torch.empty(learner.args.horizon,B,device=rewards.device);D=torch.empty_like(R)
                X[:,chunk].copy_(features.detach());R[:,chunk].copy_(rewards.detach());D[:,chunk].copy_(deaths.detach())
                del objective,alive,alive_next,rewards,deaths,features,loss
            metrics[label+'/actor_loss']=float(loss_sum)
            metrics[label+'/imagined_reward']=float(reward_sum/learner.mdp.reward_scale)
            metrics[label+'/imagined_survival']=float(survival_sum)
            critic_batches.append((learner,X,R,D))
        norm=nn.utils.clip_grad_norm_(shared.parameters(),1.)
        nn.utils.clip_grad_value_(shared.parameters(),.1)
        if not torch.isfinite(norm):raise RuntimeError('Nonfinite joint policy gradient')
        motion_learner.opt_pi.step();velocity_learner.opt_pi.step()
        metrics['actor_grad_norm']=float(norm)
    finally:
        for module in frozen:set_grad(module,True)
        set_grad(shared.decoder,True);set_grad(shared.motion_encoder,True)
    for label,(learner,X,R,D) in zip(['motion','velocity'],critic_batches):
        metrics[label+'/critic_loss']=float(learner._critic_update(X,R,D))
    return metrics
