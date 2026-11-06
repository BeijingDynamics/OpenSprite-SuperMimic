"""Paper Appendices B/G: shared dynamics/decoder, two encoders and two critics.

No velocity-only pose-prior shortcut. Velocity gradients cannot update decoder
weights; they still propagate through its inputs and through frozen dynamics.
"""
import torch
from torch import nn
from .networks import mlp,RunningNorm,run_mlp
from .mdp import MDP,RewardCfg,CONTROL_DT
from .rotations import mat_to_6d,rot_angle_sq,yaw_mat
from .state import invariant_features
from .robot import CONTEXT_SLICES

HISTORY=10
OBS_DIM=99

def proprioception(S,a_prev,xi,default_q):
    _,R,_,w,q,qd=S[:6]
    gyro=(R.transpose(-1,-2)@w[...,None]).squeeze(-1)
    return torch.cat((q-default_q+xi[...,CONTEXT_SLICES['encoder_bias']],qd,gyro,-R[...,2,:],a_prev),-1)

class MotionTrackingMDP(MDP):
    """Table A.2 reward and Appendix A termination, using v5 dimensions."""
    def __init__(self,kin,motion,default_q,velocity_limits):
        super().__init__(kin,motion,default_q,velocity_limits,RewardCfg(term_height=.25,term_tilt=.8))
        self.obs_noise=torch.tensor([.01]*31+[.5]*31+[.2]*3+[.05]*3+[0.]*31,device=default_q.device)
    def obs(self,S,a_prev,xi,f):return proprioception(S,a_prev,xi,self.default_q)
    def reward(self,S,f,a,a_prev,terms=False):
        p,R,_,_,q,_=S[:6]
        bp,bR,bv,bw,tp,tR=self._bodies_and_target(S,f)
        m=self.motion
        epos=(tp-bp).square().sum(-1).mean(-1)
        eori=rot_angle_sq(tR,bR).mean(-1)
        elv=(m.body_v[f]-bv).square().sum(-1).mean(-1)
        eav=(m.body_w[f]-bw).square().sum(-1).mean(-1)
        eroot=(m.root_p[f]-p).square().sum(-1)
        erot=rot_angle_sq(m.root_R[f],R)
        tracking=torch.exp(-epos/.3**2)+torch.exp(-eori/.4**2)+torch.exp(-elv/1.)+torch.exp(-eav/3.142**2)+.5*torch.exp(-eroot/.3**2)+.5*torch.exp(-erot/.4**2)
        limits=(torch.relu(q-self.soft_hi)+torch.relu(self.soft_lo-q)).sum(-1)
        penalty=.1*(a-a_prev).square().sum(-1)+2*limits
        reward=(tracking-penalty)*CONTROL_DT
        if not terms:return reward
        return reward,dict(joint_error_rad=(m.q[f]-q).square().mean(-1).sqrt(),body_error_m=epos.sqrt(),
                           **{'per_reward/tracking':tracking,'per_reward/penalties':-penalty})
    def termination_errors(self,S,f):
        p,R=S[:2]
        return (self.motion.root_p[f,2]-p[...,2]).abs(),(self.motion.root_R[f,2,2]-R[...,2,2]).abs()

class VelocityMDP(MotionTrackingMDP):
    reward_scale=.08
    fall_height=.25  # v5 embodiment: valid crouched references can be below 0.35m.
    reward_revision='paper_a6_torso_heading_v2'
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.torso_index=self.kin.tracked_names.index('waist_yaw_link')
    def reward_for_command(self,S,command,a,a_prev):
        _,R,v,w,q,_=S[:6]
        local=(yaw_mat(R).transpose(-1,-2)@v[...,None]).squeeze(-1)
        ew=(w[...,2]-command[...,2]).square()
        ev=(local[...,:2]-command[...,:2]).square().sum(-1)
        # Table A.6 uses /0.5 for squared yaw error, not /(0.5**2).
        tracking=2*torch.exp(-ev/.5**2)+2*torch.exp(-ew/.5)
        limits=(torch.relu(q-self.soft_hi)+torch.relu(self.soft_lo-q)).sum(-1)
        _,body_R,_,body_w=self.kin.forward(*S[:6])
        heading_inverse=yaw_mat(R).transpose(-1,-2)
        torso_up=(heading_inverse@body_R[...,self.torso_index,:,2,None]).squeeze(-1)
        localw=(heading_inverse@body_w[...,self.torso_index,:,None]).squeeze(-1)
        sinrollsq=torso_up[...,1].square()/(torso_up[...,1].square()+torso_up[...,2].square()+1e-8)
        penalty=.3*(a-a_prev).square().sum(-1)+2*limits+4*sinrollsq+.05*localw[...,:2].square().sum(-1)
        return (tracking-penalty)*CONTROL_DT
    def reward(self,S,f,a,a_prev,terms=False):
        command=self.active_command
        r=self.reward_for_command(S,command,a,a_prev)
        if not terms:return r
        R=S[1];local=(yaw_mat(R).transpose(-1,-2)@S[2][...,None]).squeeze(-1)
        return r,dict(joint_error_rad=(S[4]-self.motion.q[f]).square().mean(-1).sqrt(),
                      velocity_error_m_s=(local[...,:2]-command[...,:2]).norm(dim=-1),
                      yaw_error_rad_s=(S[3][...,2]-command[...,2]).abs())
    def termination_errors(self,S,f):return S[0][...,2],1-S[1][...,2,2]
    def termination(self,S,f):
        height,tilt=self.termination_errors(S,f)
        return (height<self.fall_height)|(tilt>.7)
    def soft_termination(self,S,f,tz=.02,tt=.04):
        height,tilt=self.termination_errors(S,f)
        return 1-torch.sigmoid((height-self.fall_height)/tz)*(1-torch.sigmoid((tilt-.7)/tt))
    def critic_features(self,S,f,xi_normalized):return torch.cat((invariant_features(S),xi_normalized),-1)

class SharedController(nn.Module):
    def __init__(self,goal_dim=770,hidden=(2048,1024,512,256)):
        super().__init__()
        self.motion_encoder=mlp(goal_dim,hidden,32)
        # The history feature is the complete normalized ten-frame stack.
        self.velocity_encoder=mlp(3+HISTORY*OBS_DIM,(256,256),32)
        self.decoder=mlp(32+HISTORY*OBS_DIM,hidden,31)
        self.obs_norm=RunningNorm(OBS_DIM)
        self.motion_norm=RunningNorm(goal_dim)
        self.command_norm=RunningNorm(3)
        with torch.no_grad():self.decoder[-1].weight.mul_(.01);self.decoder[-1].bias.zero_()
    def motion_latent(self,goal):return torch.tanh(run_mlp(self.motion_encoder,self.motion_norm(goal)))
    def velocity_latent(self,history,command):
        feature=self.obs_norm(history).flatten(-2)
        return torch.tanh(run_mlp(self.velocity_encoder,torch.cat((self.command_norm(command),feature),-1)))
    def decode(self,history,z):return run_mlp(self.decoder,torch.cat((z,self.obs_norm(history).flatten(-2)),-1))

class MotionActor(nn.Module):
    def __init__(self,shared):
        super().__init__();self.enc=shared.motion_encoder;self.dec=shared.decoder
        self.obs_norm=shared.obs_norm;self.goal_norm=shared.motion_norm;self.hist=HISTORY
    def forward(self,history,goal):
        z=torch.tanh(run_mlp(self.enc,self.goal_norm(goal)))
        return run_mlp(self.dec,torch.cat((z,self.obs_norm(history).flatten(-2)),-1))

class VelocityActor(nn.Module):
    def __init__(self,shared):
        super().__init__();self.enc=shared.velocity_encoder
        self.obs_norm=shared.obs_norm;self.goal_norm=shared.command_norm;self.hist=HISTORY
        object.__setattr__(self,'shared',shared)
    def forward(self,history,command):return self.shared.decode(history,self.shared.velocity_latent(history,command))
