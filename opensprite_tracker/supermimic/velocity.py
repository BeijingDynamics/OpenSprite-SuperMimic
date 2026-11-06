"""Experimental command-conditioned SuperMimic task; actor has no reference pose.

The joint-space public walk is only a reset/style prior. It is not UMR output.
All policy gradients pass through the learned model in engine.py.
"""
from dataclasses import dataclass
import torch
from .mdp import MDP, CONTROL_DT
from .rotations import yaw_mat
from .state import invariant_features
from .robot import CONTEXT_SLICES

@dataclass
class VelocityRewardCfg:
    profile: str = 'velocity'
    term_height: float = 0.28
    term_tilt: float = 0.7
    soft_limit_factor: float = 0.9
    velocity_sigma: float = 0.25
    yaw_sigma: float = 0.3
    style_weight: float = 0.5

class VelocityMDP(MDP):
    reward_scale = 0.12
    def __init__(self, kin, motion, default_q, velocity_limits, cfg=None):
        super().__init__(kin,motion,default_q,velocity_limits,cfg or VelocityRewardCfg())
        self.obs_noise=torch.tensor([.01]*31+[.5]*31+[0.]*31+[.2]*3+[.05]*3,device=default_q.device)

    def obs(self,S,a_prev,xi,f):
        _,R,_,w,q,qd=S[:6]
        gyro=(R.transpose(-1,-2)@w.unsqueeze(-1)).squeeze(-1)
        gravity=-R[...,2,:]
        return torch.cat((q-self.default_q+xi[...,CONTEXT_SLICES['encoder_bias']],qd,a_prev,gyro,gravity),-1)

    def reward(self,S,f,a,a_prev,terms=False):
        p,R,v,w,q,qd=S[:6]
        cmd=self.motion.goal(f)
        local=(yaw_mat(R).transpose(-1,-2)@v.unsqueeze(-1)).squeeze(-1)
        speed_error=(local[...,:2]-cmd[...,:2]).square().sum(-1)
        yaw_error=(w[...,2]-cmd[...,2]).square()
        tracking=2*torch.exp(-speed_error/self.cfg.velocity_sigma**2)+torch.exp(-yaw_error/self.cfg.yaw_sigma**2)
        tilt=(1-R[...,2,2]).clamp_min(0)
        upright=torch.exp(-tilt/.15)
        joint_error=(q-self.motion.q[f]).square().mean(-1)
        # Low-weight style keeps a locomotion prior without giving phase to actor.
        style=self.cfg.style_weight*torch.exp(-joint_error/.25**2)
        limits=(torch.relu(q-self.soft_hi)+torch.relu(self.soft_lo-q)).square().sum(-1)
        excess=(qd.abs()/self.velocity_limits-1).clamp_min(0).square().sum(-1)
        action_rate=(a-a_prev).square().sum(-1)
        bp,bR,bv,_=self.kin.forward(*S[:6])
        feet=bp[...,self.feet,:]
        sole=self.kin.sole_bottom_z(feet,bR[...,self.feet,:,:])
        moving=cmd[...,:2].norm(dim=-1)>.05
        # Penalize sliding near contact, but allow airborne feet to swing.
        near=torch.sigmoid((.012-sole)/.004)
        slip=(near*bv[...,self.feet,:2].square().sum(-1)).mean(-1)
        stand=(q-self.default_q).square().mean(-1)
        height=(p[...,2]-.46).square()
        penalty=2*tilt+2*v[...,2].square()+.05*w[...,:2].square().sum(-1)+.025*action_rate+2*limits+.2*excess+.25*slip+2*height
        penalty=penalty+torch.where(moving,torch.zeros_like(stand),2*stand)
        reward=(tracking+upright+style-penalty)*CONTROL_DT
        if not terms:return reward
        return reward,{'joint_error_rad':joint_error.sqrt(),
                      'velocity_error_m_s':speed_error.sqrt(),'yaw_error_rad_s':yaw_error.sqrt(),
                      'per_reward/velocity':tracking,'per_reward/upright':upright,
                      'per_reward/style':style,'per_reward/penalties':-penalty}

    def termination_errors(self,S,f):
        p,R=S[:2]
        return p[...,2],1-R[...,2,2]

    def termination(self,S,f):
        height,tilt=self.termination_errors(S,f)
        return (height<self.cfg.term_height)|(tilt>self.cfg.term_tilt)

    def soft_termination(self,S,f,tz=.02,tt=.04):
        height,tilt=self.termination_errors(S,f)
        alive=torch.sigmoid((height-self.cfg.term_height)/tz)*(1-torch.sigmoid((tilt-self.cfg.term_tilt)/tt))
        return 1-alive

    def critic_features(self,S,f,xi_normalized):
        return torch.cat((invariant_features(S),self.motion.goal(f),xi_normalized),-1)
