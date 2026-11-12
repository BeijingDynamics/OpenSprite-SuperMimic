"""Simulator-only recovery curriculum; never changes the deployable actor input."""
import math
import torch
from .rotations import mat_to_quat, yaw_mat


class RecoveryImpulses:
    """Inject velocity impulses between labelled physical transitions.

    The collector marks each discontinuity for WM windows, while keeping real
    policy history, actions and episode age. A sudden unknown impulse must not
    be labelled as autonomous action-conditioned robot dynamics.
    """
    def __init__(self, count, device, config, seed):
        self.config=config
        self.device=device
        self.generator=torch.Generator(device=device).manual_seed(seed)
        self.next_step=torch.zeros(count,dtype=torch.long,device=device)
        self.nominal=torch.arange(count,device=device)%4==0
        if config.get('nominal_env_fraction',.25)!=.25:
            raise ValueError('This curriculum reserves every fourth environment as nominal')
        if config['start_iteration']<0 or config['full_iteration']<=config['start_iteration']:
            raise ValueError('Recovery curriculum needs increasing iteration bounds')
        if not 0<config['max_linear_delta_m_s']<=.75 or not 0<config['max_angular_delta_rad_s']<=1.5:
            raise ValueError('Invalid bounded recovery velocity increments')
        lo,hi=config['interval_seconds']
        if not 1<=lo<=hi:raise ValueError('Recovery intervals must allow a recovery window')

    def _intervals(self,count):
        lo,hi=self.config['interval_seconds']
        return torch.randint(round(lo/.02),round(hi/.02)+1,(count,),device=self.device,generator=self.generator)

    @torch.no_grad()
    def apply(self,env,iteration):
        reset=env.ep_step==0
        self.next_step[reset]=self._intervals(int(reset.sum()))
        magnitude=min(1.,max(0.,(iteration-self.config['start_iteration'])/
                                   (self.config['full_iteration']-self.config['start_iteration'])))
        due=(env.ep_step>=self.next_step)&~self.nominal
        ids=due.nonzero().flatten()
        delta=torch.zeros(env.N,6,device=self.device)
        if not len(ids) or magnitude==0:return delta
        self.next_step[ids]=env.ep_step[ids]+self._intervals(len(ids))
        sample=torch.rand(len(ids),4,device=self.device,generator=self.generator)
        angle=sample[:,0]*2*math.pi
        speed=(.5+.5*sample[:,1])*self.config['max_linear_delta_m_s']*magnitude
        local=torch.zeros(len(ids),3,device=self.device)
        local[:,:2]=torch.stack((angle.cos(),angle.sin()),-1)*speed[:,None]
        angular=torch.zeros_like(local)
        angular[:,:2]=(2*sample[:,2:]-1)*self.config['max_angular_delta_rad_s']*magnitude/math.sqrt(2)
        state=env.state();heading=yaw_mat(state[1][ids])
        delta[ids,:3]=(heading@local[...,None]).squeeze(-1)
        delta[ids,3:]=(heading@angular[...,None]).squeeze(-1)
        root=torch.cat((state[0][ids]+env.scene.env_origins[ids],mat_to_quat(state[1][ids]),
                        state[2][ids]+delta[ids,:3],state[3][ids]+delta[ids,3:]),-1)
        env.robot.write_root_state_to_sim(root,env_ids=ids)
        env.refresh()
        return delta


def force_schedule(cases,step,device,dt=.02):
    """World-frame torso forces, independent of policy action and observations."""
    forces=torch.zeros(len(cases),1,3,device=device)
    for index,case in enumerate(cases):
        for event in case.get('forces',[]):
            begin=round(event['at_seconds']/dt)
            end=begin+round(event['duration_seconds']/dt)
            if begin<=step<end:forces[index,0]+=torch.tensor(event['force_world_n'],device=device)
    return forces


def validate_force_cases(cases,seconds):
    for case in cases:
        for event in case.get('forces',[]):
            vector=event['force_world_n']
            values=[event['at_seconds'],event['duration_seconds'],*vector]
            if len(vector)!=3 or not all(math.isfinite(x) for x in values):
                raise ValueError('Expected finite world-frame force and timing')
            if event['at_seconds']<2 or event['duration_seconds']<.02 or sum(values[:2])>seconds:
                raise ValueError('Force must be inside evaluation after initialization')
            if not all(abs(x)<=100 for x in vector):
                raise ValueError('Evaluation force exceeds the bounded test range')
