"""Opt-in velocity reward experiment; preserve the published motion interface."""
import math
import torch
from .joint_interfaces import VelocityMDP
from .mdp import CONTROL_DT
from .rotations import yaw_mat

REVISION = 'sprite_velocity_posture_width_v1'
DEFAULTS = dict(revision=REVISION, pitch_weight=8.0, pitch_deadband_deg=2.0,
                minimum_center_m=.12, minimum_edge_gap_m=.04, maximum_center_m=.18,
                width_scale_m=.04, width_weight=.4, excessive_width_weight=.1)
SWING_REVISION='sprite_velocity_posture_width_swing_v2'
SWING_DEFAULTS=dict(DEFAULTS,revision=SWING_REVISION,pitch_weight=32.0,pitch_deadband_deg=1.0,
                    swing_height_m=.020,swing_speed_m_s=.08,swing_weight=.15,command_gate_m_s=.05)
STAND_REVISION='sprite_velocity_standing_knee_v3'
STAND_DEFAULTS=dict(SWING_DEFAULTS,revision=STAND_REVISION,standing_knee_weight=.2,
                   standing_knee_deadband_deg=20.,standing_knee_scale_deg=20.,
                   standing_velocity_scale_m_s=.03,standing_yaw_scale_rad_s=.05)

def standing_knee_cost(knee_angle,command,config):
    """Soft excess-flexion cost near zero command; never generates joint targets."""
    scaled=command[...,:2].square().sum(-1)/config['standing_velocity_scale_m_s']**2
    scaled=scaled+command[...,2].square()/config['standing_yaw_scale_rad_s']**2
    excess=torch.relu(knee_angle.abs()-math.radians(config['standing_knee_deadband_deg']))/math.radians(config['standing_knee_scale_deg'])
    return config['standing_knee_weight']*torch.exp(-scaled)*excess.square().mean(-1)

def moving_sole_cost(height,velocity_xy,command,config):
    """Symmetric clearance cost, without a phase clock or prescribed left/right step."""
    speed2=velocity_xy.square().sum(-1)
    moving=speed2/(speed2+config['swing_speed_m_s']**2)
    command2=command[...,:2].square().sum(-1)+(.2*command[...,2]).square()
    commanded=command2/(command2+config['command_gate_m_s']**2)
    short=(torch.relu(config['swing_height_m']-height)/config['swing_height_m']).square()
    return config['swing_weight']*commanded*(moving*short).mean(-1)

def posture_width_cost(torso_up, separation, edge_gap, config):
    """Yaw-invariant soft costs; no gait phase, heading target or action edits."""
    sin_pitch=torso_up[...,0]/torch.sqrt(torso_up[...,0].square()+torso_up[...,2].square()+1e-8)
    pitch=torch.relu(sin_pitch.abs()-math.sin(math.radians(config['pitch_deadband_deg']))).square()
    center_short=torch.relu(config['minimum_center_m']-separation)/config['width_scale_m']
    edge_short=torch.relu(config['minimum_edge_gap_m']-edge_gap)/config['width_scale_m']
    narrow=torch.maximum(center_short,edge_short).square()
    wide=(torch.relu(separation-config['maximum_center_m'])/config['width_scale_m']).square()
    return config['pitch_weight']*pitch+config['width_weight']*narrow+config['excessive_width_weight']*wide

class PostureWidthMDP(VelocityMDP):
    def __init__(self,*args,extension,**kwargs):
        super().__init__(*args,**kwargs)
        self.extension=extension

    def posture_geometry(self,S):
        p,R,_,_=self.kin.forward(*S[:6])
        heading=yaw_mat(S[1]);lateral=heading[..., :,1]
        fp=p[...,self.feet,:];fr=R[...,self.feet,:,:]
        center=fp+(fr@self.kin.sole_centers[...,None]).squeeze(-1)
        sole_R=fr@self.kin.sole_R
        separation=((center[...,0,:]-center[...,1,:])*lateral).sum(-1)
        extent=((lateral[...,None,None,:]@sole_R).squeeze(-2).abs()*self.kin.sole_half).sum((-1,-2))
        up=(heading.transpose(-1,-2)@R[...,self.torso_index,:,2,None]).squeeze(-1)
        return up,separation,separation-extent

    def reward_for_command(self,S,command,a,a_prev):
        base=super().reward_for_command(S,command,a,a_prev)
        return base-CONTROL_DT*posture_width_cost(*self.posture_geometry(S),self.extension)

class SwingPostureWidthMDP(PostureWidthMDP):
    def reward_for_command(self,S,command,a,a_prev):
        base=super().reward_for_command(S,command,a,a_prev)
        p,R,v,w=self.kin.forward(*S[:6])
        fp,fr=p[...,self.feet,:],R[...,self.feet,:,:]
        offset=(fr@self.kin.sole_centers[...,None]).squeeze(-1)
        center_v=v[...,self.feet,:]+torch.linalg.cross(w[...,self.feet,:],offset,dim=-1)
        height=self.kin.sole_bottom_z(fp,fr)
        return base-CONTROL_DT*moving_sole_cost(height,center_v[...,:2],command,self.extension)

class StandingKneeMDP(SwingPostureWidthMDP):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.knee_ids=[self.kin.joint_names.index(s+'_knee_joint') for s in ['left','right']]
        self.knee_refs=torch.stack([self.kin.qref[self.kin.joint_idx.index(j)] for j in self.knee_ids])
    def reward_for_command(self,S,command,a,a_prev):
        base=super().reward_for_command(S,command,a,a_prev)
        return base-CONTROL_DT*standing_knee_cost(S[4][...,self.knee_ids]-self.knee_refs,command,self.extension)

def make_velocity_mdp(config,*args,**kwargs):
    extension=config.get('velocity_reward_extension')
    if extension is None:return VelocityMDP(*args,**kwargs)
    standing=extension.get('revision')==STAND_REVISION
    swing=standing or extension.get('revision')==SWING_REVISION
    expected=STAND_DEFAULTS if standing else SWING_DEFAULTS if swing else DEFAULTS
    if set(extension)!=set(expected) or extension['revision'] not in (REVISION,SWING_REVISION,STAND_REVISION):
        raise ValueError('Unknown or incomplete posture/width reward extension')
    for k,v in extension.items():
        if k!='revision' and (not math.isfinite(v) or v<0):raise ValueError('Invalid posture/width coefficient')
    if not 0<extension['width_scale_m'] or not .08<=extension['minimum_center_m']<extension['maximum_center_m']<=.25:
        raise ValueError('Invalid width envelope')
    if swing and not (.01<=extension['swing_height_m']<=.04 and .01<=extension['swing_speed_m_s']<=.3 and
                      0<extension['command_gate_m_s']<=.2 and extension['swing_weight']<=.5):
        raise ValueError('Invalid bounded moving-sole reward')
    if standing and not (0<=extension['standing_knee_weight']<=.5 and 10<=extension['standing_knee_deadband_deg']<=30 and
                         10<=extension['standing_knee_scale_deg']<=40 and .01<=extension['standing_velocity_scale_m_s']<=.05 and
                         .01<=extension['standing_yaw_scale_rad_s']<=.1):
        raise ValueError('Invalid bounded standing-knee reward')
    cls=StandingKneeMDP if standing else SwingPostureWidthMDP if swing else PostureWidthMDP
    return cls(*args,extension=dict(extension),**kwargs)
