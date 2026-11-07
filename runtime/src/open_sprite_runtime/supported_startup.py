"""Virtual fixture and gain-ramp experiment only; no device/actuation backend.

The six-axis fixture is stronger than a hanging rope. Its assistance is recorded
explicitly and reaches exactly zero before unassisted evaluation can begin.
"""
import numpy as np


def smooth(value):
    x=np.clip(value,0.,1.)
    return x*x*(3-2*x)


class SupportedStartup:
    def __init__(self, initial_q, initial_quat, root_xy, standing_z, *,
                 seconds=5.,lowering_seconds=2.,unloading_seconds=1.,initial_gain=.1,lift_height=.05):
        if not all(np.isfinite(v) and v>0 for v in (seconds,lowering_seconds,unloading_seconds)):
            raise ValueError('invalid virtual support phases')
        if not np.isfinite(initial_gain) or not 0<initial_gain<=1:raise ValueError('invalid startup gain')
        if not np.isfinite(lift_height) or not 0<lift_height<=.1:raise ValueError('invalid virtual lift height')
        self.initial_q=np.asarray(initial_q,float).copy();self.initial_quat=np.asarray(initial_quat,float).copy()
        if self.initial_q.shape!=(31,) or self.initial_quat.shape!=(4,) or not np.isfinite(np.r_[self.initial_q,self.initial_quat]).all():
            raise ValueError('invalid fixture initial state')
        if abs(np.linalg.norm(self.initial_quat)-1)>1e-6:raise ValueError('fixture quaternion not unit')
        if self.initial_quat[0]<0:self.initial_quat*=-1
        self.root_xy=np.asarray(root_xy,float);self.standing_z=float(standing_z)
        if self.root_xy.shape!=(2,) or not np.isfinite(self.root_xy).all() or not np.isfinite(self.standing_z) or not 0<self.standing_z<2:
            raise ValueError('invalid fixture root reference')
        self.seconds=seconds;self.lowering=lowering_seconds;self.unloading=unloading_seconds;self.initial_gain=initial_gain
        self.lift_height=lift_height
        self.actor_start=seconds+lowering_seconds
        self.release=self.actor_start+unloading_seconds

    def targets(self,time):
        a=float(smooth(time/self.seconds))
        return (1-a)*self.initial_q,self.initial_gain+(1-self.initial_gain)*a

    def fraction(self,time):
        return float(1-smooth((time-self.actor_start)/self.unloading)) if time<self.release else 0.

    def wrench(self,time,position,rotation,velocity,omega,com,mass,gravity):
        fraction=self.fraction(time)
        if fraction==0:return np.zeros(3),np.zeros(3)
        import mujoco
        z=self.standing_z+self.lift_height*(1-float(smooth((time-self.seconds)/self.lowering)))
        desired=np.r_[self.root_xy,z]
        a=float(smooth(time/min(self.seconds,2.)))
        quat=(1-a)*self.initial_quat+a*np.array([1.,0,0,0]);quat/=np.linalg.norm(quat)
        target_rotation=np.empty(9);mujoco.mju_quat2Mat(target_rotation,quat)
        delta=target_rotation.reshape(3,3)@rotation.T
        error_quat=np.empty(4);mujoco.mju_mat2Quat(error_quat,delta.reshape(-1))
        if error_quat[0]<0:error_quat*=-1
        sine=np.linalg.norm(error_quat[1:]);angle=2*np.arctan2(sine,error_quat[0])
        error=error_quat[1:]*(angle/sine) if sine>1e-12 else 2*error_quat[1:]
        weight=np.asarray(gravity)*mass
        force=-weight+2000*(desired-position)-100*velocity
        torque=-np.cross(com-position,weight)+50*error-5*omega
        force*=min(1.,2*np.linalg.norm(weight)/max(np.linalg.norm(force),1e-12))
        torque*=min(1.,20./max(np.linalg.norm(torque),1e-12))
        return fraction*force,fraction*torque
