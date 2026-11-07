"""Offline actuator candidate with explicit 0.5ms bus slots; no hardware API.

Unlike SimulationDriveBank's grouped 50Hz sends, each drive receives its own
CAN slot. Host ankle PD is computed at 500Hz and A/B sends hold that preview.
Direct drives retain their local 1kHz PD, 50Hz targets and residual damping.
"""
import numpy as np

from .simulation_drives import SimulationDriveBank


class SlotScheduledDriveBank(SimulationDriveBank):
    def __init__(self, hardware, contract, *, repeat_held_commands=False, internal_phase=0,
                 arrival_driven_preview=False, wire_quantization=False, ankle_at_send=False):
        super().__init__(hardware, contract, allow_nameplate_assumptions=True,
                         compensate_damping_cap=True)
        if internal_phase not in (0, 1): raise ValueError('1kHz phase must be 0 or 1')
        self.internal_phase = internal_phase
        self.repeat_held_commands = bool(repeat_held_commands)
        self.arrival_driven_preview=bool(arrival_driven_preview)
        self.target_generation=0;self.preview_generation=-1
        self.wire_quantization=bool(wire_quantization)
        self.ankle_at_send=bool(ankle_at_send)
        self.position_min=np.array([r['mit_ranges']['position_rad'][0] for r in self.records])
        self.position_max=np.array([r['mit_ranges']['position_rad'][1] for r in self.records])
        self.velocity_min=np.array([r['mit_ranges']['velocity_rad_s'][0] for r in self.records])
        self.velocity_max=np.array([r['mit_ranges']['velocity_rad_s'][1] for r in self.records])
        self.torque_min=np.array([r['mit_ranges']['torque_nm'][0] for r in self.records])
        self.torque_max=np.array([r['mit_ranges']['torque_nm'][1] for r in self.records])
        self.wire_kp=self.quantize(self.motor_kp,0.,500.,12) if wire_quantization else self.motor_kp.copy()
        self.wire_kd=self.quantize(self.motor_kd,0.,5.,12) if wire_quantization else self.motor_kd.copy()
        self.wire_velocity=self.quantize(np.zeros(31),self.velocity_min,self.velocity_max,12) if wire_quantization else np.zeros(31)
        self.raw_held_ff=np.zeros(31)
        self.raw_held_ankle=np.zeros(31)
        self.nominal_joint_kp=self.kp.copy();self.nominal_joint_kd=self.kd.copy()
        self.embedded_kd_max=hardware['controller']['damiao_embedded_kd_max']
        self.preview_wire_kp=self.wire_kp.copy();self.preview_wire_kd=self.wire_kd.copy()
        self.gain_scale=1.
        self.send_phase = np.zeros(31, dtype=int)
        self.send_period = np.full(31, 4 if repeat_held_commands else 40, dtype=int)
        self.send_period[self.ankle] = 4
        for bus in hardware['can_adapter']['interfaces']:
            members=sorted([i for i,r in enumerate(self.records)
                if hardware['can_adapter']['interfaces'][r['can_channel']]==bus],
                key=lambda i:self.records[i]['can_id'])
            ankle_count=other_count=0
            for i in members:
                if repeat_held_commands:
                    # Four 0.5ms slots; frames for a bus slot are serialized.
                    # Up to two frames/slot/bus, unlike the old one-frame slots.
                    self.send_phase[i] = members.index(i) % 4
                elif self.ankle[i]:
                    self.send_phase[i]=2*ankle_count;ankle_count+=1
                else:
                    self.send_phase[i]=1+2*other_count;other_count+=1
            if not repeat_held_commands and ankle_count>2:raise ValueError('too many ankle drives per bus')
        self.preview_target = self.motor_target.copy()
        self.preview_ff = np.zeros(31)
        self.preview_ankle = np.zeros(31)
        self.has_command = np.zeros(31, dtype=bool)
        self.command_sends = np.zeros(31, dtype=int)
        self.instantaneous_rejection = np.zeros(31, dtype=bool)
        self.guard_request = np.zeros(31)
        self.guard_cap = np.zeros(31)

    def due(self, tick):
        return tick % self.send_period == self.send_phase

    @staticmethod
    def quantize(value,lower,upper,bits):
        # Official SDK conversion truncates. Any unrepresentable raw field is
        # rejected separately; clipping here only permits post-trip diagnostics.
        levels=(1<<bits)-1
        code=np.floor((np.clip(value,lower,upper)-lower)/(upper-lower)*levels)
        return code*(upper-lower)/levels+lower

    def quantize_position(self,value):
        levels=65535.;span=self.position_max-self.position_min
        code=np.floor((value-self.position_min)/span*levels)
        low=np.ceil((self.soft_lower-self.position_min)/span*levels)
        high=np.floor((self.soft_upper-self.position_min)/span*levels)
        low+=low*span/levels+self.position_min<self.soft_lower
        high-=high*span/levels+self.position_min>self.soft_upper
        return np.clip(code,low,high)*span/levels+self.position_min

    def latch(self, target, q_feedback, v_feedback, *, soft_limits=True):
        super().latch(target,q_feedback,v_feedback,soft_limits=soft_limits)
        self.target_generation+=1

    def set_gain_scale(self, scale):
        """Supported offline startup only; final published gains and caps unchanged."""
        if not np.isfinite(scale) or not 0<=scale<=1:raise ValueError('startup gain scale outside 0..1')
        self.gain_scale=float(scale)
        self.kp=self.nominal_joint_kp*scale;self.kd=self.nominal_joint_kd*scale
        kp=self.inverse.T@np.diag(self.kp)@self.inverse
        kd=self.inverse.T@np.diag(self.kd)@self.inverse
        self.motor_kp=np.diag(kp).copy()
        self.motor_kd=np.minimum(np.diag(kd),self.embedded_kd_max)
        self.cross_kp=kp-np.diag(self.motor_kp)
        self.cross_kd=kd-np.diag(self.motor_kd)
        self.preview_wire_kp=self.quantize(self.motor_kp,0.,500.,12) if self.wire_quantization else self.motor_kp.copy()
        self.preview_wire_kd=self.quantize(self.motor_kd,0.,5.,12) if self.wire_quantization else self.motor_kd.copy()
        self.target_generation+=1

    def step(self, q, v, q_feedback, v_feedback, tick, *, dt=.0005,
             multirate=True, envelope=True, lag_seconds=0., rated_only=False):
        if not multirate:raise ValueError('slot schedule requires multirate mode')
        pos,speed=self.matrix@q+self.zero,self.matrix@v
        feedback_pos=self.matrix@q_feedback+self.zero
        feedback_speed=self.matrix@v_feedback
        due=self.due(tick)
        if tick%4==0 or (self.ankle_at_send and np.any(due & self.ankle)):
            joint_pd=np.clip(self.kp*(self.target-q_feedback)-self.kd*v_feedback,
                             -self.joint_limit,self.joint_limit)
            self.preview_ankle=self.map_joint_torque(joint_pd)
        preview_due=(self.target_generation!=self.preview_generation and tick%4==0) if self.arrival_driven_preview else tick%40==0
        if preview_due:
            self.preview_target=self.pending_motor_target.copy()
            self.preview_ff=self.cross_kp@(self.preview_target-feedback_pos)-self.cross_kd@feedback_speed
            self.preview_generation=self.target_generation
        direct=due & self.other
        self.wire_kp[direct]=self.preview_wire_kp[direct];self.wire_kd[direct]=self.preview_wire_kd[direct]
        target=self.quantize_position(self.preview_target) if self.wire_quantization else self.preview_target
        self.motor_target[direct]=target[direct]
        self.raw_held_ff[direct]=self.preview_ff[direct]
        ff=self.quantize(self.preview_ff,self.torque_min,self.torque_max,12) if self.wire_quantization else self.preview_ff
        self.held_ff[direct]=ff[direct]
        ankle=due & self.ankle
        self.raw_held_ankle[ankle]=self.preview_ankle[ankle]
        ankle_ff=self.quantize(self.preview_ankle,self.torque_min,self.torque_max,12) if self.wire_quantization else self.preview_ankle
        self.held_request[ankle]=ankle_ff[ankle]
        self.has_command[due]=True;self.command_sends+=due
        if tick%2==self.internal_phase:
            request=self.wire_kp*(self.motor_target-pos)+self.wire_kd*(self.wire_velocity-speed)+self.held_ff
            update=self.other & self.has_command
            self.held_request[update]=request[update]
        cap=self.limits(speed,rated_only=rated_only) if envelope else np.full(31,1e6)
        # A runtime can only estimate with the feedback it has actually received.
        estimate=self.wire_kp*(self.motor_target-feedback_pos)+self.wire_kd*(self.wire_velocity-feedback_speed)+self.held_ff
        estimate[self.ankle]=self.held_request[self.ankle]
        raw_estimate=self.motor_kp*(self.preview_target-feedback_pos)-self.motor_kd*feedback_speed+self.raw_held_ff
        raw_estimate[self.ankle]=self.raw_held_ankle[self.ankle]
        feedback_cap=self.limits(feedback_speed,rated_only=rated_only) if envelope else np.full(31,1e6)
        self.guard_request=estimate.copy();self.guard_cap=feedback_cap.copy()
        self.command_rejection=due & ((np.abs(estimate)>feedback_cap+1e-6)|(np.abs(raw_estimate)>feedback_cap+1e-6))
        transmitted=np.where(self.ankle,self.held_request,self.held_ff)
        raw_transmitted=np.where(self.ankle,self.raw_held_ankle,self.raw_held_ff)
        self.feedforward_rejection=due & ((np.abs(transmitted)>self.protocol_torque+1e-6)|(np.abs(raw_transmitted)>self.protocol_torque+1e-6))
        self.feedforward_exceedances+=self.feedforward_rejection
        self.peak_feedforward=np.maximum(self.peak_feedforward,np.abs(transmitted))
        self.instantaneous_rejection=np.abs(self.held_request)>cap+1e-6
        self.saturations+=np.abs(self.held_request)>cap+1e-9
        limited=np.clip(self.held_request,-cap,cap)
        alpha=1. if lag_seconds==0 else 1.-np.exp(-dt/lag_seconds)
        self.output+=alpha*(limited-self.output)
        self.output=np.clip(self.output,-cap,cap)
        self.speed_exceedances+=np.abs(speed)>self.maximum_speed
        self.peak_speed=np.maximum(self.peak_speed,np.abs(speed))
        self.peak_torque=np.maximum(self.peak_torque,np.abs(self.output))
        self.squared_torque+=self.output**2;self.samples+=1
        return self.map_motor_torque(self.output)
