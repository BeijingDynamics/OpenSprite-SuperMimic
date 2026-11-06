"""Offline calibrated transmission and motor-envelope model; no transport API."""
from __future__ import annotations

import numpy as np

from .motor_mapping import motor_map_from_hardware_config


def link_frame_twist(model, data, body_id):
    """MuJoCo BODY uses principal inertia axes; the policy needs the link frame."""
    import mujoco
    velocity=np.zeros(6)
    mujoco.mj_objectVelocity(model,data,mujoco.mjtObj.mjOBJ_XBODY,body_id,velocity,1)
    return velocity


class SimulationDriveBank:
    def __init__(self, hardware, contract, *, allow_nameplate_assumptions=False,
                 compensate_damping_cap=False):
        self.mapping = motor_map_from_hardware_config(hardware, contract['joint_names'], require_armable=False)
        self.names = self.mapping.physical_motor_names
        self.records = [hardware['motor_map'][n] for n in self.names]
        self.zero = np.array(list(self.mapping.joint_to_motor_positions(np.zeros(31)).values()))
        self.matrix = np.column_stack([
            np.array(list(self.mapping.joint_to_motor_positions(np.eye(31)[i]).values())) - self.zero
            for i in range(31)])
        self.inverse = np.linalg.inv(self.matrix)
        self.ankle = np.array(['ankle_motor' in n for n in self.names])
        self.ankle_joints = np.array(['ankle_' in n for n in contract['joint_names']])
        self.other = ~self.ankle
        self.phase = np.array([0 if n.endswith('_a') else 2 for n in self.names])
        self.peak = np.array([r['peak_torque_nm'] for r in self.records], float)
        self.rated = np.array([r['rated_torque_nm'] for r in self.records], float)
        self.assumptions = []
        rated_speed=[];no_load=[];maximum_speed=[]
        design_speed=np.abs(self.matrix)@np.asarray(contract['velocity_limit'])
        for i,r in enumerate(self.records):
            rated=r['rated_speed_rad_s'];maximum=r['max_speed_rad_s']
            if rated is None or maximum is None:
                if not allow_nameplate_assumptions:
                    raise ValueError(f"unidentified motor speed fields: {self.names[i]}")
                spec=hardware['motor_model_specs'][r['model']]
                rated=spec['rated_speed_rad_s']
                free=spec['maximum_no_load_speed_rad_s_at_48v']*hardware['controller']['nominal_bus_voltage_v']/48.
                project=min(free,design_speed[i],min(abs(v) for v in r['mit_ranges']['velocity_rad_s']))
                self.assumptions.append({'motor':self.names[i],'rated_speed_rad_s':rated,
                    'envelope_no_load_rad_s':free,'project_max_speed_rad_s':project,
                    'source':'nameplate rated speed; voltage-scaled free speed proxy; project speed from transformed URDF limits'})
            else:free=maximum;project=maximum
            rated_speed.append(rated);no_load.append(free);maximum_speed.append(project)
        self.rated_speed=np.array(rated_speed);self.no_load_speed=np.array(no_load)
        self.maximum_speed=np.array(maximum_speed)
        if not np.isfinite(np.r_[self.peak,self.rated,self.rated_speed,self.no_load_speed,self.maximum_speed]).all():
            raise ValueError('motor envelope contains unidentified/nonfinite values')
        if np.any(self.no_load_speed<=self.rated_speed) or np.any(self.rated_speed<=0):
            raise ValueError('motor envelope speed breakpoints must increase')
        self.protocol_torque = np.array([min(abs(x) for x in r['mit_ranges']['torque_nm']) for r in self.records])
        self.soft_lower = np.array([r['soft_limit_rad'][0] for r in self.records])
        self.soft_upper = np.array([r['soft_limit_rad'][1] for r in self.records])
        self.hard_lower = np.array([r['hard_limit_rad'][0] for r in self.records])
        self.hard_upper = np.array([r['hard_limit_rad'][1] for r in self.records])
        self.kp = np.array(contract['stiffness'], float)
        self.kd = np.array(contract['damping'], float)
        self.joint_limit = np.array(contract['effort_limit'], float)
        kp_m = self.inverse.T @ np.diag(self.kp) @ self.inverse
        kd_m = self.inverse.T @ np.diag(self.kd) @ self.inverse
        self.motor_kp = np.diag(kp_m).copy()
        self.motor_kd = np.minimum(np.diag(kd_m), hardware['controller']['damiao_embedded_kd_max'])
        self.cross_kp = kp_m - np.diag(np.diag(kp_m))
        # Optional controller candidate: keep the qualified embedded Kd ceiling
        # and realize its missing diagonal damping in sampled host feedforward.
        # Default retains the previously evaluated deployment realization.
        self.compensate_damping_cap = bool(compensate_damping_cap)
        embedded = self.motor_kd if self.compensate_damping_cap else np.diag(kd_m)
        self.cross_kd = kd_m - np.diag(embedded)
        self.target = np.zeros(31)
        self.motor_target = self.zero.copy()
        self.pending_motor_target = self.motor_target.copy()
        self.command_rejection = np.zeros(31, bool)
        self.feedforward_rejection = np.zeros(31, bool)
        self.held_ff = np.zeros(31)
        self.held_request = np.zeros(31)
        self.feedback_q = np.zeros(31)
        self.feedback_v = np.zeros(31)
        self.output = np.zeros(31)
        self.saturations = np.zeros(31, int)
        self.soft_clips = np.zeros(31, int)
        self.speed_exceedances = np.zeros(31, int)
        self.feedforward_exceedances = np.zeros(31, int)
        self.peak_feedforward = np.zeros(31)
        self.peak_speed = np.zeros(31)
        self.peak_torque = np.zeros(31)
        self.squared_torque = np.zeros(31)
        self.samples = 0

    def limits(self, speed, *, rated_only=False):
        speed = np.abs(speed)
        low = self.peak + (self.rated - self.peak) * speed / self.rated_speed
        high = self.rated * (self.no_load_speed - speed) / (self.no_load_speed - self.rated_speed)
        limit = np.maximum(0, np.where(speed <= self.rated_speed, low, high))
        limit = np.minimum(limit, self.protocol_torque)
        return np.minimum(limit, self.rated) if rated_only else limit

    def map_joint_torque(self, torque):
        return self.inverse.T @ torque

    def map_motor_torque(self, torque):
        return self.matrix.T @ torque

    def latch(self, target, q_feedback, v_feedback, *, soft_limits=True):
        self.target = np.asarray(target).copy()
        target_m = self.matrix @ self.target + self.zero
        if soft_limits:
            self.soft_clips += (target_m < self.soft_lower) | (target_m > self.soft_upper)
            target_m = np.clip(target_m, self.soft_lower, self.soft_upper)
        self.pending_motor_target = target_m

    def step(self, q, v, q_feedback, v_feedback, tick, *, dt=.0005,
             multirate=True, envelope=True, lag_seconds=0., rated_only=False):
        pos, speed = self.matrix @ q + self.zero, self.matrix @ v
        if tick % (40 if multirate else 4) == 0:
            self.motor_target = self.pending_motor_target.copy()
            error = self.motor_target - (self.matrix @ q_feedback + self.zero)
            self.held_ff = self.cross_kp @ error - self.cross_kd @ (self.matrix @ v_feedback)
        if tick % (2 if multirate else 4) == 0:  # 1kHz deployment /500Hz ablation.
            request = self.motor_kp * (self.motor_target - pos) - self.motor_kd * speed + self.held_ff
            self.held_request[self.other] = request[self.other]
        # Stagger A/B ankle commands by1ms, each at500Hz; host PD uses delayed feedback.
        due = self.ankle & ((tick % 4 == self.phase) if multirate else tick % 4 == 0)
        if np.any(due):
            joint_tau = np.clip(self.kp * (self.target - q_feedback) - self.kd * v_feedback,
                                -self.joint_limit, self.joint_limit)
            self.held_request[due] = self.map_joint_torque(joint_tau)[due]
        cap = self.limits(speed, rated_only=rated_only) if envelope else np.full(31, 1e6)
        command_due = due | (self.other if tick % (40 if multirate else 4) == 0 else False)
        self.command_rejection = command_due & (np.abs(self.held_request)>cap+1e-6)
        # Feedforward field representability is independent of total PD torque.
        transmitted_ff = np.where(self.ankle,self.held_request,self.held_ff)
        self.feedforward_rejection = command_due & (np.abs(transmitted_ff)>self.protocol_torque+1e-6)
        self.feedforward_exceedances += self.feedforward_rejection
        self.peak_feedforward = np.maximum(self.peak_feedforward,np.abs(transmitted_ff))
        limited = np.clip(self.held_request, -cap, cap)
        self.saturations += np.abs(self.held_request) > cap + 1e-9
        alpha = 1. if lag_seconds == 0 else 1. - np.exp(-dt / lag_seconds)
        self.output += alpha * (limited - self.output)
        # Enforce the instantaneous output envelope even when response lag is enabled.
        self.output = np.clip(self.output, -cap, cap)
        self.speed_exceedances += np.abs(speed) > self.maximum_speed
        self.peak_speed = np.maximum(self.peak_speed, np.abs(speed))
        self.peak_torque = np.maximum(self.peak_torque, np.abs(self.output))
        self.squared_torque += self.output**2
        self.samples += 1
        return self.map_motor_torque(self.output)

    def report(self):
        return {name: {'peak_speed_rad_s': float(self.peak_speed[i]),
                       'peak_torque_nm': float(self.peak_torque[i]),
                       'rms_torque_nm': float(np.sqrt(self.squared_torque[i]/max(1,self.samples))),
                       'saturated_samples': int(self.saturations[i]),
                       'speed_exceedances': int(self.speed_exceedances[i]),
                       'feedforward_field_exceedances': int(self.feedforward_exceedances[i]),
                       'peak_feedforward_nm': float(self.peak_feedforward[i]),
                       'target_soft_clips': int(self.soft_clips[i])}
                for i,name in enumerate(self.names)}
