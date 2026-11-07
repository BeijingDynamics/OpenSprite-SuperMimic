"""Frozen-policy closed-loop MuJoCo evaluation with calibrated motor coordinates."""
import argparse
from concurrent.futures import ProcessPoolExecutor
from collections import deque
import json
import os
from pathlib import Path
import time

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.handoff import MeasuredPoseActionHandoff
from open_sprite_runtime.policy_shadow import PolicyObservationHistory
from open_sprite_runtime.simulation_drives import SimulationDriveBank,link_frame_twist
from open_sprite_runtime.supermimic_replay import checked_model, file_sha256
from open_sprite_runtime.command_slew import CommandSlewLimiter
from open_sprite_runtime.scheduled_drives import SlotScheduledDriveBank
from open_sprite_runtime.supported_startup import SupportedStartup

MODES = {
    'joint_pd': dict(mapped=False, envelope=False, multirate=False, delay_ms=0, feedback_ms=0, lag_ms=0),
    'mapped': dict(mapped=True, envelope=False, multirate=False, delay_ms=0, feedback_ms=0, lag_ms=0),
    'envelope': dict(mapped=True, envelope=True, multirate=False, delay_ms=0, feedback_ms=0, lag_ms=0),
    'multirate': dict(mapped=True, envelope=True, multirate=True, delay_ms=0, feedback_ms=0, lag_ms=0),
    'measured_delay': dict(mapped=True, envelope=True, multirate=True, delay_ms=10, feedback_ms=0, lag_ms=0),
    'runtime': dict(mapped=True, envelope=True, multirate=True, delay_ms=10, feedback_ms=2, lag_ms=5),
    'runtime_strict': dict(mapped=True, envelope=True, multirate=True, delay_ms=10, feedback_ms=0, lag_ms=0),
    'stress': dict(mapped=True, envelope=True, multirate=True, delay_ms=15, feedback_ms=5, lag_ms=10),
    'slot_sparse_strict': dict(mapped=True, envelope=True, multirate=True, delay_ms=12, feedback_ms=5, lag_ms=5, slotted=True, repeat=False),
    'slot_repeat_strict': dict(mapped=True, envelope=True, multirate=True, delay_ms=12, feedback_ms=5, lag_ms=5, slotted=True, repeat=True),
    'slot_arrival_strict': dict(mapped=True, envelope=True, multirate=True, delay_ms=12, feedback_ms=5, lag_ms=5, slotted=True, repeat=True, arrival=True),
    'slot_fresh_ankle_strict': dict(mapped=True, envelope=True, multirate=True, delay_ms=12, feedback_ms=5, lag_ms=5, slotted=True, repeat=True, arrival=True, ankle_at_send=True),
}


def run_case(job):
    import mujoco
    import onnxruntime as ort
    contract_path,hardware_path,scene_path,out_path,case,mode,seed,seconds,handoff,profile_path = job[:10]
    experiment=job[10] if len(job)>10 else {}
    c=PolicyContract.load(contract_path);model_path=checked_model(c)
    cfg=dict(MODES[mode]);hardware=json.loads(Path(hardware_path).read_text())
    for key in ('delay_ms','feedback_ms','lag_ms'):
        if experiment.get(key) is not None:
            if mode not in ('runtime_strict','slot_sparse_strict','slot_repeat_strict','slot_arrival_strict','slot_fresh_ankle_strict'):raise ValueError('timing overrides are isolated strict-mode experiments')
            cfg[key]=experiment[key]
    controls=dict(c.data)
    if profile_path:
        profile=json.loads(Path(profile_path).read_text())
        if set(profile['joints'])!=set(c.data['joint_names']):raise ValueError('actuator profile joint names differ')
        controls['stiffness']=[profile['joints'][n]['kp'] for n in c.data['joint_names']]
        controls['damping']=[profile['joints'][n]['kd'] for n in c.data['joint_names']]
        if not np.isfinite(np.r_[controls['stiffness'],controls['damping']]).all():raise ValueError('nonfinite gains')
    slotted=cfg.get('slotted',False)
    strict=mode in ('runtime_strict','slot_sparse_strict','slot_repeat_strict','slot_arrival_strict','slot_fresh_ankle_strict')
    if slotted:
        if not experiment.get('damping_compensation',False):raise ValueError('slot candidate requires explicit residual damping acknowledgement')
        bank=SlotScheduledDriveBank(hardware,controls,repeat_held_commands=cfg['repeat'],
            internal_phase=experiment.get('internal_phase',0),arrival_driven_preview=cfg.get('arrival',False),
            wire_quantization=experiment.get('wire_quantization',False),ankle_at_send=cfg.get('ankle_at_send',False))
    else:
        bank=SimulationDriveBank(hardware,controls,allow_nameplate_assumptions=True,
            compensate_damping_cap=experiment.get('damping_compensation',False))
    model=mujoco.MjModel.from_binary_path(str(scene_path));model.opt.timestep=.0005
    # qfrc_applied is the sole actuation source; train-scene motors remain at ctrl=0.
    data=mujoco.MjData(model)
    ids=[next(j for j in range(model.njnt) if model.joint(j).name.rsplit('/',1)[-1]==n) for n in c.data['joint_names']]
    qi=model.jnt_qposadr[ids];vi=model.jnt_dofadr[ids]
    base=next(i for i in range(model.nbody) if model.body(i).name.rsplit('/',1)[-1]=='pelvis_link')
    feet=[i for i in range(model.ngeom) if model.geom(i).name.rsplit('/',1)[-1] in ('left_foot_collision','right_foot_collision')]
    feet.sort(key=lambda i:model.geom(i).name)
    floors=set(i for i in range(model.ngeom) if model.geom_type[i]==mujoco.mjtGeom.mjGEOM_PLANE)
    data.qpos[:3]=[0,0,.46324583888053894];data.qpos[3:7]=[1,0,0,0];data.qpos[qi]=0
    rng=np.random.default_rng(seed)
    if seed!=101:
        data.qpos[0:2]=rng.uniform(-.001,.001,2)
        data.qvel[vi]=rng.normal(0,.005,31)
    joint_jitter=float(experiment.get('initial_joint_jitter',0.))
    base_tilt=float(experiment.get('initial_base_tilt',0.))
    if joint_jitter:
        data.qpos[qi]=np.clip(rng.uniform(-joint_jitter,joint_jitter,31),
                              model.jnt_range[ids,0],model.jnt_range[ids,1])
    if base_tilt:
        roll,pitch=rng.uniform(-base_tilt,base_tilt,2)
        cr,sr=np.cos(roll/2),np.sin(roll/2);cp,sp=np.cos(pitch/2),np.sin(pitch/2)
        data.qpos[3:7]=[cr*cp,sr*cp,cr*sp,-sr*sp]
    initial_path=experiment.get('initial_state')
    if initial_path:
        initial=json.loads(Path(initial_path).read_text())
        init_q=np.asarray(initial['joint_position'],float);init_v=np.asarray(initial['joint_velocity'],float)
        quat=np.asarray(initial['root_quaternion_wxyz'],float)
        if init_q.shape!=(31,) or init_v.shape!=(31,) or quat.shape!=(4,) or not np.isfinite(np.r_[init_q,init_v,quat]).all():
            raise ValueError('invalid initial-state dimensions or values')
        if abs(np.linalg.norm(quat)-1)>1e-6:raise ValueError('initial quaternion not unit length')
        motor_init=bank.matrix@init_q+bank.zero
        if np.any((motor_init<bank.hard_lower)|(motor_init>bank.hard_upper)):
            raise ValueError('recorded/counterfactual initial pose rejected by unchanged hard position guards')
        if np.any((init_q<model.jnt_range[ids,0])|(init_q>model.jnt_range[ids,1])):
            raise ValueError('initial pose rejected by unchanged URDF joint limits')
        data.qpos[qi]=init_q;data.qvel[vi]=init_v;data.qpos[3:7]=quat
    mujoco.mj_forward(model,data)
    if joint_jitter or base_tilt or initial_path:
        if any(model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_BOX for g in feet):
            raise ValueError('initial ground placement requires the validated box-foot geometry')
        bottom=min(data.geom_xpos[g,2]-abs(data.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g] for g in feet)
        data.qpos[2]+=.001-bottom
        mujoco.mj_forward(model,data)
    initial_state={'root_pose':data.qpos[:7].tolist(),'joint_position':data.qpos[qi].tolist(),
                   'joint_velocity':data.qvel[vi].tolist()}
    support=None;peak_support_force=0.;peak_support_torque=0.;post_release_support=0.
    if experiment.get('supported_preposition',False):
        if not slotted or not initial_path:raise ValueError('supported preposition needs explicit initial state and slot mode')
        probe=mujoco.MjData(model);probe.qpos[:3]=[0,0,1];probe.qpos[3:7]=[1,0,0,0];probe.qpos[qi]=0
        mujoco.mj_forward(model,probe)
        bottom=min(probe.geom_xpos[g,2]-abs(probe.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g] for g in feet)
        standing_z=1-bottom+.001
        support=SupportedStartup(data.qpos[qi],data.qpos[3:7],data.qpos[:2],standing_z,
            seconds=experiment.get('preposition_seconds',5.),initial_gain=experiment.get('preposition_initial_gain',.1),
            lowering_seconds=experiment.get('lowering_seconds',2.),unloading_seconds=experiment.get('unloading_seconds',1.),
            lift_height=experiment.get('lift_height',.05))
        bank.set_gain_scale(support.initial_gain)
    opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
    opts.execution_mode=ort.ExecutionMode.ORT_SEQUENTIAL
    session=ort.InferenceSession(str(model_path),sess_options=opts,providers=['CPUExecutionProvider'])
    history=PolicyObservationHistory(c);prev=np.zeros(31,np.float32)
    cc=dict(c.data)
    if not handoff:cc['deployment_handoff_seconds']=0.
    if experiment.get('handoff_seconds') is not None:
        if not handoff:raise ValueError('handoff duration override requires --handoff')
        cc['deployment_handoff_seconds']=experiment['handoff_seconds']
    blend=MeasuredPoseActionHandoff(PolicyContract(c.path,cc))
    offset=np.array(c.data['action_offset']);scale=np.array(c.data['action_scale'])
    kp=np.array(controls['stiffness']);kd=np.array(controls['damping']);limits=np.array(controls['effort_limit'])
    q=data.qpos[qi].copy();v=data.qvel[vi].copy();target=q.copy();bank.latch(target,q,v)
    # Feedback timestamps differ by motor: ankles500Hz, other drives50Hz.
    measured_qm=bank.matrix@q+bank.zero;measured_vm=bank.matrix@v
    measured_stamp=np.zeros(31)
    feedback_trace=experiment.get('feedback_timing_trace')
    recorded_age=None
    if feedback_trace:
        if mode not in ('slot_repeat_strict','slot_arrival_strict','slot_fresh_ankle_strict') or cfg['feedback_ms']!=0:
            raise ValueError('recorded 500Hz timing requires repeat mode and no double-counted fixed delay')
        with np.load(feedback_trace,allow_pickle=False) as timing:
            names=timing['motor_names'].tolist();ages=timing['age_ticks']
            if len(names)!=31 or set(names)!=set(bank.names) or float(timing['dt'])!=.0005:
                raise ValueError('feedback trace motor identity/timestep mismatch')
            if ages.ndim!=2 or ages.shape[1]!=31 or not np.issubdtype(ages.dtype,np.integer) or ages.min()<0 or ages.max()>60:
                raise ValueError('invalid feedback age trace')
            if len(ages)<round(seconds/.0005):raise ValueError('feedback trace too short; never looped or padded')
            recorded_age=ages[:,[names.index(n) for n in bank.names]].copy()
        ring=int(recorded_age.max())+1
        qm_ring=np.tile(measured_qm,(ring,1));vm_ring=np.tile(measured_vm,(ring,1))
    peak_state_age_ms=0.;peak_post_inference_age_ms=0.
    imu_ticks=round(float(experiment.get('imu_delay_ms',0.))/1000/.0005)
    imu_history=deque(maxlen=imu_ticks+1)
    lag_ticks=round(cfg['feedback_ms']/1000/.0005)
    state_history=deque([(q.copy(),v.copy()) for _ in range(lag_ticks+1)],maxlen=lag_ticks+1)
    pending=deque();first_failure=None;first_reason=None;first_motor=None;trip_motor_diagnostics=None;protection_event=None
    total_ticks=round(seconds/.0005);policy_ticks=40
    saved={key:[] for key in ('qpos','commands','velocity','yaw_rate','failed','proprioception','policy_action','raw_commands')}
    slew=None
    if experiment.get('command_linear_accel') is not None:
        slew=CommandSlewLimiter(experiment['command_linear_accel'],experiment['command_yaw_accel'])
    trace={key:[] for key in ('time','q','qd','q_feedback','qd_feedback','target',
        'motor_target','motor_position','motor_speed','motor_request','motor_cap','motor_output')}
    trace_seconds=float(experiment.get('trace_seconds',0.))
    command_start=float(experiment.get('command_start_seconds',2.))
    if support:command_start=max(command_start,support.release-support.actor_start+2.)
    started=time.monotonic();joint_target=target.copy();last_action=np.zeros(31)
    contact_samples=0;force_sums=np.zeros(2);unloaded=np.zeros(2);slip_samples=np.zeros(2);loaded_samples=np.zeros(2)
    force_weighted_speed=np.zeros(2);normal_total=np.zeros(2);max_contact_speed=np.zeros(2)
    jacp=np.zeros((3,model.nv));jacr=np.zeros_like(jacp)
    fvec=np.zeros(6);peak_joint_torque=np.zeros(31);held_joint_torque=np.zeros(31)
    for tick in range(total_ticks):
        now=tick*.0005
        policy_time=now-(support.actor_start if support else 0.)
        q=data.qpos[qi].copy();v=data.qvel[vi].copy()
        state_history.append((q.copy(),v.copy()));q_old,v_old=state_history[0]
        qm_old=bank.matrix@q_old+bank.zero;vm_old=bank.matrix@v_old
        if recorded_age is not None:
            qm_ring[tick%ring]=bank.matrix@q+bank.zero;vm_ring[tick%ring]=bank.matrix@v
            sample_tick=tick-recorded_age[tick]
            indices=np.where(sample_tick<0,0,sample_tick)%ring
            measured_qm=qm_ring[indices,np.arange(31)];measured_vm=vm_ring[indices,np.arange(31)]
            if experiment.get('feedback_quantization',False):
                measured_qm=bank.quantize(measured_qm,bank.position_min,bank.position_max,16)
                measured_vm=bank.quantize(measured_vm,bank.velocity_min,bank.velocity_max,12)
            measured_stamp=np.maximum(0,sample_tick)*.0005
        else:
            if slotted:due=bank.due(tick)
            else:
                due=bank.ankle & (tick%4==bank.phase)
                if tick%40==0:due=due | bank.other
            if experiment.get('feedback_quantization',False):
                qm_old=bank.quantize(qm_old,bank.position_min,bank.position_max,16)
                vm_old=bank.quantize(vm_old,bank.velocity_min,bank.velocity_max,12)
            measured_qm[due]=qm_old[due];measured_vm[due]=vm_old[due];measured_stamp[due]=max(0,now-lag_ticks*.0005)
        q_obs=bank.inverse@(measured_qm-bank.zero);v_obs=bank.inverse@measured_vm
        if not cfg['multirate']:q_obs,v_obs=q_old,v_old
        if imu_ticks:
            mujoco.mj_forward(model,data)
            imu_history.append((link_frame_twist(model,data,base)[:3].copy(),-data.xmat[base].reshape(3,3)[2].copy()))
        if tick%policy_ticks==0:
            mujoco.mj_forward(model,data)
            rotation=data.xmat[base].reshape(3,3)
            body_vel=link_frame_twist(model,data,base)
            gyro=body_vel[:3];gravity=-rotation[2]
            if imu_ticks:gyro,gravity=imu_history[0]
            state_age_ms=(now-measured_stamp.min())*1000
            post_age=state_age_ms+float(experiment.get('inference_budget_ms',0.))
            peak_state_age_ms=max(peak_state_age_ms,state_age_ms)
            peak_post_inference_age_ms=max(peak_post_inference_age_ms,post_age)
            if slotted and post_age>30 and first_failure is None:
                first_failure=now;first_reason='feedback_post_inference_age';first_motor=bank.names[int(np.argmin(measured_stamp))]
            command=np.zeros(3) if policy_time<command_start else np.array(case['command'],float)
            for stage in case.get('stages',[]):
                if policy_time>=stage['at_seconds']:command=np.array(stage['command'],float)
            raw_command=command.copy()
            if slew is not None:command=slew.step(command,.02)
            q_input,v_input=(q_obs,v_obs) if cfg['mapped'] else (q,v)
            history.append(q_input,v_input,prev,gyro,gravity)
            obs=history.observation(command)
            if support and policy_time<0:
                joint_target,gain=support.targets(now)
                prev=((joint_target-offset)/scale).astype(np.float32);action=prev.copy()
            else:
                action=session.run(None,{session.get_inputs()[0].name:obs[None]})[0][0]
                prev=blend.blend(action,q_input);joint_target=offset+scale*prev;gain=1.
            pending.append((tick+round(cfg['delay_ms']/1000/.0005),joint_target.copy(),gain))
            yaw=np.arctan2(rotation[1,0],rotation[0,0]);co,si=np.cos(yaw),np.sin(yaw)
            world_vel=rotation@body_vel[3:];velocity=np.array([co*world_vel[0]+si*world_vel[1],-si*world_vel[0]+co*world_vel[1],world_vel[2]])
            for key,value in zip(saved, (np.r_[data.qpos[:7],q],command,velocity,(rotation@body_vel[:3])[2],first_failure is not None,
                        np.r_[q_input-offset,v_input,gyro,gravity,np.asarray(last_action)],action,raw_command)):
                saved[key].append(np.array(value))
            last_action=prev.copy()
        while pending and pending[0][0]<=tick:
            _,target,gain=pending.popleft()
            if support:bank.set_gain_scale(gain)
            bank.latch(target,q_obs,v_obs,soft_limits=cfg['envelope'])
        if not cfg['mapped']:
            if tick%4==0:held_joint_torque=np.clip(kp*(target-q)-kd*v,-limits,limits)
            tau=held_joint_torque.copy()
        elif mode=='mapped':
            # Exact transmission-only control: no PD approximation or motor clipping.
            if tick%4==0:held_joint_torque=np.clip(kp*(target-q)-kd*v,-limits,limits)
            tau=bank.map_motor_torque(bank.map_joint_torque(held_joint_torque))
        else:
            tau=bank.step(q,v,q_obs,v_obs,tick,multirate=cfg['multirate'],envelope=cfg['envelope'],lag_seconds=cfg['lag_ms']/1000)
        if now<trace_seconds and tick%4==0 and first_failure is None:
            for key,value in zip(trace,(now,q,v,q_obs,v_obs,target,bank.motor_target,
                bank.matrix@q+bank.zero,bank.matrix@v,bank.held_request,
                bank.limits(bank.matrix@v),bank.output)):
                trace[key].append(np.array(value).copy())
        if first_failure is None:
            peak_joint_torque=np.maximum(peak_joint_torque,np.abs(tau))
            motors=bank.matrix@q+bank.zero;speed=bank.matrix@v
            if strict and np.any(bank.command_rejection):
                first_failure=now;first_reason='runtime_command_envelope_rejection'
                first_motor=bank.names[int(np.flatnonzero(bank.command_rejection)[0])]
            elif strict and np.any(bank.feedforward_rejection):
                first_failure=now;first_reason='runtime_feedforward_range_rejection'
                first_motor=bank.names[int(np.flatnonzero(bank.feedforward_rejection)[0])]
            elif slotted and np.any(bank.instantaneous_rejection):
                first_failure=now;first_reason='raw_motor_torque_over_envelope'
                first_motor=bank.names[int(np.flatnonzero(bank.instantaneous_rejection)[0])]
            elif data.qpos[2]<.25 or data.xmat[base].reshape(3,3)[2,2]<.5:
                first_failure=now;first_reason='base_height_or_tilt'
            elif cfg['envelope'] and np.any(np.abs(speed)>bank.maximum_speed+1e-5):
                first_failure=now;first_reason='motor_speed_limit';first_motor=bank.names[int(np.argmax(np.abs(speed)/bank.maximum_speed))]
            elif cfg['envelope'] and np.any((motors<bank.hard_lower-(0 if slotted else .005))|(motors>bank.hard_upper+(0 if slotted else .005))):
                first_failure=now;first_reason='motor_hard_position_limit'
                first_motor=bank.names[int(np.flatnonzero((motors<bank.hard_lower-(0 if slotted else .005))|(motors>bank.hard_upper+(0 if slotted else .005)))[0])]
        # Freeze diagnostics at the first protection event. Passive post-trip
        # motion must not inflate the admitted controller's motor statistics.
        if first_failure is not None and trip_motor_diagnostics is None:
            trip_motor_diagnostics=bank.report()
            if first_motor is not None:
                index=bank.names.index(first_motor)
                protection_event={'time_seconds':first_failure,'motor':first_motor,
                    'policy_command':command.tolist(),'requested_motor_torque_nm':float(bank.held_request[index]),
                    'modeled_motor_torque_cap_nm':float(bank.limits(bank.matrix@v)[index]),
                    'motor_speed_rad_s':float((bank.matrix@v)[index]),
                    'motor_position_rad':float((bank.matrix@q+bank.zero)[index]),
                    'held_motor_target_rad':float(bank.motor_target[index]),
                    'kp':float(bank.motor_kp[index]),'kd':float(bank.motor_kd[index]),
                    'p_torque_nm':float((bank.motor_kp*(bank.motor_target-(bank.matrix@q+bank.zero)))[index]),
                    'd_torque_nm':float((-bank.motor_kd*(bank.matrix@v))[index]),
                    'ff_torque_nm':float(bank.held_ff[index]),
                    'cap_scope':'Conservative speed/nameplate and MIT feedforward-field proxy, not measured total motor torque'}
                if slotted:
                    protection_event.update(feedback_estimated_request_nm=float(bank.guard_request[index]),
                        feedback_estimated_cap_nm=float(bank.guard_cap[index]),
                        motor_feedback_age_ms=float((now-measured_stamp[index])*1000))
                    if bank.ankle[index]:
                        protection_event.update(equivalent_matrix_kp=protection_event['kp'],equivalent_matrix_kd=protection_event['kd'],
                            kp=0.,kd=0.,p_torque_nm=0.,d_torque_nm=0.,ff_torque_nm=float(bank.held_request[index]),
                            command_realization='host_ankle_pd_pure_feedforward')
                    else:
                        protection_event.update(kp=float(bank.wire_kp[index]),kd=float(bank.wire_kd[index]),
                            p_torque_nm=float((bank.wire_kp*(bank.motor_target-(bank.matrix@q+bank.zero)))[index]),
                            d_torque_nm=float((bank.wire_kd*(bank.wire_velocity-bank.matrix@v))[index]))
        data.qfrc_applied[:]=0
        if not np.isfinite(tau).all():raise RuntimeError('nonfinite actuation request; simulation invalid')
        data.qfrc_applied[vi]=tau if first_failure is None else 0
        if support:
            mujoco.mj_forward(model,data)
            rotation=data.xmat[base].reshape(3,3);twist=link_frame_twist(model,data,base)
            force,torque=support.wrench(now,data.xpos[base],rotation,rotation@twist[3:],rotation@twist[:3],
                data.subtree_com[base],model.body_subtreemass[base],model.opt.gravity)
            peak_support_force=max(peak_support_force,float(np.linalg.norm(force)))
            peak_support_torque=max(peak_support_torque,float(np.linalg.norm(torque)))
            if now>=support.release:post_release_support=max(post_release_support,float(np.linalg.norm(np.r_[force,torque])))
            mujoco.mj_applyFT(model,data,force,torque,data.xpos[base],base,data.qfrc_applied)
        if tick%4==0 and first_failure is None:
            # Recompute contacts/forces and point velocities at one identical physical state.
            mujoco.mj_forward(model,data);normal=np.zeros(2)
            for k in range(data.ncon):
                con=data.contact[k];g1,g2=con.geom1,con.geom2
                foot=g2 if g1 in floors else g1 if g2 in floors else -1
                if foot not in feet:continue
                index=feet.index(foot);mujoco.mj_contactForce(model,data,k,fvec);force=max(0,float(fvec[0]));normal[index]+=force
                if force>5:
                    mujoco.mj_jac(model,data,jacp,jacr,con.pos,int(model.geom_bodyid[foot]));speed_point=float(np.linalg.norm((jacp@data.qvel)[:2]))
                    loaded_samples[index]+=1;slip_samples[index]+=speed_point>.05
                    normal_total[index]+=force;force_weighted_speed[index]+=force*speed_point
                    max_contact_speed[index]=max(max_contact_speed[index],speed_point)
            force_sums+=normal;unloaded+=normal<5;contact_samples+=1
        time_before=float(data.time)
        mujoco.mj_step(model,data)
        bad_state_warnings=sum(data.warning[i].number for i in (
            mujoco.mjtWarning.mjWARN_BADQPOS,mujoco.mjtWarning.mjWARN_BADQVEL,mujoco.mjtWarning.mjWARN_BADQACC))
        if abs(data.time-time_before-.0005)>1e-9 or bad_state_warnings:
            raise RuntimeError(f'invalid simulation/reset at tick={tick}, t={now}, time={data.time}, warnings={bad_state_warnings}, max_tau={np.max(abs(tau))}, qacc={np.max(abs(data.qacc))}')
        if not np.isfinite(data.qpos).all():raise RuntimeError('nonfinite closed-loop state')
    output=Path(out_path);output.mkdir(parents=True,exist_ok=False)
    arrays={key:np.asarray(value)[:,None] for key,value in saved.items()}
    arrays.update(fps=np.array(50.),case_names=np.array([case['name']]))
    np.savez_compressed(output/'rollout.npz',**arrays)
    if trace_seconds:
        np.savez_compressed(output/'startup-trace.npz',**{k:np.array(v) for k,v in trace.items()},
                            motor_names=np.array(bank.names))
    velocity=arrays['velocity'][:,0];valid_end=round(first_failure/.02) if first_failure is not None else len(velocity)
    start=max(150,valid_end-250);part=velocity[start:valid_end]
    report={'case':case['name'],'command':case['command'],'mode':mode,'seed':seed,'seconds':seconds,'handoff':handoff,
        'settings':cfg,'survived':first_failure is None,'first_failure_seconds':first_failure,'first_failure_reason':first_reason,
        'handoff_seconds':cc['deployment_handoff_seconds'],'command_start_seconds':command_start,
        'experiment':experiment,
        'supported_preposition':({'actor_start_seconds':support.actor_start,'fixture_release_seconds':support.release,
            'preposition_seconds':support.seconds,'initial_gain_scale':support.initial_gain,'final_gain_scale':1.,
            'lowering_seconds':support.lowering,'unloading_seconds':support.unloading,'lift_height_m':support.lift_height,
            'peak_external_fixture_force_n':peak_support_force,'peak_external_fixture_torque_nm':peak_support_torque,
            'maximum_fixture_wrench_after_release':post_release_support,
            'planned_unassisted_evaluation_seconds':max(0,seconds-support.release),
            'admitted_unassisted_evaluation_seconds':max(0,min(seconds,first_failure if first_failure is not None else seconds)-support.release),
            'release_reached_before_trip':first_failure is None or first_failure>=support.release,
            'fixture_peak_scope':'Full scheduled fixture timeline, including passive post-trip states; not an admission metric',
            'scope':'Assumed six-axis external fixture, not an identified rope model. Policy starts during unloading; evaluation after release is unassisted.'} if support else None),
        'peak_motor_feedback_age_ms':peak_state_age_ms,
        'peak_motor_feedback_post_inference_budget_age_ms':peak_post_inference_age_ms,
        'feedback_timing_trace_sha256':file_sha256(Path(feedback_trace)) if feedback_trace else None,
        'feedback_timing_scope':'Private zero-gain receive timing with estimated clock conversion and explicit sample-age margin; not enabled-drive qualification' if feedback_trace else 'constant timing sensitivity',
        'slot_schedule':({'send_period_slots':dict(zip(bank.names,map(int,bank.send_period))),
            'send_phase_slots':dict(zip(bank.names,map(int,bank.send_phase))),
            'internal_pd_phase':bank.internal_phase,'repeat_held_commands':bank.repeat_held_commands,
            'arrival_driven_preview':bank.arrival_driven_preview,
            'wire_quantization':bank.wire_quantization,
            'feedback_quantization':bool(experiment.get('feedback_quantization',False)),
            'ankle_at_send':bank.ankle_at_send,
            'command_sends':dict(zip(bank.names,map(int,bank.command_sends)))} if slotted else None),
        'initial_state':initial_state,
        'first_failure_motor':first_motor,'last5s_velocity_m_s':part.mean(0).tolist() if len(part) else None,
        'protection_event':protection_event,
        'last5s_yaw_rad_s':float(arrays['yaw_rate'][start:valid_end,0].mean()) if len(part) else None,
        'peak_joint_torque_nm':peak_joint_torque.tolist(),
        'motor_diagnostics':trip_motor_diagnostics if trip_motor_diagnostics is not None else bank.report(),
        'motor_diagnostics_scope':'pre_trip_including_trigger_tick' if first_failure is not None else 'full_trial',
        'post_trip_controller_output_applied':False,
        'contact':{'sample_interval_seconds':.002,'synchronized_instantaneous':True,'samples':contact_samples,
            'foot_unloaded_fraction':(unloaded/max(1,contact_samples)).tolist(),
            'loaded_contact_slip_fraction_over_0_05_m_s':(slip_samples/np.maximum(1,loaded_samples)).tolist(),
            'normal_force_weighted_contact_speed_m_s':(force_weighted_speed/np.maximum(1,normal_total)).tolist()},
        'model_sha256':c.data['policy_onnx_sha256'],'scene_sha256':file_sha256(Path(scene_path)),
        'hardware_sha256':file_sha256(Path(hardware_path)),'mujoco_version':mujoco.__version__,'wall_seconds':time.monotonic()-started,
        'motor_envelope_assumptions':bank.assumptions,
        'gain_profile_sha256':file_sha256(Path(profile_path)) if profile_path else None,
        'gain_override_scope':'Kp/Kd ONLY; actor observation/action offsets, effort caps and500Hz serial PD kept' if profile_path else 'checkpoint contract controls',
        'training_run':c.data.get('training_run'),'asset_revision':c.data.get('asset_revision'),
        'control_profile_sha256':c.data.get('control_profile_sha256'),
        'physics_dt':.0005,'policy_hz':50,'motor_internal_hz':1000 if cfg['multirate'] else 500,'ankle_hz':500,'other_motors_hz':50,
        'base_twist_frame':'mjOBJ_XBODY link frame, not principal inertia axes',
        'hardware_tx':0,'hardware_qualified':False,
        'limitations':['Measured constant linear differential, not nonlinear linkage geometry',
          'Motor torque-speed polyline is a conservative nameplate model, not a measured dynamometer curve',
          'Feedback delays and torque response lag are sensitivity assumptions; thermal/current limits not identified',
          'IMU uses physics-derived attitude/gyro, not a full raw-sensor software-estimator model']}
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:report[k] for k in ('case','mode','seed','survived','first_failure_seconds','first_failure_reason','first_failure_motor','last5s_velocity_m_s','wall_seconds')}),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('contract','hardware','scene','commands','output-dir'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--modes',nargs='+',choices=MODES,default=[m for m in MODES if not MODES[m].get('slotted',False)])
    p.add_argument('--seeds',nargs='+',type=int,default=[101])
    p.add_argument('--seconds',type=int,default=30);p.add_argument('--workers',type=int,default=4)
    p.add_argument('--handoff',action='store_true')
    p.add_argument('--gain-profile',type=Path,help='Factorial transfer probe: override Kp/Kd only; not a full training-profile replay')
    p.add_argument('--handoff-seconds',type=float,help='Offline startup experiment; never edits the frozen contract')
    p.add_argument('--command-start-seconds',type=float,default=2.)
    p.add_argument('--trace-seconds',type=float,default=0.)
    p.add_argument('--damping-compensation',action='store_true',help='Offline candidate: sampled host residual damping, embedded Kd ceiling unchanged')
    for flag in ('delay-ms','feedback-ms','lag-ms'):p.add_argument('--'+flag,type=float)
    p.add_argument('--command-linear-accel',type=float)
    p.add_argument('--command-yaw-accel',type=float)
    p.add_argument('--initial-joint-jitter',type=float,default=0.)
    p.add_argument('--initial-base-tilt',type=float,default=0.)
    p.add_argument('--initial-state',type=Path,help='Offline-only measured/counterfactual state; hard guards checked before dynamics')
    p.add_argument('--imu-delay-ms',type=float,default=0.)
    p.add_argument('--inference-budget-ms',type=float,default=0.,help='Post-inference 30ms freshness guard in slot experiments')
    p.add_argument('--internal-phase',type=int,choices=[0,1],default=0)
    p.add_argument('--feedback-timing-trace',type=Path,help='Private 500Hz receive timing sensitivity; repeat mode only')
    p.add_argument('--wire-quantization',action='store_true',help='Quantize actual held MIT position/velocity/gains/feedforward in slot experiments')
    p.add_argument('--feedback-quantization',action='store_true',help='Quantize host motor position/velocity observations in slot experiments')
    p.add_argument('--supported-preposition',action='store_true',help='Virtual external fixture + gain ramp; never a hardware entry')
    p.add_argument('--preposition-seconds',type=float,default=5.)
    p.add_argument('--preposition-initial-gain',type=float,default=.1)
    p.add_argument('--lowering-seconds',type=float,default=2.)
    p.add_argument('--unloading-seconds',type=float,default=1.)
    p.add_argument('--lift-height',type=float,default=.05)
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=False)
    if a.handoff_seconds is not None and (not a.handoff or not np.isfinite(a.handoff_seconds) or a.handoff_seconds<0):
        raise ValueError('handoff duration requires --handoff and must be finite/nonnegative')
    if not np.isfinite(a.command_start_seconds) or a.command_start_seconds<0 or not np.isfinite(a.trace_seconds) or a.trace_seconds<0:
        raise ValueError('invalid command/trace timing')
    experiment=dict(handoff_seconds=a.handoff_seconds,command_start_seconds=a.command_start_seconds,
                    trace_seconds=a.trace_seconds,damping_compensation=a.damping_compensation,
                    delay_ms=a.delay_ms,feedback_ms=a.feedback_ms,lag_ms=a.lag_ms,
                    command_linear_accel=a.command_linear_accel,command_yaw_accel=a.command_yaw_accel,
                    initial_joint_jitter=a.initial_joint_jitter,initial_base_tilt=a.initial_base_tilt,
                    initial_state=str(a.initial_state.resolve()) if a.initial_state else None,
                    imu_delay_ms=a.imu_delay_ms,inference_budget_ms=a.inference_budget_ms,internal_phase=a.internal_phase,
                    wire_quantization=a.wire_quantization,
                    feedback_quantization=a.feedback_quantization,
                    supported_preposition=a.supported_preposition,preposition_seconds=a.preposition_seconds,
                    preposition_initial_gain=a.preposition_initial_gain,
                    lowering_seconds=a.lowering_seconds,unloading_seconds=a.unloading_seconds,lift_height=a.lift_height,
                    feedback_timing_trace=str(a.feedback_timing_trace.resolve()) if a.feedback_timing_trace else None)
    if not all(np.isfinite(x) and x>=0 for x in (a.initial_joint_jitter,a.initial_base_tilt)):
        raise ValueError('initial state perturbations must be finite and nonnegative')
    if (a.command_linear_accel is None)!=(a.command_yaw_accel is None):
        raise ValueError('both command acceleration limits are required together')
    if a.command_linear_accel is not None and not all(np.isfinite(x) and x>0 for x in (a.command_linear_accel,a.command_yaw_accel)):
        raise ValueError('invalid command acceleration limits')
    if any(value is not None and (not np.isfinite(value) or value<0) for value in (a.delay_ms,a.feedback_ms,a.lag_ms)):
        raise ValueError('timing overrides must be finite and nonnegative')
    if not all(np.isfinite(v) and v>=0 for v in (a.imu_delay_ms,a.inference_budget_ms)):
        raise ValueError('invalid IMU delay or inference budget')
    if (a.wire_quantization or a.feedback_quantization) and not all(MODES[m].get('slotted',False) for m in a.modes):
        raise ValueError('wire and feedback quantization are isolated slot-mode experiments')
    cases=json.loads(a.commands.read_text());jobs=[]
    for mode in a.modes:
        for seed in a.seeds:
            for i,case in enumerate(cases):
                jobs.append((a.contract.resolve(),a.hardware.resolve(),a.scene.resolve(),a.output_dir/f'{mode}-s{seed}-c{i:02d}',case,mode,seed,a.seconds,a.handoff,a.gain_profile.resolve() if a.gain_profile else None,experiment))
    with ProcessPoolExecutor(max_workers=a.workers) as pool:results=list(pool.map(run_case,jobs))
    report={'results':results,'survived':sum(r['survived'] for r in results),'trials':len(results),'hardware_tx':0,'hardware_qualified':False}
    (a.output_dir/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
