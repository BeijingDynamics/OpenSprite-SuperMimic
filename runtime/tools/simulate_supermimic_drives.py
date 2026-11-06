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

MODES = {
    'joint_pd': dict(mapped=False, envelope=False, multirate=False, delay_ms=0, feedback_ms=0, lag_ms=0),
    'mapped': dict(mapped=True, envelope=False, multirate=False, delay_ms=0, feedback_ms=0, lag_ms=0),
    'envelope': dict(mapped=True, envelope=True, multirate=False, delay_ms=0, feedback_ms=0, lag_ms=0),
    'multirate': dict(mapped=True, envelope=True, multirate=True, delay_ms=0, feedback_ms=0, lag_ms=0),
    'measured_delay': dict(mapped=True, envelope=True, multirate=True, delay_ms=10, feedback_ms=0, lag_ms=0),
    'runtime': dict(mapped=True, envelope=True, multirate=True, delay_ms=10, feedback_ms=2, lag_ms=5),
    'runtime_strict': dict(mapped=True, envelope=True, multirate=True, delay_ms=10, feedback_ms=0, lag_ms=0),
    'stress': dict(mapped=True, envelope=True, multirate=True, delay_ms=15, feedback_ms=5, lag_ms=10),
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
            if mode!='runtime_strict':raise ValueError('timing overrides are isolated strict-mode experiments')
            cfg[key]=experiment[key]
    controls=dict(c.data)
    if profile_path:
        profile=json.loads(Path(profile_path).read_text())
        if set(profile['joints'])!=set(c.data['joint_names']):raise ValueError('actuator profile joint names differ')
        controls['stiffness']=[profile['joints'][n]['kp'] for n in c.data['joint_names']]
        controls['damping']=[profile['joints'][n]['kd'] for n in c.data['joint_names']]
        if not np.isfinite(np.r_[controls['stiffness'],controls['damping']]).all():raise ValueError('nonfinite gains')
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
    mujoco.mj_forward(model,data)
    if joint_jitter or base_tilt:
        if any(model.geom_type[g]!=mujoco.mjtGeom.mjGEOM_BOX for g in feet):
            raise ValueError('initial ground placement requires the validated box-foot geometry')
        bottom=min(data.geom_xpos[g,2]-abs(data.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g] for g in feet)
        data.qpos[2]+=.001-bottom
        mujoco.mj_forward(model,data)
    initial_state={'root_pose':data.qpos[:7].tolist(),'joint_position':data.qpos[qi].tolist(),
                   'joint_velocity':data.qvel[vi].tolist()}
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
    started=time.monotonic();joint_target=target.copy();last_action=np.zeros(31)
    contact_samples=0;force_sums=np.zeros(2);unloaded=np.zeros(2);slip_samples=np.zeros(2);loaded_samples=np.zeros(2)
    force_weighted_speed=np.zeros(2);normal_total=np.zeros(2);max_contact_speed=np.zeros(2)
    jacp=np.zeros((3,model.nv));jacr=np.zeros_like(jacp)
    fvec=np.zeros(6);peak_joint_torque=np.zeros(31);held_joint_torque=np.zeros(31)
    for tick in range(total_ticks):
        now=tick*.0005
        q=data.qpos[qi].copy();v=data.qvel[vi].copy()
        state_history.append((q.copy(),v.copy()));q_old,v_old=state_history[0]
        qm_old=bank.matrix@q_old+bank.zero;vm_old=bank.matrix@v_old
        due=bank.ankle & (tick%4==bank.phase)
        if tick%40==0:due=due | bank.other
        measured_qm[due]=qm_old[due];measured_vm[due]=vm_old[due]
        q_obs=bank.inverse@(measured_qm-bank.zero);v_obs=bank.inverse@measured_vm
        if not cfg['multirate']:q_obs,v_obs=q_old,v_old
        if tick%policy_ticks==0:
            mujoco.mj_forward(model,data)
            rotation=data.xmat[base].reshape(3,3)
            body_vel=link_frame_twist(model,data,base)
            gyro=body_vel[:3];gravity=-rotation[2]
            command=np.zeros(3) if now<command_start else np.array(case['command'],float)
            for stage in case.get('stages',[]):
                if now>=stage['at_seconds']:command=np.array(stage['command'],float)
            raw_command=command.copy()
            if slew is not None:command=slew.step(command,.02)
            q_input,v_input=(q_obs,v_obs) if cfg['mapped'] else (q,v)
            history.append(q_input,v_input,prev,gyro,gravity)
            obs=history.observation(command)
            action=session.run(None,{session.get_inputs()[0].name:obs[None]})[0][0]
            prev=blend.blend(action,q_input);joint_target=offset+scale*prev
            pending.append((tick+round(cfg['delay_ms']/1000/.0005),joint_target.copy()))
            yaw=np.arctan2(rotation[1,0],rotation[0,0]);co,si=np.cos(yaw),np.sin(yaw)
            world_vel=rotation@body_vel[3:];velocity=np.array([co*world_vel[0]+si*world_vel[1],-si*world_vel[0]+co*world_vel[1],world_vel[2]])
            for key,value in zip(saved, (np.r_[data.qpos[:7],q],command,velocity,(rotation@gyro)[2],first_failure is not None,
                        np.r_[q_input-offset,v_input,gyro,gravity,np.asarray(last_action)],action,raw_command)):
                saved[key].append(np.array(value))
            last_action=prev.copy()
        while pending and pending[0][0]<=tick:
            _,target=pending.popleft();bank.latch(target,q_obs,v_obs,soft_limits=cfg['envelope'])
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
            if mode=='runtime_strict' and np.any(bank.command_rejection):
                first_failure=now;first_reason='runtime_command_envelope_rejection'
                first_motor=bank.names[int(np.flatnonzero(bank.command_rejection)[0])]
            elif mode=='runtime_strict' and np.any(bank.feedforward_rejection):
                first_failure=now;first_reason='runtime_feedforward_range_rejection'
                first_motor=bank.names[int(np.flatnonzero(bank.feedforward_rejection)[0])]
            elif data.qpos[2]<.25 or data.xmat[base].reshape(3,3)[2,2]<.5:
                first_failure=now;first_reason='base_height_or_tilt'
            elif cfg['envelope'] and np.any(np.abs(speed)>bank.maximum_speed+1e-5):
                first_failure=now;first_reason='motor_speed_limit';first_motor=bank.names[int(np.argmax(np.abs(speed)/bank.maximum_speed))]
            elif cfg['envelope'] and np.any((motors<bank.hard_lower-.005)|(motors>bank.hard_upper+.005)):
                first_failure=now;first_reason='motor_hard_position_limit'
                first_motor=bank.names[int(np.flatnonzero((motors<bank.hard_lower-.005)|(motors>bank.hard_upper+.005))[0])]
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
        data.qfrc_applied[:]=0
        if not np.isfinite(tau).all():raise RuntimeError('nonfinite actuation request; simulation invalid')
        data.qfrc_applied[vi]=tau if first_failure is None else 0
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
    p.add_argument('--modes',nargs='+',choices=MODES,default=list(MODES))
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
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=False)
    if a.handoff_seconds is not None and (not a.handoff or not np.isfinite(a.handoff_seconds) or a.handoff_seconds<0):
        raise ValueError('handoff duration requires --handoff and must be finite/nonnegative')
    if not np.isfinite(a.command_start_seconds) or a.command_start_seconds<0 or not np.isfinite(a.trace_seconds) or a.trace_seconds<0:
        raise ValueError('invalid command/trace timing')
    experiment=dict(handoff_seconds=a.handoff_seconds,command_start_seconds=a.command_start_seconds,
                    trace_seconds=a.trace_seconds,damping_compensation=a.damping_compensation,
                    delay_ms=a.delay_ms,feedback_ms=a.feedback_ms,lag_ms=a.lag_ms,
                    command_linear_accel=a.command_linear_accel,command_yaw_accel=a.command_yaw_accel,
                    initial_joint_jitter=a.initial_joint_jitter,initial_base_tilt=a.initial_base_tilt)
    if not all(np.isfinite(x) and x>=0 for x in (a.initial_joint_jitter,a.initial_base_tilt)):
        raise ValueError('initial state perturbations must be finite and nonnegative')
    if (a.command_linear_accel is None)!=(a.command_yaw_accel is None):
        raise ValueError('both command acceleration limits are required together')
    if a.command_linear_accel is not None and not all(np.isfinite(x) and x>0 for x in (a.command_linear_accel,a.command_yaw_accel)):
        raise ValueError('invalid command acceleration limits')
    if any(value is not None and (not np.isfinite(value) or value<0) for value in (a.delay_ms,a.feedback_ms,a.lag_ms)):
        raise ValueError('timing overrides must be finite and nonnegative')
    cases=json.loads(a.commands.read_text());jobs=[]
    for mode in a.modes:
        for seed in a.seeds:
            for i,case in enumerate(cases):
                jobs.append((a.contract.resolve(),a.hardware.resolve(),a.scene.resolve(),a.output_dir/f'{mode}-s{seed}-c{i:02d}',case,mode,seed,a.seconds,a.handoff,a.gain_profile.resolve() if a.gain_profile else None,experiment))
    with ProcessPoolExecutor(max_workers=a.workers) as pool:results=list(pool.map(run_case,jobs))
    report={'results':results,'survived':sum(r['survived'] for r in results),'trials':len(results),'hardware_tx':0,'hardware_qualified':False}
    (a.output_dir/'summary.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
