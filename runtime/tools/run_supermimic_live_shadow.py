"""Supervise C++ live zero-gain CAN, ICM and receive-only actor; no Python sensor/control loop."""
from pathlib import Path
import argparse
import json
import os
import subprocess
import sys
import time

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.supermimic_replay import write_fixture
from open_sprite_runtime.supermimic_run002 import verify_bundle
from open_sprite_runtime.socketcan import audit_socketcan_active_fd_snapshot
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--bundle',type=Path,required=True)
    p.add_argument('--hardware',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=10)
    p.add_argument('--supported-unloaded',action='store_true',required=True)
    p.add_argument('--acknowledge-zero-gain-tx',action='store_true',required=True)
    p.add_argument('--can-cpu',type=int,default=2);p.add_argument('--imu-cpu',type=int,default=3)
    p.add_argument('--actor-cpus',type=int,nargs='+',default=[4,5])
    args = p.parse_args()
    if not 1 <= args.seconds <= 60: p.error('bounded to 1..60 seconds')
    verify_bundle(args.bundle)
    contract=PolicyContract.load(args.bundle/'contract.json')
    args.output.mkdir(parents=True,exist_ok=False)
    out=args.output.resolve();bundle=args.bundle.resolve()
    env=dict(os.environ,PYTHONPATH=str(ROOT/'src'))
    subprocess.run([sys.executable,str(ROOT/'tools/prepare_supermimic_live_shadow.py'),
        '--bundle',str(bundle),'--hardware',str(args.hardware.resolve()),'--output',str(out/'fixtures'),
        '--frames',str(args.seconds*50)],check=True,env=env)
    logs=[];children=[];report={'passed':False,'errors':[],'motor_enable_attempts':0,
        'all_realtime_loops_cpp':True,'python_role':'offline preparation, process supervision and post-run parity only',
        'physical_sensor_access':True,'policy_sha256':contract.data['policy_onnx_sha256'],
        'command':[0,0,0],'handoff_seconds':1.5,'hardware_qualified':False}
    snapshot=lambda:subprocess.check_output(['ip','-j','-d','-s','link','show','type','can'])
    before=snapshot();(out/'can-before.json').write_bytes(before)
    hardware=json.loads(args.hardware.read_text())
    for interface in hardware['can_adapter']['interfaces']:
        preflight=audit_socketcan_active_fd_snapshot(json.loads(before),interface,
            arbitration_bitrate=1_000_000,data_bitrate=5_000_000)
        if not preflight.passed:raise RuntimeError('CAN preflight failed on '+interface+': '+repr(preflight.errors))
    def launch(name,command):
        f=(out/(name+'.log')).open('w');logs.append(f)
        process=subprocess.Popen(command,stdout=f,stderr=subprocess.STDOUT,env=env);children.append(process)
        return process
    try:
        imu_socket=str(out/'imu.sock');state_socket=str(out/'state.sock')
        imu_env=dict(env,ICM_IPC_SOCKET=imu_socket)
        f=(out/'imu.log').open('w');logs.append(f)
        imu=subprocess.Popen(['taskset','-c',str(args.imu_cpu),str(ROOT/'build/icm/sprite_icm45686'),
            'capture','/dev/spidev0.0','/dev/gpiochip0','123','800',str(args.seconds+25),
            str(out/'imu.csv'),'X_DOWN_Z_BACK','CONFIGURE_ICM45686_RAW_NO_CAN'],stdout=f,stderr=subprocess.STDOUT,env=imu_env)
        children.append(imu);deadline=time.monotonic()+25
        while 'BIAS_READY' not in (out/'imu.log').read_text():
            if imu.poll() is not None: raise RuntimeError('ICM exited before stationary calibration')
            if time.monotonic()>deadline:raise TimeoutError('ICM stationary calibration timeout')
            time.sleep(.1)
        print('LIVE_SHADOW_IMU_READY',flush=True)
        can=launch('can',['taskset','-c',str(args.can_cpu),str(ROOT/'build/native/sprite_zero_gain_can_probe'),
            str(out/'fixtures/inventory.tsv'),str(args.seconds+3),'500',str(out/'can'),
            'ZERO_GAIN_DISABLED_MOTORS_ONLY','--state-socket',state_socket,'--state-frames',str(args.seconds*50)])
        deadline=time.monotonic()+3
        while not Path(state_socket).exists():
            if can.poll() is not None:raise RuntimeError('CAN state server failed before actor connection')
            if time.monotonic()>deadline:raise TimeoutError('CAN state socket startup timeout')
            time.sleep(.01)
        report['actor_start_monotonic_ns']=time.monotonic_ns()
        actor=launch('actor',['taskset','-c',','.join(map(str,args.actor_cpus)),
            str(ROOT/'build/native/sprite_supermimic_ipc_readonly'),str(bundle/'policy.onnx'),
            str(out/'fixtures/input.bin'),str(out/'native.bin'),'--state-socket',state_socket,
            '--imu-socket',imu_socket,'--feedback-config',str(out/'fixtures/feedback.bin'),
            '--imu-kind','icm','--threads',str(len(args.actor_cpus)),'--warmup','100'])
        actor_rc=actor.wait(timeout=args.seconds+12);report['actor_end_monotonic_ns']=time.monotonic_ns()
        can_rc=can.wait(timeout=10);imu_rc=imu.wait(timeout=20)
        report.update(actor_returncode=actor_rc,can_returncode=can_rc,imu_returncode=imu_rc)
        if actor_rc or can_rc or imu_rc:raise RuntimeError('a native component failed; inspect its log')
        consumed=np.fromfile(out/'native.bin.inputs',dtype='<f8').reshape(-1,71)
        write_fixture(out/'consumed.bin',contract,consumed,75)
        offline=launch('offline',[str(ROOT/'build/native/sprite_supermimic_replay'),str(bundle/'policy.onnx'),
            str(out/'consumed.bin'),str(out/'offline.bin'),'--threads',str(len(args.actor_cpus))])
        offline_rc=offline.wait(timeout=60)
        report['offline_byte_exact']=offline_rc==0 and (out/'native.bin').read_bytes()==(out/'offline.bin').read_bytes()
        report['actor']=json.loads((out/'native.bin.timing.json').read_text())
        report['can']=json.loads((out/'can.json').read_text())
        report['imu']=json.loads((out/'imu.csv.json').read_text())
        report['passed']=bool(report['offline_byte_exact'] and report['can']['passed'] and
            report['can']['state_frames_sent']==args.seconds*50 and report['can']['actor_write_half_closed'] and
            report['can']['state_socket_reverse_bytes']==0 and report['actor']['frames']==args.seconds*50 and
            report['actor']['imu_kind']=='icm' and report['actor']['deadline_misses']==0)
    except Exception as ex:
        report['errors'].append(type(ex).__name__+': '+str(ex))
    finally:
        for process in children:
            if process.poll() is None:process.terminate()
        for process in children:
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        for f in logs:f.close()
        (out/'can-after.json').write_bytes(snapshot())
        (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('actor','can','imu')}),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
