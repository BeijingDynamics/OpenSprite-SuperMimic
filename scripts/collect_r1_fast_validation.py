"""Audit and locally export the faster-start profile after device processes exit."""
import hashlib
import csv
import json
from pathlib import Path
import subprocess
import sys
import tarfile
import numpy as np

STAGE=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
OLD=Path('/home/tony/sprite-sm-walk-r1-xbox120-20261119')
sys.path.insert(0,'/home/tony/sprite-sm-walk-r1-20261119')
from compare_live_cpu_frequency import audit


def main():
    active=[p for p in subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
            if p.strip().startswith(('sprite_can','sprite_super','sprite_icm'))]
    if active:raise RuntimeError('Control processes remain: '+str(active))
    subprocess.run(['bash',str(STAGE/'operator-xbox.sh'),'--check'],check=True)
    live,_,fingerprints=audit(STAGE/'sleeping-disabled-01')
    repeat,_,repeat_fingerprints=audit(STAGE/'sleeping-disabled-02')
    assert repeat_fingerprints==fingerprints
    fault=json.loads((STAGE/'sleeping-fault-01/commissioning.json').read_text())
    old=json.loads((OLD/'fixed-disabled-01/commissioning.json').read_text())
    unchanged=['policy.onnx','sprite_icm45686','motors.tsv','kinematics.tsv','joints.tsv',
               'feedback.bin','target.bin','input.bin',
               'cpu_frequency_client.py','cpu_frequency_guard.py','action_tx_audit.py']
    for key in unchanged:assert fingerprints[key]==old['fingerprints'][key],key
    assert fingerprints['loaded_kcan_srcversion']=='404E8D65A6EFF60079EAEC1'
    inputs=np.fromfile(STAGE/'sleeping-disabled-01/native.bin.inputs',dtype='<f8').reshape(-1,71)
    assert inputs.shape==(7301,71) and not np.any(inputs[:,68:71])
    tx_coverage={}
    for name in ['sleeping-disabled-01','sleeping-disabled-02','sleeping-fault-01']:
        report=json.loads((STAGE/name/'commissioning.json').read_text())
        can=json.loads((STAGE/name/'can.json').read_text())
        assert report['passed'] and report['final_disabled_count']==31 and report['motor_enable_attempts']==0
        assert report['inference_no_spinning'] is True
        assert report['feedback_torque_mode']=='observe_only' and report['xbox_forward_acceleration_m_s2']==.10
        assert all(m['feedback_torque_observe_only'] for m in can['motors'].values())
        affinity=report['inference_affinity']
        assert affinity['verified'] and affinity['main_cpu']==4 and affinity['ort_worker_cpu']==5 and affinity['io_cpu']==1
        total=missing=0
        with (STAGE/name/'can.json.tx.csv').open() as stream:
            for row in csv.DictReader(stream):
                total+=1
                missing+=int(not (int(row['software_realtime_ns'])>0 and int(row['hardware_raw_ns'])>0 and int(row['hardware_received_ns'])>0))
        tx_coverage[name]=dict(total=total,missing=missing)
        assert total>0 and missing==0,(name,tx_coverage[name])
    since=int((STAGE/'sleeping-disabled-01/can-before.json').stat().st_mtime)
    kernel=subprocess.check_output(['journalctl','-k','--since','@'+str(since),'--no-pager'],text=True)
    (STAGE/'qualified-kernel-log.txt').write_text(kernel)
    assert not any('echo_skb' in line and 'occupied' in line for line in kernel.splitlines()),'echo ownership race recurred'
    result=dict(profile='Sprite SM-Walk R1 faster start / feedback torque observation',live=live,repeat=repeat,
                fault_passed=fault['passed'],final_disabled_count=31,motor_enable_attempts=0,
                driver_srcversion=fingerprints['loaded_kcan_srcversion'],echo_occupied_messages=0,
                all_packet_timestamp_coverage=tx_coverage,
                unchanged_fingerprints={k:fingerprints[k] for k in unchanged},
                changed_behavior=['Forward acceleration 0.05 to 0.10 m/s^2.',
                                  'Instantaneous feedback torque excursions recorded without stopping in this Xbox profile.',
                                  'Verified main CPU 4, ORT worker CPU 5, sensor/gamepad IO CPU 1. CAN CPU 2 / FIFO 50 unchanged.',
                                  'Every coherent action refreshes all motor targets and commits in the current transport slot.',
                                  'CAN mapping compiled with -O2 -ffp-contract=off; no fast-math.',
                                  'Timestamp driver registers echo skb before publishing TX and cleans failed submissions.',
                                  'Two ORT threads with idle spinning disabled; policy recurrence unchanged.'],
                limitations=['Torque RMS remains telemetry, not an independent stop condition.',
                             'Earlier shared-layout failures are retained; passing finite trials do not establish hard realtime guarantees.',
                             'Full-scale feedback cannot reveal torque beyond its measurement range.',
                             'Prior R1 runs include real inference and TX-receipt tails; no timing protection was relaxed.',
                             'No powered run of this profile has been performed by deployment.'])
    (STAGE/'deployment-summary.json').write_text(json.dumps(result,indent=2)+'\n')
    members=['deployment-summary.json','build-manifest.json','operator-xbox.sh','run-disabled.py',
             'ensure-can-scheduler.sh','cpu-replay','sleeping-disabled-01','sleeping-disabled-02','sleeping-fault-01',
             'sleeping-disabled-01.log','sleeping-fault-01.log','sleeping-disabled-01-job.json','sleeping-fault-01-job.json',
             'sleeping-disabled-02.log','sleeping-disabled-02-job.json',
             'diagnostic-shared-01','diagnostic-shared-01.log','diagnostic-isolated-01','diagnostic-isolated-01.log',
             'diagnostic-parity','layout-parity','build-manifest-before-thread-isolation.json',
             'build-manifest-before-atomic-commit.json','r1_atomic_replay.cpp',
             'atomic-replay-o0.bin','atomic-replay-o2.bin',
             'driver-echo-order','driver-echo-order.patch','driver-echo-order.sha256',
             'driver-echo-order-manifest.json','driver-echo-order-build.log','load-echo-order-driver.sh',
             'echo-race-kernel-evidence.txt',
             'qualified-kernel-log.txt',
             'build-manifest-before-inference-modes.json',
             'build/sprite_can_shadow','build/sprite_supermimic_protected_ipc','bundle/policy.onnx',
             'bundle/bundle.json','bundle/contract.json','bundle/training_control_profile.json','operator-status.log']
    members += [p.name for p in STAGE.glob('failed-*')]
    members += [p.name for p in STAGE.glob('fixed-disabled-*')]
    members += [p.name for p in STAGE.glob('single-disabled-*')]
    members += ['inference-mode-parity']
    members += [p.name for p in STAGE.glob('passed-atomic-previous-driver-*')]
    archive=STAGE/'validation-evidence.tar.gz'
    with tarfile.open(archive,'w:gz',compresslevel=1) as stream:
        for name in members:stream.add(STAGE/name,arcname=name)
    meta=dict(bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    (STAGE/'validation-manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(dict(live=live,fault_passed=fault['passed'],archive=meta),indent=2))


if __name__=='__main__':main()
