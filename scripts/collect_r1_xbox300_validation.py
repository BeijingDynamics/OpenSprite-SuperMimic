"""Collect five-minute no-enable evidence after all device processes have exited."""
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile

import numpy as np

stage = Path('/home/tony/sprite-sm-walk-r1-xbox300-20261119')
old = Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
sys.path.insert(0, '/home/tony/sprite-sm-walk-r1-20261119')
from compare_live_cpu_frequency import audit


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8*1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    active = [name.strip() for name in subprocess.check_output(['ps', '-eo', 'comm='], text=True).splitlines()
              if name.strip().startswith(('sprite_can', 'sprite_super', 'sprite_icm'))]
    assert not active, active
    subprocess.run(['bash', str(stage/'operator-xbox.sh'), '--check'], check=True)
    manifest_path = stage/'build-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    for name, expected in manifest['files'].items():
        if name != 'operator-xbox.sh':
            assert digest(stage/name) == expected, name
    (stage/'build-manifest-before-evidence-selection.json').write_text(manifest_path.read_text())
    manifest['files']['operator-xbox.sh'] = digest(stage/'operator-xbox.sh')
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    live, _, fingerprints = audit(stage/'disabled-04')
    original = json.loads((old/'sleeping-disabled-02/commissioning.json').read_text())
    unchanged = ['policy.onnx', 'sprite_icm45686', 'motors.tsv', 'kinematics.tsv', 'joints.tsv',
                 'feedback.bin', 'target.bin', 'cpu_frequency_client.py', 'cpu_frequency_guard.py',
                 'action_tx_audit.py', 'loaded_kcan_srcversion']
    for key in unchanged:
        assert fingerprints[key] == original['fingerprints'][key], key
    coverage = {}
    for name in ['disabled-04', 'fault-02']:
        report = json.loads((stage/name/'commissioning.json').read_text())
        assert report['passed'] and report['motor_enable_attempts'] == 0 and report['final_disabled_count'] == 31
        assert report['duration_s'] == 326 and report['training_full_body']
        assert report['fingerprints'] == fingerprints
        assert report['inference_no_spinning'] and report['inference_affinity']['verified']
        assert report['cpu_frequency']['released']['status'] == 'restored'
        assert report['feedback_torque_mode'] == 'observe_only' and report['xbox_forward_acceleration_m_s2'] == .10
        total = missing = 0
        buses = {}
        with (stage/name/'can.json.tx.csv').open() as stream:
            for row in csv.DictReader(stream):
                total += 1
                missing += int(not all(int(row[k]) > 0 for k in
                                       ['software_realtime_ns', 'hardware_raw_ns', 'hardware_received_ns']))
                bus = row.get('interface', row.get('bus', 'unknown'))
                buses[bus] = buses.get(bus, 0)+1
        assert total > 0 and missing == 0
        coverage[name] = dict(total=total, missing=missing, per_bus=buses)
    inputs = np.fromfile(stage/'disabled-04/native.bin.inputs', dtype='<f8').reshape(-1, 71)
    assert inputs.shape == (16301, 71)
    assert not np.any(inputs[:1002, 68:71]) and not np.any(inputs[16205:, 68:71])
    interrupt = json.loads((stage/'interrupt-02/interrupt-verification.json').read_text())
    assert interrupt['passed'] and interrupt['final_disabled_count'] == 31
    since = int((stage/'disabled-04/can-before.json').stat().st_mtime)
    kernel = subprocess.check_output(['journalctl', '-k', '--since', '@'+str(since), '--no-pager'], text=True)
    (stage/'qualified-kernel-log.txt').write_text(kernel)
    assert not any('echo_skb' in line and 'occupied' in line for line in kernel.splitlines())
    result = dict(profile='Sprite SM-Walk R1 Xbox 300 seconds', passed=True,
                  control_window_s=300, total_control_s=326, live=live,
                  deadline_fault_passed=True, interrupt=interrupt,
                  all_packet_timestamp_coverage=coverage, echo_occupied_messages=0,
                  unchanged_fingerprints={key: fingerprints[key] for key in unchanged},
                  changes=['Xbox command window increased from 120 to 300 seconds.',
                           'Bounded fixture capacity increased to 16301 policy frames.',
                           'TX trace capacity increased to 700000 events per socket.',
                           'IMU capture and post-control audit limits cover the longer recording.',
                           'IPC closes after disable readback and before trace serialization.'],
                  limitations=['No powered five-minute run was initiated during deployment.',
                               'Finite no-enable timing validation is not a hard real-time guarantee.',
                               'Two preceding no-enable trials hit the unchanged 20 ms guard after an approximately 11.5 ms actor receipt delay; root scheduler cause remains unresolved.',
                               'LT release brakes the command; Ctrl+C requests shutdown, not controlled braking.',
                               'The dormant LB+RB stop combo is not enabled in this production entry.'])
    (stage/'deployment-summary.json').write_text(json.dumps(result, indent=2)+'\n')
    # Keep exact deployed source, binaries, model and raw timing evidence together.
    members = ['deployment-summary.json', 'build-manifest.json', 'operator-xbox.sh', 'run-disabled.py',
               'ensure-can-scheduler.sh', 'load-echo-order-driver.sh', 'runtime', 'native', 'tools',
               'bundle', 'fixture-capacity-check', 'disabled-01', 'disabled-01.log', 'disabled-01-job.json',
               'disabled-02', 'disabled-02.log', 'disabled-02-job.json',
               'disabled-03', 'disabled-03.log', 'disabled-03-job.json',
               'disabled-04', 'disabled-04.log', 'disabled-04-job.json',
               'fault-02', 'fault-02.log', 'fault-02-job.json',
               'fault-01', 'fault-01.log', 'fault-01-job.json',
               'interrupt-02', 'interrupt-02.log', 'interrupt-02-status.log',
               'interrupt-01', 'interrupt-01.log', 'interrupt-01-status.log', 'pose-after-reboot-01',
               'operator-status.log', 'qualified-kernel-log.txt',
               'build/sprite_can_shadow', 'build/sprite_supermimic_protected_ipc']
    archive = stage/'validation-evidence.tar.gz'
    with tarfile.open(archive, 'w:gz', compresslevel=1) as stream:
        for name in members:
            stream.add(stage/name, arcname=name)
        powered = old/'xbox-powered-20261119-165413-12501'
        stream.add(powered, arcname='previous-120s-powered/'+powered.name)
    meta = dict(bytes=archive.stat().st_size, sha256=digest(archive))
    (stage/'validation-manifest.json').write_text(json.dumps(meta, indent=2)+'\n')
    print(json.dumps(dict(passed=True, coverage=coverage, timing=live['metrics'], archive=meta), indent=2))


if __name__ == '__main__':
    main()
