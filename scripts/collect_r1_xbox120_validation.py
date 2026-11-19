"""Collect extended R1 validation only after every device process has exited."""
import hashlib
import json
from pathlib import Path
import subprocess
import struct
import sys
import tarfile
import numpy as np

STAGE = Path('/home/tony/sprite-sm-walk-r1-xbox120-20261119')
OLD = Path('/home/tony/sprite-sm-walk-r1-20261119')
sys.path.insert(0, str(OLD))
from compare_live_cpu_frequency import audit


def main():
    names = subprocess.check_output(['ps', '-eo', 'comm='], text=True).splitlines()
    active = [n for n in names if n.strip().startswith(
        ('sprite_can', 'sprite_zero', 'sprite_policy', 'sprite_super', 'sprite_icm'))]
    if active:
        raise RuntimeError('Control processes remain: ' + str(active))
    subprocess.run(['bash', str(STAGE / 'operator-xbox.sh'), '--check'], check=True)
    summary, _, fingerprints = audit(STAGE / 'fixed-disabled-01')
    fault = json.loads((STAGE / 'fixed-fault-01/commissioning.json').read_text())
    previous = json.loads((OLD / 'fixed-disabled-01/commissioning.json').read_text())
    unchanged = ['policy.onnx', 'sprite_icm45686', 'motors.tsv', 'kinematics.tsv',
                 'joints.tsv', 'feedback.bin', 'target.bin',
                 'loaded_kcan_srcversion', 'cpu_frequency_client.py', 'cpu_frequency_guard.py']
    for key in unchanged:
        assert fingerprints[key] == previous['fingerprints'][key], key
    # The fixture frame count changes with duration; gains, scales and each state do not.
    old_fixture = (OLD / 'fixed-disabled-01/input.bin').read_bytes()
    new_fixture = (STAGE / 'fixed-disabled-01/input.bin').read_bytes()
    assert old_fixture[:24] == new_fixture[:24]
    assert old_fixture[28:776] == new_fixture[28:776]
    assert struct.unpack_from('<I', old_fixture, 24)[0] == 2701
    assert struct.unpack_from('<I', new_fixture, 24)[0] == 7301
    old_states = np.frombuffer(old_fixture, dtype='<f8', offset=776).reshape(2701, 71)
    new_states = np.frombuffer(new_fixture, dtype='<f8', offset=776).reshape(7301, 71)
    assert np.all(old_states == old_states[0]) and np.all(new_states == old_states[0])
    inputs = np.fromfile(STAGE / 'fixed-disabled-01/native.bin.inputs', dtype='<f8').reshape(-1, 71)
    assert inputs.shape == (7301, 71) and not np.any(inputs[:, 68:71])
    assert summary['passed'] and fault['passed']
    assert fault['motor_enable_attempts'] == 0 and fault['final_disabled_count'] == 31
    report = dict(profile='Sprite SM-Walk R1 Xbox 120 s / 0.40 m/s', live=summary,
                  deadline_fault_passed=True, deadline_final_disabled_count=31,
                  input_frames=len(inputs), all_commands_zero=True,
                  known_limitations=['A same-build earlier fault-test attempt had a genuine 15.44 ms inference tail, crossed the 20 ms feedback deadline, and stopped before scheduled injection. This is retained as failed-natural-tail-01. Passing runs do not establish tail-free operation.'],
                  unchanged_fingerprints={k: fingerprints[k] for k in unchanged},
                  powered_test_performed=False, final_control_processes=active)
    (STAGE / 'deployment-summary.json').write_text(json.dumps(report, indent=2) + '\n')
    archive = STAGE / 'validation-evidence.tar.gz'
    members = ['deployment-summary.json', 'build-manifest.json', 'operator-xbox.sh',
               'run-disabled.py', 'ensure-can-scheduler.sh', 'cpu-replay',
               'fixed-disabled-01', 'fixed-fault-01', 'fixed-disabled-01.log',
               'fixed-fault-01.log', 'fixed-disabled-01-job.json', 'fixed-fault-01-job.json',
               'failed-capacity-disabled-01', 'failed-capacity-disabled-01.log',
               'failed-capacity-disabled-01-job.json',
               'failed-natural-tail-01', 'failed-natural-tail-01.log',
               'failed-natural-tail-01-job.json',
               'bundle/bundle.json', 'bundle/contract.json', 'bundle/training_control_profile.json',
               'operator-status.log']
    with tarfile.open(archive, 'w:gz', compresslevel=1) as stream:
        for name in members:
            stream.add(STAGE / name, arcname=name)
    meta = dict(bytes=archive.stat().st_size, sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    (STAGE / 'validation-manifest.json').write_text(json.dumps(meta, indent=2) + '\n')
    print(json.dumps(dict(live_passed=summary['passed'], frames=summary['frames_dispatched'],
                         fault_passed=fault['passed'], metrics=summary['metrics'], archive=meta), indent=2))


if __name__ == '__main__':
    main()
