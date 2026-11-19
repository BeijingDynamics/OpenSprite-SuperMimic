"""Extend only the isolated five-minute fixture capacity and recheck CPU recurrence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('stage', type=Path)
    parser.add_argument('--build', action='store_true')
    args = parser.parse_args()
    stage = args.stage
    for name, old, new in [
        ('runtime/src/open_sprite_runtime/supermimic_replay.py', '<= 10000', '<= 16301'),
        ('runtime/src/open_sprite_runtime/supermimic_replay.py', '1..10000 recorded', '1..16301 recorded'),
        ('native/src/sprite_supermimic_replay.cpp', 'count>10000 ||', 'count>16301 ||'),
        ('operator-xbox.sh', 'disabled-01', 'disabled-02'),
    ]:
        path = stage/name
        content = path.read_text()
        assert old in content, (name, old)
        path.write_text(content.replace(old, new), newline='\n')
    if not args.build:
        return
    can_hash = digest(stage/'build/sprite_can_shadow')
    subprocess.run(['cmake', '--build', str(stage/'build'), '-j2', '--target',
                    'sprite_supermimic_replay', 'sprite_supermimic_protected_ipc'], check=True)
    assert digest(stage/'build/sprite_can_shadow') == can_hash
    subprocess.run([str(stage/'build/sprite_can_shadow'), '--check-commissioning-scheduler', '2'], check=True)
    subprocess.run(['ctest', '--test-dir', str(stage/'build'), '--output-on-failure'], check=True)
    sys.path.insert(0, str(stage/'runtime/src'))
    import numpy as np
    from open_sprite_runtime.contracts import PolicyContract
    from open_sprite_runtime.supermimic_replay import write_fixture
    contract = PolicyContract.load(stage/'bundle/contract.json')
    out = stage/'fixture-capacity-check'
    out.mkdir()
    for count in [0, 16301, 16302]:
        values = np.zeros((count, 71))
        try:
            write_fixture(out/f'{count}.bin', contract, values, 0)
            assert count == 16301
        except ValueError:
            assert count != 16301
    # A valid large fixture must pass native parsing and reach model loading.
    env = {**os.environ, 'LD_LIBRARY_PATH': str(stage/'build')}
    probe = subprocess.run([str(stage/'build/sprite_supermimic_replay'), '/nonexistent-model.onnx',
                            str(out/'16301.bin'), str(out/'unused.bin')],
                           env=env, text=True, capture_output=True)
    (out/'native-parse.log').write_text(probe.stdout+probe.stderr)
    assert probe.returncode != 0 and 'fixture count/handoff out of bounds' not in probe.stderr
    assert 'nonexistent-model.onnx' in probe.stderr
    parity = out/'parity.bin'
    command = ['taskset', '-c', '4,5', str(stage/'build/sprite_supermimic_replay'),
               str(stage/'bundle/policy.onnx'),
               '/home/tony/sprite-sm-walk-r1-20261119/cpu-replay/env-00-handoff-off/input.bin',
               str(parity), '--threads', '2', '--warmup', '100', '--isolated-inference',
               '--benchmark', '--no-ort-spinning']
    with (out/'parity.log').open('x') as stream:
        subprocess.run(command, env=env, stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=90)
    expected = '0bc4612dab90c5e78b8cb59a82110b07cb885bf5b4e3d7cec71e6a26dd8ac076'
    assert digest(parity) == expected
    report = dict(passed=True, frame_limit=16301, rejected_counts=[0, 16302],
                  recurrence_sha256=expected, can_binary_unchanged=can_hash, hardware_tx=0)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    manifest_path = stage/'build-manifest.json'
    manifest = json.loads(manifest_path.read_text())
    (stage/'build-manifest-before-fixture-capacity.json').write_text(manifest_path.read_text())
    for name in list(manifest['files'])+['runtime/src/open_sprite_runtime/supermimic_replay.py']:
        manifest['files'][name] = digest(stage/name)
    manifest_path.write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
