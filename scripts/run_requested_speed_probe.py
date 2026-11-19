"""Evaluate the retained upright-clearance policy at requested forward speeds."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'outputs/upright-clearance-speed-probe-20261118'
CHECKPOINT = ROOT / 'outputs/targeted-upright-clearance-20261118/train-upright-clearance-3000/model_003000.pt'
EXPECTED = '884946b4f03a03d53057208bfdafe3b0c116f91fec06c493ec7307a49a9c5e4c'
MOTION = ROOT / 'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz'


def main():
    if hashlib.sha256(CHECKPOINT.read_bytes()).hexdigest() != EXPECTED:
        raise ValueError('Checkpoint differs from the retained 3000-update model')
    OUT.mkdir(exist_ok=False)
    state = dict(phase='preparing', checkpoint_sha256=EXPECTED, hardware_access=False,
                 started_unix=time.time(), completed=[])
    def save(phase):
        state.update(phase=phase, updated_unix=time.time())
        (OUT / 'status.json').write_text(json.dumps(state, indent=2) + '\n')
    def run(label, args, timeout):
        save(label)
        with (OUT / (label + '.log')).open('x') as log:
            subprocess.run([sys.executable, *map(str, args)], cwd=ROOT,
                           env={**os.environ, 'MUJOCO_GL':'egl', 'OMP_NUM_THREADS':'2',
                                'OPENBLAS_NUM_THREADS':'1', 'PYTHONUNBUFFERED':'1'},
                           stdout=log, stderr=subprocess.STDOUT, check=True, timeout=timeout)
        state['completed'].append(label)
    try:
        cases = [dict(name=name, command=[speed, 0, 0], category=category)
                 for name, speed, category in [('zero', 0, 'standing_control'),
                    ('forward025', .25, 'prior_speed_control'),
                    ('forward050', .5, 'requested_in_range'),
                    ('forward100', 1., 'requested_outside_training_range')]]
        commands = OUT / 'commands.json'
        commands.write_text(json.dumps(cases, indent=2) + '\n')
        evaluation = OUT / 'eval'
        run('evaluate', ['scripts/evaluate_joint.py', '--checkpoint', CHECKPOINT,
            '--motion', MOTION, '--interface', 'velocity', '--seconds', '22',
            '--seed', '101', '--commands-file', commands, '--output', evaluation], 900)
        run('posture', ['scripts/score_posture_width.py', evaluation], 300)
        run('geometry', ['scripts/summarize_locomotion_geometry.py', '--evaluation', evaluation,
            '--output', evaluation / 'locomotion-geometry.json'], 300)
        run('video', ['scripts/render_velocity.py', '--rollout', evaluation / 'rollout.npz',
            '--output', OUT / 'speed-probe.mp4', '--envs', '0', '1', '2', '3',
            '--seconds', '22', '--bright', '--camera-distance', '1.7', '--camera-azimuth', '135'], 900)
        save('complete')
    except Exception:
        state['error'] = traceback.format_exc()
        save('failed')
        raise


if __name__ == '__main__':
    main()
