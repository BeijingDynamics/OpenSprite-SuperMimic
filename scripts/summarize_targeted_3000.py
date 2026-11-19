"""Summarize retained physical rollouts without executing a policy or physics."""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from opensprite_tracker.robot import BODY_JOINT_NAMES


def main():
    groups = [
        ('targeted-upright-clearance-20261118', ['control', 'upright', 'upright-clearance'], 3000),
        ('evidence-followup-20261118', ['matched-control', 'one-factor'], 600),
    ]
    report = {'scoring_start_seconds': 3, 'scoring_end_seconds': 20, 'evaluations': {}}
    for experiment, arms, iteration in groups:
        base = ROOT / 'outputs' / experiment / 'evidence'
        for arm in arms:
            label = f'eval-{arm}-{iteration}'
            folder = base / label
            geometry = json.loads((folder / 'locomotion-geometry.json').read_text())
            posture = {r['case_name']: r for r in json.loads((folder / 'posture-width.json').read_text())['results']}
            digest = hashlib.sha256((base / f'{label}-checkpoint.pt').read_bytes()).hexdigest()
            if digest != geometry['checkpoint_sha256']:
                raise ValueError(f'Checkpoint hash mismatch: {label}')
            rows = []
            with np.load(folder / 'rollout.npz') as trace:
                names = list(trace['case_names'])
                knee_ids = [7 + BODY_JOINT_NAMES.index(side + '_knee_joint') for side in ['left', 'right']]
                for row in geometry['results']:
                    name = row['case_name']
                    pose = trace['qpos'][150:, names.index(name)]
                    geo, post = row['geometry'], posture[name]
                    rows.append(dict(
                        case=name,
                        median_signed_knee_joint_deg=np.rad2deg(np.median(pose[:, knee_ids], axis=0)).tolist(),
                        torso_pitch_deg=post['pitch_median_deg'],
                        foot_center_spacing_p05_mm=1000 * post['center_p05_m'],
                        crossed_fraction=post['crossed_fraction'],
                        clearance_events=geo['clearance_event_counts'],
                        median_swing_peak_mm=[None if h is None else 1000 * h for h in geo['event_peak_median_m']],
                        mean_vx_m_s=row['mean_vx_m_s'], mean_yaw_rad_s=row['mean_yaw_rad_s'],
                        survived=row['survived'],
                    ))
            report['evaluations'][f'{experiment}/{label}'] = {'checkpoint_sha256': digest, 'cases': rows}
    output = ROOT / 'outputs/targeted-upright-clearance-20261118/evidence/three-arms-3000-and-standing600-audit.json'
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(f'Verified and summarized {len(report["evaluations"])} checkpoints: {output}')


if __name__ == '__main__':
    main()
