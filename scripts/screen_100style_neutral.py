"""Screen official-trim Neutral locomotion; never admit raw names as quality proof."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.signal import find_peaks, savgol_filter

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from screen_natural_lafan import features, wrap
from retarget.bvh_tools.bvh_adapters import inspect_bvh, bvh_skeleton_signature, validate_geno_skeleton, read_transforms


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-dir', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--per-category', type=int, default=2)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    with (args.source_dir / 'Frame_Cuts.csv').open() as stream:
        cuts = next(row for row in csv.DictReader(stream) if row['STYLE_NAME'] == 'Neutral')
    all_rows, sources = [], []
    for path in sorted(args.source_dir.glob('Neutral_*.bvh')):
        kind = path.stem.split('_')[1]
        info = inspect_bvh(path)
        validate_geno_skeleton(info, bvh_skeleton_signature(path).local_rest_offsets_m)
        data = features(path)
        _, names, _, _, _, positions, _ = read_transforms(path)
        positions = positions / 100.
        support = []
        for side in ('Left', 'Right'):
            ankle = positions[:, names.index(side+'Foot')]
            toe = positions[:, names.index(side+'ToeBase')]
            va = np.linalg.norm(np.gradient(savgol_filter(ankle[:, [0,2]],9,2,axis=0),data['dt'],axis=0),axis=-1)
            vt = np.linalg.norm(np.gradient(savgol_filter(toe[:, [0,2]],9,2,axis=0),data['dt'],axis=0),axis=-1)
            low = np.minimum(ankle[:,1]-np.quantile(ankle[:,1],.05),toe[:,1]-np.quantile(toe[:,1],.05)) < .035
            support.append(low & (np.minimum(va,vt) < .10))
        support = np.stack(support, -1)
        fps = data['fps']
        # Index-base ambiguity is bounded conservatively by a one-second inset.
        begin = int(cuts[kind + '_START']) + fps
        end = int(cuts[kind + '_STOP']) - fps
        if not 0 <= begin < end <= len(data['frames']):
            raise ValueError(f'Official trim outside converted file: {path.name}')
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        rows, rejected = [], Counter()
        for seconds in (3, 6, 12, 20):
            size = seconds * fps
            for start in range(begin, end-size+1, fps):
                stop = start + size
                sl = slice(start, stop)
                speed, rate = data['speed'][sl], data['rate'][sl]
                travel = data['travel_error'][sl]
                foot = data['foot_yaw'][sl]
                support_mask = support[sl]
                # Toe-off can make the ankle-to-toe horizontal projection nearly
                # zero. Assess planted-foot heading, not that singular swing yaw.
                if np.min(support_mask.sum(0)) < .15*size:
                    rejected['insufficient_support_samples'] += 1; continue
                support_yaw = foot[support_mask]
                yaw = data['heading'][stop-1] - data['heading'][start]
                if data['hand_drop'][sl].min() < .12:
                    rejected['raised_hands_or_calibration'] += 1; continue
                if np.rad2deg(np.quantile(data['root_tilt'][sl], .95)) > 15:
                    rejected['root_tilt'] += 1; continue
                if np.quantile(np.abs(support_yaw), .9) > np.deg2rad(30):
                    rejected['toe_out_over_30deg_p90'] += 1; continue
                active = speed > .2
                direction = float(np.arctan2(np.sin(travel[active]).mean(), np.cos(travel[active]).mean())) if active.any() else 0.
                direction_error = float(np.quantile(np.abs(wrap(travel[active]-direction)), .9)) if active.any() else 0.
                first, last = float(speed[:fps].mean()), float(speed[-fps:].mean())
                categories = []
                if kind == 'ID' and seconds == 6 and speed.max() < .12 and abs(rate).max() < .15:
                    categories.append('stand')
                if seconds >= 3 and active.mean() > .9 and .25 < speed.mean() < 2.0 and direction_error < .25:
                    if abs(yaw) < .15 and np.quantile(abs(rate), .9) < .18:
                        if kind == 'FW' and abs(direction) < .25: categories.append('forward')
                        if kind == 'BW' and abs(abs(direction)-np.pi) < .25: categories.append('backward')
                        if kind == 'SW' and abs(abs(direction)-np.pi/2) < .3:
                            categories.append('side_left' if direction > 0 else 'side_right')
                    if kind in ('FW', 'TR1') and .3 < abs(yaw) < 1.4 and abs(direction) < .3 and (rate*np.sign(yaw) > -.05).mean() > .9:
                        categories.append('turn_left' if yaw > 0 else 'turn_right')
                if seconds >= 12 and kind in ('FW','TR1') and .25 < speed.mean() < 1.5 and abs(direction) < .3 and direction_error < .4 and np.quantile(abs(rate),.95) < 1.8:
                    categories.append('natural_path')
                if kind in ('FW', 'TR1') and seconds >= 6 and abs(yaw) < .2 and abs(direction) < .3:
                    if first < .15 and last > .4: categories.append('start')
                    if last < .15 and first > .4: categories.append('stop')
                if not categories: continue
                peaks = []
                for side in range(2):
                    z = data['foot_height'][sl, side]
                    indices, _ = find_peaks(z, distance=round(.25*fps), prominence=.015)
                    peaks.append(z[indices].tolist())
                if 'stand' not in categories and any(len(x) < 3 for x in peaks):
                    rejected['insufficient_bilateral_swings'] += 1; continue
                for category in categories:
                    rows.append(dict(source=path.name,source_sha256=digest,category=category,
                        performer='single_100style_actor',begin=start,end=stop,fps=1/data['dt'],duration_s=seconds,
                        mean_human_speed_m_s=float(speed.mean()),net_yaw_rad=float(yaw),
                        travel_direction_deg=float(np.rad2deg(direction)),direction_error_p90_deg=float(np.rad2deg(direction_error)),
                        foot_heading_p90_deg=float(np.rad2deg(np.quantile(abs(support_yaw),.9))),
                        all_frame_foot_heading_p90_deg=float(np.rad2deg(np.quantile(abs(foot),.9))),
                        mean_foot_heading_deg=[float(np.rad2deg(foot[:,i][support_mask[:,i]].mean())) for i in range(2)],
                        heading_measure='Inferred support only; not measured force contact',
                        swing_peak_landmark_heights_m=peaks,
                        score=float(np.quantile(abs(foot),.9)+direction_error)))
        all_rows.extend(rows)
        sources.append(dict(file=path.name,sha256=digest,frames=len(data['frames']),fps=1/data['dt'],
                            official_trim=[int(cuts[kind+'_START']),int(cuts[kind+'_STOP'])],
                            conservative_trim=[begin,end],candidates=len(rows),rejections=dict(rejected)))
        print(json.dumps(sources[-1]),flush=True)
    selected, counts = [], Counter()
    for row in sorted(all_rows,key=lambda r:(-r['duration_s'],r['score'])):
        if counts[row['category']] >= args.per_category: continue
        if any(x['source']==row['source'] and row['begin'] < x['end']+row['fps'] and x['begin'] < row['end']+row['fps'] for x in selected): continue
        selected.append(row);counts[row['category']] += 1
    for name in sorted({r['source'] for r in selected}):
        hierarchy,motion=(args.source_dir/name).read_text().split('MOTION',1)
        lines=motion.strip().splitlines();dt=float(lines[1].split(':',1)[1])
        for row in [r for r in selected if r['source']==name]:
            crop=f"{Path(name).stem}_{row['begin']:06d}_{row['end']:06d}_{row['category']}.bvh"
            target=args.output/crop
            target.write_text(hierarchy+f"MOTION\nFrames: {row['end']-row['begin']}\nFrame Time: {dt:.17g}\n"+'\n'.join(lines[2+row['begin']:2+row['end']])+'\n')
            row.update(name=target.stem,crop=crop,crop_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    report=dict(accepted_for_training=False,visual_review_passed=False,
                scope='Human-source screening; landmark heights are not robot sole clearance.',
                trim_note='Official frame cuts with one-second inward margins; no frame reordering or time scaling.',
                split_note='One actor only; no cross-subject generalization claim. Hold out disjoint complete clips before training.',
                license='CC-BY-4.0',sources=sources,selected=selected,counts=dict(counts))
    (args.output/'screening.json').write_text(json.dumps(report,indent=2)+'\n')
    (args.output/'source_manifest.json').write_text(json.dumps(selected,indent=2)+'\n')
    print(json.dumps(dict(selected=len(selected),counts=dict(counts))),flush=True)


if __name__ == '__main__':
    main()
