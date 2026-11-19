"""Attribute recorded action interval jitter without hardware access or log mutation."""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'outputs/deploy-20261116'
OUT = BASE / 'action-interval-jitter'
RUNS = ['xbox-powered-20261116-152917-5629', 'xbox-powered-20261116-153129-5687']

def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))

def stats(values):
    a = np.asarray(values)
    return dict(n=len(a), median=float(np.median(a)), p99=float(np.quantile(a, .99)),
                maximum=float(a.max()), minimum=float(a.min()))

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    results = []
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    for col, run in enumerate(RUNS):
        p = BASE / 'live-evidence/supermimic-pm01-xbox30-1116' / run
        actors = {int(r['sequence']): {k: int(v) for k, v in r.items()} for r in read(p/'native.bin.ipc.csv')}
        timing = {int(r['sequence']): r for r in read(p/'can.json.timing.csv')}
        groups = {}
        for r in read(p/'action-motor-latency.csv'):
            groups.setdefault(int(r['sequence']), {})[r['motor']] = r
        rows = []
        for seq in sorted(groups):
            if seq-1 not in groups:
                continue
            assert len(groups[seq]) == len(groups[seq-1]) == 31
            row = dict(sequence=seq, time_s=(seq-1)*.02)
            for side in ['left', 'right']:
                name = side+'_hip_pitch_motor'
                now, prev = groups[seq][name], groups[seq-1][name]
                row[side+'_driver_interval_ms'] = (int(now['driver_mono_upper_ns'])-int(prev['driver_mono_upper_ns']))/1e6
                # Same motor, same CAN adapter clock: no cross-clock subtraction.
                assert int(now['hardware_raw_ns']) > int(prev['hardware_raw_ns']) > 0
                row[side+'_hardware_interval_ms'] = (int(now['hardware_raw_ns'])-int(prev['hardware_raw_ns']))/1e6
            stages = []
            for q in [seq-1, seq]:
                a, c = actors[q], timing[q]
                points = [int(c['publish_ns']), a['receive_ns'], a['infer_start_ns'], a['infer_end_ns'],
                          a['target_ns'], int(groups[q]['left_hip_pitch_motor']['driver_mono_upper_ns'])]
                stages.append(np.diff(points)/1e6)
            names = ['state_ipc', 'preprocess', 'inference', 'postprocess', 'target_to_driver']
            for i, name in enumerate(names):
                row[name+'_previous_ms'] = float(stages[0][i])
                row[name+'_current_ms'] = float(stages[1][i])
                row[name+'_delta_ms'] = float(stages[1][i]-stages[0][i])
            row['publish_interval_ms'] = (int(timing[seq]['publish_ns'])-int(timing[seq-1]['publish_ns']))/1e6
            reconstructed = row['publish_interval_ms'] + sum(row[n+'_delta_ms'] for n in names)
            assert abs(reconstructed-row['left_driver_interval_ms']) < 1e-8
            intervals = [(int(groups[seq][m]['driver_mono_upper_ns'])-int(groups[seq-1][m]['driver_mono_upper_ns']))/1e6 for m in groups[seq]]
            row['all_motor_interval_min_ms'] = min(intervals)
            row['all_motor_interval_max_ms'] = max(intervals)
            rows.append(row)
        with (OUT/(run+'.csv')).open('w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
        peak = max(rows, key=lambda r:r['left_driver_interval_ms'])
        result = dict(run=run, peak=peak, peak_neighbors=[r for r in rows if abs(r['sequence']-peak['sequence'])<=1],
                      intervals={k:stats([r[k] for r in rows]) for k in ['left_driver_interval_ms', 'right_driver_interval_ms', 'left_hardware_interval_ms', 'right_hardware_interval_ms', 'publish_interval_ms']}, segments={})
        geom = np.load(BASE/'walking-diagnosis'/(run+'.npz'))
        for label, lo, hi in [('startup',0,20),('early',22,32),('late',40,50),('stop',50,54.1)]:
            selected = [r for r in rows if lo<=r['time_s']<hi]
            mask = (geom['time']>=lo)&(geom['time']<hi)
            result['segments'][label] = dict(frames=len(selected),
                intervals_over_22_ms=sum(r['left_driver_interval_ms']>22 for r in selected),
                intervals_over_24_ms=sum(r['left_driver_interval_ms']>24 for r in selected),
                inference_ms=stats([r['inference_current_ms'] for r in selected]),
                torso_roll_median_deg=float(np.median(geom['measured'][mask,5])))
        result['left_right_interval_difference_ms'] = stats([r['left_driver_interval_ms']-r['right_driver_interval_ms'] for r in rows])
        results.append(result)
        t = [r['time_s'] for r in rows]
        axes[0,col].plot(t,[r['left_hardware_interval_ms'] for r in rows],lw=.6,label='Left hip: hardware TX interval')
        axes[0,col].axhline(20,color='black',lw=.6); axes[0,col].set_title('Run '+str(col+1))
        axes[1,col].plot(t,[r['inference_current_ms'] for r in rows],lw=.6)
        axes[2,col].plot(geom['time'],geom['measured'][:,5],lw=.7)
        for ax in axes[:,col]:
            ax.axvline(peak['time_s'],color='red',ls='--',lw=.8)
            ax.axvspan(20,50,color='green',alpha=.06); ax.grid(alpha=.2)
        axes[2,col].set_xlabel('Policy time (s)')
    axes[0,0].set_ylabel('Action TX interval (ms)')
    axes[1,0].set_ylabel('Inference (ms)')
    axes[2,0].set_ylabel('FK torso roll (deg)')
    fig.tight_layout(); fig.savefig(OUT/'interval_and_roll.png',dpi=150)
    (OUT/'summary.json').write_text(json.dumps(results,indent=2)+'\n',encoding='utf-8')
    for r in results:
        print(json.dumps(dict(run=r['run'],peak=r['peak'],segments=r['segments']),indent=2))

if __name__ == '__main__':
    main()
