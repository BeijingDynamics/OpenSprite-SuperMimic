"""Summarize survival separately from measured command accuracy."""
import csv,json,statistics
import numpy as np
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];folder=ROOT/'outputs/speed-envelope-001'
reports=[json.loads(path.read_text()) for path in sorted(folder.glob('seed-*/summary.json'))]
if not reports:raise SystemExit('No completed trials')
group={}
for report in reports:
    for row in report['results']:
        ranges=report['training_random_command_ranges'];cmd=row['label']
        row['within_training_random_command_range']=all(lo-1e-6<=value<=hi+1e-6 for value,(lo,hi) in zip(cmd,ranges))
        group.setdefault(row['case_name'],[]).append(row)
    path=folder/f'seed-{report["seed"]}'/'summary.json';path.write_text(json.dumps(report,indent=2))
rows=[]
for name,trials in group.items():
    successful=[r for r in trials if r['survived']]
    mean=lambda key:statistics.mean(r[key] for r in successful) if successful else None
    vector=lambda i:statistics.mean(r['last5s_mean_velocity_m_s'][i] for r in successful) if successful else None
    row=dict(name=name,category=trials[0]['command_category'],vx=trials[0]['label'][0],vy=trials[0]['label'][1],yaw=trials[0]['label'][2],
        inside_random_training_range=trials[0]['within_training_random_command_range'],survived=len(successful),trials=len(trials),
        measured_vx=vector(0),measured_vy=vector(1),measured_yaw=mean('last5s_mean_yaw_rad_s'),
        velocity_error=mean('last5s_velocity_error_m_s'),yaw_error=mean('last5s_yaw_error_rad_s'),
        first_failure_seconds=min([r['first_failure_seconds'] for r in trials if r['first_failure_seconds'] is not None],default=None))
    rows.append(row)
result=dict(checkpoint_sha256=reports[0]['checkpoint_sha256'],seeds=[r['seed'] for r in reports],cases=len(rows),seconds=30,
    initialization='URDF-zero joints, identity base orientation, ground clearance1mm; first2seconds zero command; no failure resets',
    training_random_command_ranges=reports[0]['training_random_command_ranges'],
    accuracy_window='last5seconds of surviving trials; velocity errors are mean instantaneous norm, not only error of average',
    outside_range_note='Outside random-command ranges; reference-derived training commands may extend farther, so this is not proof the command was unseen',results=rows)
live=folder/'live-command-changes'
if (live/'summary.json').exists():
    live_summary=json.loads((live/'summary.json').read_text())
    with np.load(live/'rollout.npz',allow_pickle=False) as archive:trace={k:archive[k] for k in ['velocity','yaw_rate','failed']}
    sequences=[]
    for env,case in enumerate(live_summary['results']):
        stage_results=[]
        for i,stage in enumerate(case['stages']):
            end=case['stages'][i+1]['at_seconds'] if i+1<len(case['stages']) else live_summary['seconds']
            begin=round((end-2)*50);stop=round(end*50)
            stage_results.append(dict(at_seconds=stage['at_seconds'],command=stage['command'],last2s_measured_velocity=trace['velocity'][begin:stop,env].mean(0).tolist(),
                last2s_measured_yaw=float(trace['yaw_rate'][begin:stop,env].mean()),failed=bool(trace['failed'][:stop,env].any())))
        sequences.append(dict(case_name=case['case_name'],survived=case['survived'],stages=stage_results))
    result['live_command_sequences']=sequences
(folder/'envelope_summary.json').write_text(json.dumps(result,indent=2))
with (folder/'envelope_summary.csv').open('w',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
text=['# Current v5 policy velocity-command tests','',f'{len(rows)} command sets, seeds {result["seeds"]}, 30 seconds each. All start from the URDF zero-joint pose; failures are not reset.',
      'Survival and tracking accuracy are reported separately. Measurements below average the final 5 seconds of surviving trials. In/out denotes the random training-command range; reference-derived commands may exceed it.','',
      '|Category|Command vx/vy/yaw|Survived 30 s|Measured vx/vy/yaw|Random training range|',
      '|---|---|---|---|---|']
fmt=lambda value:'—' if value is None else f'{value:+.3f}'
for r in rows:
    text.append(f'|{r["category"]}|{r["vx"]:+.2f}/{r["vy"]:+.2f}/{r["yaw"]:+.2f}|{r["survived"]}/{r["trials"]}|'+
                '/'.join(fmt(r[k]) for k in ['measured_vx','measured_vy','measured_yaw'])+f'|{"In" if r["inside_random_training_range"] else "Out"}|')
text+=['','## Continuous command switching with one model','',
       'Each panel follows one robot continuously, switching commands every 5 seconds without resetting or changing weights. White CMD is the target; green MEAS is measured velocity.',
       'Low foot clearance alone is not classified as failure. These tests assess command response and 30-second survival, not contact quality or hardware qualification.']
for sequence in result.get('live_command_sequences',[]):
    text.append(f'- {sequence["case_name"]}: survived 32 seconds={sequence["survived"]}; see JSON for per-segment measurements.')
(folder/'SPEED_ENVELOPE_REPORT.md').write_text('\n'.join(text)+'\n')
print(json.dumps(dict(seeds=result['seeds'],cases=len(rows),survived=sum(r['survived'] for r in rows),trials=sum(r['trials'] for r in rows),
    forward=[r for r in rows if r['category']=='forward']),indent=2),flush=True)
