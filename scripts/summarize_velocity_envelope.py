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
text=['# 当前v5策略速度命令测试','',f'{len(rows)}组命令，种子{result["seeds"]}，每组30秒。全部从URDF零关节姿态起步，不重置失败。',
      '存活与跟速精度分别统计；下表实测为最后5秒的存活试验平均。范围内/外指训练随机命令范围，参考命令可能越界。','',
      '|类别|命令vx/vy/yaw|30秒未倒|实测vx/vy/yaw|随机命令范围|',
      '|---|---|---|---|---|']
fmt=lambda value:'—' if value is None else f'{value:+.3f}'
for r in rows:
    text.append(f'|{r["category"]}|{r["vx"]:+.2f}/{r["vy"]:+.2f}/{r["yaw"]:+.2f}|{r["survived"]}/{r["trials"]}|'+
                '/'.join(fmt(r[k]) for k in ['measured_vx','measured_vy','measured_yaw'])+f'|{"内" if r["inside_random_training_range"] else "外"}|')
text+=['','## 同一个模型连续切换命令','',
       '每个面板为同一机器人连续运行，每5秒切换命令，中途不重置、不换权重。录像白色CMD为目标，绿色MEAS为实际速度。',
       '低抬脚不直接判失败；本测试确认命令响应与30秒存活，不能据此认定接触质量或硬件资格。']
for sequence in result.get('live_command_sequences',[]):
    text.append(f'- {sequence["case_name"]}：32秒未倒={sequence["survived"]}；具体每段实测见JSON。')
(folder/'SPEED_ENVELOPE_REPORT.md').write_text('\n'.join(text)+'\n')
print(json.dumps(dict(seeds=result['seeds'],cases=len(rows),survived=sum(r['survived'] for r in rows),trials=sum(r['trials'] for r in rows),
    forward=[r for r in rows if r['category']=='forward']),indent=2),flush=True)
