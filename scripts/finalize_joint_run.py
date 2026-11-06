"""Validate completed artifacts and distinguish finished training from walking success."""
import json,hashlib,time,tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'outputs/joint-paper-v5-001'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
checkpoint=sorted(run.glob('model_*.pt'))[-1];checkpoint_sha=sha(checkpoint)
summaries={};gaits={}
for seed in [101,202]:
    for interface in ['motion','velocity']:
        folder=run/f'final-{interface}-{seed}'
        summary=json.loads((folder/'summary.json').read_text())
        assert summary['checkpoint_sha256']==checkpoint_sha
        assert (folder/'evaluation.mp4').stat().st_size>0 and (folder/'rollout.npz').stat().st_size>0
        summaries[folder.name]=dict(survived=sum(row['survived'] for row in summary['results']),total=len(summary['results']),seconds=summary['seconds'],results=summary['results'])
        if (folder/'gait_quality.json').exists():gaits[folder.name]=json.loads((folder/'gait_quality.json').read_text())
contract=json.loads((run/'export-final/contract.json').read_text())
assert contract['checkpoint_sha256']==checkpoint_sha and sha(run/'export-final/policy.onnx')==contract['policy_onnx_sha256']
assert contract['parity_max_abs_error']<1e-4
walking=all(gaits.get(f'final-velocity-{seed}',{}).get('moving_command_walking_accepted',False) for seed in [101,202])
last=json.loads((run/'metrics.jsonl').read_text().splitlines()[-1])
report=dict(training_completed=True,walking_goal_achieved=walking,iteration=last['iteration'],training_wall_seconds=last['wall_seconds'],sim_steps=last['sim_steps'],
            checkpoint=str(checkpoint.relative_to(ROOT)),checkpoint_sha256=checkpoint_sha,onnx_parity_max_abs_error=contract['parity_max_abs_error'],
            evaluations=summaries,gait_quality=gaits,hardware_qualified=False,
            wrapper_error='Trailing CRLF blank shell line reported failure after training/evaluations/export finished; all artifact checks above passed',
            corrected_reward_revision='paper_a6_torso_heading_v2',corrected_long_training_started=False)
existing=run/'completion_report.json'
if existing.exists():
    previous=json.loads(existing.read_text())
    if previous.get('classification_review'):
        for key in ['classification_review','walking_classification','previous_2cm_gate_result']:report[key]=previous[key]
        report['walking_goal_achieved']=None
(run/'completion_report.json').write_text(json.dumps(report,indent=2))
(ROOT/'joint_state.json').write_text(json.dumps(dict(stage='training_complete_low_speed_gait_under_review' if report.get('classification_review') else ('training_completed_gait_rejected' if not walking else 'simulation_walking_accepted'),iteration=last['iteration'],
    utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),walking_goal_achieved=report['walking_goal_achieved']),indent=2))
for interface in ['motion','velocity']:
    path=run/f'best_{interface}.json';meta=json.loads(path.read_text());meta.update(walking_qualified=False,selection_basis='original survival/task error only; not gait-qualified')
    path.write_text(json.dumps(meta,indent=2))
print(json.dumps({k:report[k] for k in ['training_completed','walking_goal_achieved','iteration','training_wall_seconds','checkpoint_sha256']}),flush=True)
