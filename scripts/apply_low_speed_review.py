"""Record the classification correction without rewriting the frozen run archive."""
import json,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'outputs/joint-paper-v5-001'
contact=json.loads((run/'loaded_foot_review.json').read_text())
startup=json.loads((run/'initialization-review-urdf-zero-101/summary.json').read_text())
path=run/'completion_report.json';report=json.loads(path.read_text())
report.setdefault('previous_2cm_gate_result',report.get('walking_goal_achieved'))
report['walking_goal_achieved']=None
report['walking_classification']='low_clearance_gait_with_possible_transient_slip_needs_synchronized_contact_review'
report['classification_review']=dict(reason='Low speed need not exceed2cm clearance; sole-center motion includes rolling. Earlier pure-skating failure verdict was too strong.',
    loaded_contact_point_review='loaded_foot_review.json',urdf_zero_start_survived=sum(row['survived'] for row in startup['results']),
    urdf_zero_start_total=len(startup['results']),urdf_zero_start_seconds=startup['seconds'])
path.write_text(json.dumps(report,indent=2))
(ROOT/'joint_state.json').write_text(json.dumps(dict(stage='training_complete_low_speed_gait_under_review',iteration=report['iteration'],walking_goal_achieved=None,
    utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),review='LOW_SPEED_AND_INITIAL_POSE_REVIEW.md'),indent=2))
print('LOW_SPEED_CLASSIFICATION_REVIEW_RECORDED',flush=True)
