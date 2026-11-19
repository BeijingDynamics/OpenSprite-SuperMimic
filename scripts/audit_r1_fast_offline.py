"""Check recorded direct-drive state limits and isolate the requested source changes."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'outputs/deploy-sm-walk-r1-xbox120-20261119'
NEW=ROOT/'outputs/deploy-sm-walk-r1-fast-20261119'
changed=[]
for path in (OLD/'source').rglob('*'):
    other=NEW/'source'/path.relative_to(OLD/'source')
    if path.is_file() and other.is_file():
        if path.read_bytes().replace(b'\r\n',b'\n')!=other.read_bytes().replace(b'\r\n',b'\n'):
            changed.append(path.relative_to(OLD/'source').as_posix())
expected={'native/src/sprite_can_shadow.cpp','native/src/sprite_supermimic_replay.cpp',
          'native/tests/test_bounded_preview.cpp','native/tests/test_gamepad_control.cpp',
          'tools/run_supermimic_commissioning.py','operator-xbox.sh','run-disabled.py'}
assert set(changed)==expected,changed
old_actor=(OLD/'source/native/src/sprite_supermimic_replay.cpp').read_text()
new_actor=(NEW/'source/native/src/sprite_supermimic_replay.cpp').read_text()
assert new_actor.replace('\n      gamepad_config.acceleration=.10;','')==old_actor
runs=[]
for run in sorted((OLD/'powered-through-144201/powered').glob('*')):
    if not (run/'native.bin.inputs').exists():continue
    x=np.fromfile(run/'native.bin.inputs',dtype='<f8').reshape(-1,71)
    with (run/'motors.tsv').open() as f:motors=list(csv.DictReader(f,delimiter='\t'))
    with (run/'kinematics.tsv').open() as f:kin=list(csv.DictReader(f,delimiter='\t'))[:31]
    assert [r['name'] for r in kin]==[r['motor_name'] for r in motors]
    M=np.array([[float(r[f'c{i}']) for i in range(31)] for r in kin])
    offset=np.array([float(r['offset_or_effort']) for r in kin])
    q=x[:,:31]@M.T+offset;qd=x[:,31:62]@M.T
    violations=[]
    for i,m in enumerate(motors):
        if 'ankle' in m['motor_name']:continue
        bad=(q[:,i]<float(m['hard_position_min_rad']))|(q[:,i]>float(m['hard_position_max_rad']))|(abs(qd[:,i])>float(m['deployment_velocity_max_rad_s']))
        if np.any(bad):violations.append(dict(motor=m['motor_name'],frames=int(bad.sum())))
    report=json.loads((run/'can.json').read_text())
    knee=report['motors']['right_knee_motor']
    runs.append(dict(run=run.name,frames=len(x),direct_drive_position_speed_violations=violations,
                     knee_max_feedback_nm=knee['max_feedback_torque_nm'],
                     knee_feedback_observation_boundary_nm=knee['feedback_torque_cap_nm'],
                     final_disabled_count=sum(m['final_disabled'] for m in report['motors'].values())))
out=dict(changed_files=changed,actor_change_is_only_command_acceleration=True,runs=runs,
         pd_and_model_change=False,requested_forward_acceleration_m_s2=.10,
         limitations=['Recorded policy states exclude CAN feedback rejected before publication.',
                      'Raw feedback torque is not present in policy input histories; whole-session torque maxima are not a packet-level replay.',
                      'Parallel ankle geometry is excluded from this linear direct-drive state check.',
                      'Existing motions do not predict closed-loop behavior with a different command ramp.',
                      'Native socket tests independently exercise full-scale torque and retained stop conditions in the production decoder.'])
(NEW/'offline-audit.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
