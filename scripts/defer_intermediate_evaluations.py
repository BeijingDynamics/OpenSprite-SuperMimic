"""Honor the new finish-training-first instruction without touching training."""
from pathlib import Path
import os,signal,json,datetime,time
ROOT=Path(__file__).resolve().parents[1]
state=json.loads((ROOT/'run002_state.json').read_text())
training=int(state['training_pid']);watcher=int(state['watcher_pid'])
train_cmd=Path(f'/proc/{training}/cmdline').read_bytes().replace(b'\0',b' ').decode()
assert 'scripts/train_joint.py' in train_cmd and 'joint-waist-v5-002' in train_cmd
if Path(f'/proc/{watcher}/cmdline').exists():
    command=Path(f'/proc/{watcher}/cmdline').read_bytes().replace(b'\0',b' ').decode()
    assert 'scripts/watch_run002.py' in command and f'--training-pid {training}' in command
    assert os.getpgid(watcher)==watcher and os.getpgid(training)!=watcher
    os.killpg(watcher,signal.SIGTERM)
record=dict(changed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    user_instruction='Finish the existing12hour training phase before further evaluations; offline runtime preparation thereafter. Jetson253 stays off.',
    training_pid=training,training_unchanged=True,stopped_intermediate_watcher_pid=watcher,
    next_evaluation='after_training_completion',hardware_qualified=False)
(ROOT/'evaluation_schedule_override.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record),flush=True)
