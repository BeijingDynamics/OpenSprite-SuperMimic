"""Compact progress snapshot; no credentials or large training logs."""
import csv,json,time
from pathlib import Path
root=Path(__file__).resolve().parents[1]
result={'utc':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}
state=root/'experiment_state.json'
if state.exists():result['experiment']=json.loads(state.read_text())
run=root/'outputs/v5-velocity-first-8h'
path=run/'metrics.csv'
if path.exists():
    with path.open() as f:rows=list(csv.DictReader(f))
    if rows:
        keys=['iteration','time/wall_hours','time/iterations_per_second','real/episode_length_steps','real/reward','real/terminated_per_step','model/loss','imagination/actor_loss','memory/peak_gib']
        result['latest']={k:rows[-1].get(k) for k in keys}
checkpoints=sorted(run.glob('model_*.pt'))
if checkpoints:result['latest_checkpoint']=checkpoints[-1].name
print(json.dumps(result,indent=2))
