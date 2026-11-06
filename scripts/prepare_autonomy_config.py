import json,time
from datetime import datetime
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
window=json.loads((ROOT/'autonomy_window.json').read_text())
deadline=datetime.fromisoformat(window['authorized_end_utc']).timestamp()
remaining=int(deadline-time.time()-window['final_evaluation_reserve_seconds'])
if remaining<60:raise RuntimeError('Autonomous training window has ended; evaluate and archive instead')
config=json.loads((ROOT/'configs/joint-paper.json').read_text())
config.update(max_wall_seconds=remaining,autonomous_deadline_utc=window['authorized_end_utc'])
(ROOT/'configs/joint-14h-active.json').write_text(json.dumps(config,indent=2))
print('AUTONOMOUS_TRAINING_BUDGET_SECONDS',remaining,flush=True)
