"""Pin evaluation source and hash completed speed-test artifacts for local backup."""
import hashlib,json,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];folder=ROOT/'outputs/speed-envelope-001'
source=folder/'evaluation-source';source.mkdir(exist_ok=True)
for name in ['evaluate_joint.py','render_velocity.py','run_velocity_envelope.py','summarize_velocity_envelope.py','analyze_joint_gait.py']:
    shutil.copy2(ROOT/'scripts'/name,source/name)
manifest={str(p.relative_to(folder)):dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest())
          for p in sorted(folder.rglob('*')) if p.is_file() and p.name!='SHA256SUMS.json'}
(folder/'SHA256SUMS.json').write_text(json.dumps(manifest,indent=2))
print(json.dumps(dict(files=len(manifest),bytes=sum(row['bytes'] for row in manifest.values()))),flush=True)
