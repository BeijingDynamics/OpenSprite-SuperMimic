"""Freeze the exact run001 implementation before applying corrective changes."""
import hashlib,json,tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'outputs/joint-paper-v5-001'
output=run/'run001-source-snapshot.tar.gz'
if output.exists():raise SystemExit('Source snapshot already exists; never overwrite')
paths=[]
for folder in ['opensprite_tracker','scripts','configs']:
    paths.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in {'.py','.json','.sh'})
paths.extend(ROOT/name for name in ['source_provenance.json','JOINT_EXPERIMENT.md','AGENTS.md','run_joint_experiment.sh','autonomy_window.json'])
manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
with tarfile.open(output,'w:gz') as archive:
    for p in paths:archive.add(p,arcname=str(p.relative_to(ROOT)))
(run/'run001-source-manifest.json').write_text(json.dumps(manifest,indent=2))
print(output,flush=True)
