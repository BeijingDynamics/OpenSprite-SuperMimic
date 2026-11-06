"""Create a bounded, checksummed bundle of important results, not public caches."""
import hashlib,json,tarfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];run=ROOT/'outputs/joint-paper-v5-001';files=set()
for p in run.iterdir():
    if p.is_file() and (p.suffix in {'.json','.jsonl','.gz'} or p.name in {'model_026951.pt','best_motion.pt','best_velocity.pt'}):files.add(p)
for pattern in ['final-*/*','export-final/*','audit-*/motion/summary.json','audit-*/velocity/summary.json']:
    files.update(p for p in run.glob(pattern) if p.is_file())
for folder in ['data/motions/contact_v5_v11','data/motions/contact_v5_v11_slow2']:
    files.update(p for p in (ROOT/folder).rglob('*') if p.is_file())
for pattern in ['data/sources/lafan1-resolved/v5_*.bvh','data/sources/lafan1-resolved/*manifest.json']:
    files.update(p for p in ROOT.glob(pattern) if p.is_file())
for name in ['RUN001_RESULT.md','JOINT_EXPERIMENT.md','source_provenance.json','joint_state.json','autonomy_window.json','AGENTS.md']:
    files.add(ROOT/name)
for folder in ['opensprite_tracker','scripts','configs']:
    files.update(p for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in {'.py','.json','.sh'})
for name in ['opensprite.xml','retarget_scene.xml']:
    files.add(ROOT/'assets/opensprite'/name)
files.add(ROOT/'run_joint_experiment.sh')
manifest={str(p.relative_to(ROOT)):dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in sorted(files)}
output=ROOT/'final_artifacts';output.mkdir(exist_ok=True)
manifest_path=output/'run001-important.manifest.json';manifest_path.write_text(json.dumps(manifest,indent=2))
target=output/'run001-important.tar'
with tarfile.open(target,'w') as archive:
    for p in sorted(files):archive.add(p,arcname=str(p.relative_to(ROOT)))
    archive.add(manifest_path,arcname=manifest_path.name)
digest=hashlib.sha256(target.read_bytes()).hexdigest();target.with_suffix('.tar.sha256').write_text(digest+'  '+target.name+'\n')
print(json.dumps(dict(archive=str(target),files=len(files),bytes=target.stat().st_size,sha256=digest)),flush=True)
