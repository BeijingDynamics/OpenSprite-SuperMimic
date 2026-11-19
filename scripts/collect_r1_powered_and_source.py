"""Collect completed operator evidence and the exact mixed-rate native source."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

stage=Path('/home/tony/sprite-sm-walk-r1-20261119')
names=subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
if any(n.strip().startswith(('sprite_can','sprite_super','sprite_icm')) for n in names):
    raise RuntimeError('A control session is active; postpone archive creation')
archive=stage/'powered-source-evidence.tar.gz'
if archive.exists():raise FileExistsError(archive)
with tarfile.open(archive,'w:gz',compresslevel=1) as t:
    for p in sorted(stage.glob('xbox-powered-*')):t.add(p,arcname='powered/'+p.name)
    t.add('/home/tony/supermimic-pm01-xbox30-1116/native',arcname='native',filter=lambda i:None if '__pycache__' in i.name else i)
    for name in ['operator-xbox.sh','run-disabled.py','ensure-can-scheduler.sh','tools','runtime','bundle']:
        if name=='bundle':
            for p in (stage/name).glob('*.json'):t.add(p,arcname=name+'/'+p.name)
        else:t.add(stage/name,arcname=name,filter=lambda i:None if '__pycache__' in i.name else i)
meta=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
(stage/'powered-source-evidence-manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps(meta))
