"""Archive completed extended R1 operator trials without changing hardware."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

stage = Path('/home/tony/sprite-sm-walk-r1-xbox120-20261119')
active = subprocess.check_output(['ps', '-eo', 'comm='], text=True).splitlines()
if any(n.strip().startswith(('sprite_can', 'sprite_super', 'sprite_icm')) for n in active):
    raise RuntimeError('Device processes are active; postpone collection')
archive = stage / 'powered-through-144201.tar.gz'
if archive.exists():
    raise FileExistsError(archive)
runs = sorted(stage.glob('xbox-powered-*'))
with tarfile.open(archive, 'w:gz', compresslevel=1) as stream:
    for run in runs:
        if not (run / 'commissioning.json').exists():
            raise RuntimeError('Incomplete session: ' + str(run))
        stream.add(run, arcname='powered/' + run.name)
    for path in (stage / 'bundle').glob('*.json'):
        stream.add(path, arcname='bundle/' + path.name)
meta = dict(bytes=archive.stat().st_size, sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),
            runs=[r.name for r in runs])
(stage / 'powered-through-144201-manifest.json').write_text(json.dumps(meta, indent=2) + '\n')
print(json.dumps(meta))
