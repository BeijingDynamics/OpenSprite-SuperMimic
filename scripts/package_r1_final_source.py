"""Preserve the exact deployed source candidate independently of older archives."""
import hashlib
import json
import argparse
from pathlib import Path
import tarfile

root=Path(__file__).resolve().parents[1]
dest=root/'outputs/deploy-sm-walk-r1-fast-20261119'
source=dest/'source'
parser=argparse.ArgumentParser();parser.add_argument('--name',default='source-atomic-commit')
name=parser.parse_args().name
if not name.replace('-','').isalnum():raise ValueError('Invalid archive name')
archive=dest/(name+'.tar.gz')
if archive.exists():raise FileExistsError(archive)
with tarfile.open(archive,'w:gz') as out:
    for path in sorted(source.rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            out.add(path,arcname=path.relative_to(source).as_posix())
files={p.relative_to(source).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
       for p in source.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
meta=dict(archive=archive.name,bytes=archive.stat().st_size,
          sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),files=files)
(dest/(name+'-manifest.json')).write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps({k:v for k,v in meta.items() if k!='files'}))
