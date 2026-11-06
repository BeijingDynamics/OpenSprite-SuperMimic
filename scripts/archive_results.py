"""One SSH stream per backup, with content-addressed incremental tar payload."""
import sys,json,io,tarfile,hashlib,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
entries=json.loads(subprocess.check_output([sys.executable,str(ROOT/'scripts/collect_run002_results.py')]))
known=json.load(sys.stdin)
with tarfile.open(fileobj=sys.stdout.buffer,mode='w|gz') as archive:
    actual=[]
    for entry in entries:
        p=Path(entry['remote'])
        try:blob=p.read_bytes()
        except FileNotFoundError:continue
        entry['sha256']=hashlib.sha256(blob).hexdigest();entry['size']=len(blob)
        actual.append(entry)
        if known.get(entry['local'])==entry['sha256']:continue
        info=tarfile.TarInfo(entry['local']);info.size=len(blob)
        archive.addfile(info,io.BytesIO(blob))
    blob=json.dumps(actual,indent=2).encode()
    info=tarfile.TarInfo('remote-manifest.json');info.size=len(blob)
    archive.addfile(info,io.BytesIO(blob))
