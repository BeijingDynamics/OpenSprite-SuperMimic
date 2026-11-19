"""Verify the five-minute local archive and extract its exact deployable files."""
import hashlib
import json
from pathlib import Path
import tarfile

stage = Path(__file__).resolve().parents[1]/'outputs/deploy-sm-walk-r1-xbox300-20261119'
archive = stage/'validation-evidence.tar.gz'
meta = json.loads((stage/'validation-manifest.json').read_text())
digest = hashlib.sha256()
with archive.open('rb') as stream:
    for block in iter(lambda: stream.read(8*1024*1024), b''):
        digest.update(block)
assert archive.stat().st_size == meta['bytes'] and digest.hexdigest() == meta['sha256']
identities = json.loads((stage/'build-manifest.json').read_text())['files']
required = {'bundle/policy.onnx', 'build/sprite_can_shadow', 'build/sprite_supermimic_protected_ipc',
            'operator-xbox.sh', 'deployment-summary.json', 'qualified-kernel-log.txt',
            'interrupt-02/interrupt-verification.json', 'fixture-capacity-check/report.json',
            'previous-120s-powered/xbox-powered-20261119-165413-12501/commissioning.json'}
for run in ['disabled-04', 'fault-02']:
    required |= {run+'/'+name for name in ['commissioning.json', 'can.json', 'action-tx-audit.json']}
found = set()
with tarfile.open(archive, 'r|gz') as stream:
    for item in stream:
        if item.name not in required:
            continue
        assert item.isfile()
        data = stream.extractfile(item).read()
        if item.name in identities:
            assert hashlib.sha256(data).hexdigest() == identities[item.name]
        target = stage/'validated'/item.name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        found.add(item.name)
assert found == required, required-found
result = dict(passed=True, archive=meta, extracted=sorted(found))
(stage/'local-backup-verification.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(dict(passed=True, bytes=meta['bytes'], extracted_files=len(found))))
