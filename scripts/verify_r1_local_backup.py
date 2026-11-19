"""Verify the received archive and materialize exact validated deployment files."""
import hashlib,json,tarfile
from pathlib import Path

s=Path(__file__).resolve().parents[1]/'outputs/deploy-sm-walk-r1-fast-20261119'
archive=s/'validation-evidence.tar.gz';meta=json.loads((s/'validation-manifest.json').read_text())
h=hashlib.sha256()
with archive.open('rb') as stream:
    for block in iter(lambda:stream.read(8*1024*1024),b''):h.update(block)
assert archive.stat().st_size==meta['bytes'] and h.hexdigest()==meta['sha256']
identities=json.loads((s/'build-manifest.json').read_text())['files']
required={'bundle/policy.onnx','build/sprite_can_shadow','build/sprite_supermimic_protected_ipc',
          'driver-echo-order/kcan.ko','qualified-kernel-log.txt','operator-xbox.sh','load-echo-order-driver.sh'}
for run in ['sleeping-disabled-01','sleeping-disabled-02','sleeping-fault-01']:
    required|={run+'/'+name for name in ['commissioning.json','action-tx-audit.json','can.json','native.bin.affinity.json']}
found=set()
with tarfile.open(archive,'r|gz') as stream:
    for item in stream:
        if item.name not in required:continue
        assert item.isfile()
        data=stream.extractfile(item).read()
        if item.name in identities:assert hashlib.sha256(data).hexdigest()==identities[item.name],item.name
        target=s/'validated'/item.name
        target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
        found.add(item.name)
assert found==required,required-found
result=dict(passed=True,archive=meta,materialized_files=sorted(found))
(s/'local-backup-verification.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=True,bytes=meta['bytes'],sha256=h.hexdigest(),materialized=len(found))))
