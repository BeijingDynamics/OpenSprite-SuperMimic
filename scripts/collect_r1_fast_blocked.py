"""Preserve unsuccessful qualification without treating it as operator approval."""
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

stage=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
active=[n for n in subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
        if n.strip().startswith(('sprite_can','sprite_super','sprite_icm','probe'))]
if active:raise RuntimeError('Device or probe process active: '+str(active))
runs=[]
for name in ['failed-send-tail-01','fixed-disabled-01']:
    report=json.loads((stage/name/'commissioning.json').read_text())
    can=json.loads((stage/name/'can.json').read_text())
    assert report['motor_enable_attempts']==0 and report['final_disabled_count']==31
    assert report['cpu_frequency']['released']['status']=='restored'
    runs.append(dict(name=name,passed=report['passed'],failure=report['failure'],
                     native_errors=can['errors'],native_deadline_stage=can['deadline_stage'],
                     final_disabled_count=31,motor_enable_attempts=0))
probe=json.loads((stage/'affinity-probe/report.json').read_text())
assert probe['released']['status']=='restored'
summary=dict(qualification_passed=False,powered_script_released=False,runs=runs,
             cpu_only_probe=probe,active_control_processes=active,
             reason='Live timing guards/audits failed. Functional source changes are implemented; qualification is incomplete.')
(stage/'blocked-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
archive=stage/'blocked-evidence.tar.gz'
if archive.exists():raise FileExistsError(archive)
members=['blocked-summary.json','build-manifest.json','operator-xbox.sh','run-disabled.py',
         'failed-send-tail-01','failed-send-tail-01.log','failed-send-tail-01-job.json',
         'fixed-disabled-01','fixed-disabled-01.log','fixed-disabled-01-job.json','affinity-probe',
         'bundle/bundle.json','bundle/contract.json','bundle/training_control_profile.json']
with tarfile.open(archive,'w:gz',compresslevel=1) as stream:
    for name in members:stream.add(stage/name,arcname=name)
meta=dict(bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
(stage/'blocked-manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps(dict(summary=summary,archive=meta),indent=2))
