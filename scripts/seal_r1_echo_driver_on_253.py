"""Preserve pre-driver-fix trials and verify the new module before qualification."""
import hashlib
import json
from pathlib import Path
import subprocess

s=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
expected='404E8D65A6EFF60079EAEC1'
assert Path('/sys/module/kcan/srcversion').read_text().strip()==expected
for old,new in [('fixed-disabled-01','passed-atomic-previous-driver-01'),
                ('fixed-disabled-02','passed-atomic-previous-driver-02'),
                ('fixed-fault-01','failed-echo-race-01')]:
    for suffix in ['', '.log', '-job.json']:
        a=s/(old+suffix);b=s/(new+suffix)
        if b.exists():raise FileExistsError(b)
        a.rename(b)
log=subprocess.check_output(['journalctl','-k','--since','2026-11-19 16:08:50','--until','2026-11-19 16:09:05','--no-pager'],text=True)
(s/'echo-race-kernel-evidence.txt').write_text(log)
m=json.loads((s/'build-manifest.json').read_text())
m['driver_srcversion']=expected
m['files']['driver-echo-order/kcan.ko']=hashlib.sha256((s/'driver-echo-order/kcan.ko').read_bytes()).hexdigest()
(s/'build-manifest.json').write_text(json.dumps(m,indent=2)+'\n')
subprocess.run([str(s/'build/sprite_can_shadow'),'--check-commissioning-scheduler','2'],check=True)
print('Corrected timestamp driver verified; preceding trial evidence preserved.')
