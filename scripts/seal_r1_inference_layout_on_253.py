"""Seal current build identity and preserve prior failures before final validation."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

stage=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
for suffix in ['', '.log', '-job.json']:
    a=stage/('fixed-disabled-01'+suffix);b=stage/('failed-deadline-01'+suffix)
    if b.exists():raise FileExistsError(b)
    a.rename(b)
subprocess.run(['ctest','--test-dir',str(stage/'build'),'--output-on-failure'],check=True)
subprocess.run(['bash','-n',str(stage/'operator-xbox.sh')],check=True)
subprocess.run(['python3','-m','py_compile',str(stage/'tools/run_supermimic_commissioning.py')],check=True)
subprocess.run([str(stage/'build/sprite_can_shadow'),'--check-commissioning-scheduler','2'],check=True)
shutil.copyfile(stage/'build-manifest.json',stage/'build-manifest-before-thread-isolation.json')
old=json.loads((stage/'build-manifest.json').read_text())
old['inference_layout']=dict(main_cpu=4,ort_worker_cpu=5,io_cpu=1,startup_verified=True)
files=['build/sprite_can_shadow','build/sprite_supermimic_protected_ipc','bundle/policy.onnx',
       'tools/run_supermimic_commissioning.py','native/src/inference_cpu_layout.hpp',
       'native/src/inference_diagnostics.hpp','operator-xbox.sh']
old['files']={p:hashlib.sha256((stage/p).read_bytes()).hexdigest() for p in files}
assert old['files']['build/sprite_can_shadow']=='c7ce58b948196779d8dcf03bbae71174e081530f955a1758ac4011d82cc1aabd'
(stage/'build-manifest.json').write_text(json.dumps(old,indent=2)+'\n')
print('Final identity sealed. CAN binary and realtime capability unchanged.')
