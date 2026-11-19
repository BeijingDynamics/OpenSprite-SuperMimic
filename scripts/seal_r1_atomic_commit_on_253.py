"""Preserve the failed timing run and seal the corrected coherent-action build."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

s=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
for suffix in ['', '.log', '-job.json']:
    a=s/('fixed-disabled-01'+suffix);b=s/('failed-isolated-dispatch-01'+suffix)
    if b.exists():raise FileExistsError(b)
    a.rename(b)
shutil.copyfile(s/'build-manifest.json',s/'build-manifest-before-atomic-commit.json')
m=json.loads((s/'build-manifest.json').read_text())
m['coherent_mapping']='refresh_all_on_commit; commit_in_current_transport_slot'
m['can_optimization']='-O2 -ffp-contract=off; no fast-math'
for p in ['build/sprite_can_shadow','build/sprite_supermimic_protected_ipc','bundle/policy.onnx',
          'native/src/sprite_can_shadow.cpp','native/src/action_tx_trace.hpp','native/CMakeLists.txt',
          'tools/run_supermimic_commissioning.py','operator-xbox.sh']:
    m['files'][p]=hashlib.sha256((s/p).read_bytes()).hexdigest()
(s/'build-manifest.json').write_text(json.dumps(m,indent=2)+'\n')
subprocess.run(['ctest','--test-dir',str(s/'build'),'--output-on-failure'],check=True)
print('Coherent build identity sealed. No hardware access performed.')
