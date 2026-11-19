"""Record final build identities after selecting ORT no-idle-spinning mode."""
import hashlib,json,shutil
from pathlib import Path
s=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
backup=s/'build-manifest-before-inference-modes.json'
if backup.exists():raise FileExistsError(backup)
shutil.copyfile(s/'build-manifest.json',backup)
m=json.loads((s/'build-manifest.json').read_text())
m['inference_mode']=dict(threads=2,idle_spinning=False,main_cpu=4,worker_cpu=5,io_cpu=1)
for name in ['build/sprite_can_shadow','build/sprite_supermimic_protected_ipc','build/sprite_supermimic_replay',
             'native/src/inference_cpu_layout.hpp','native/src/sprite_supermimic_replay.cpp',
             'tools/run_supermimic_commissioning.py','tools/can_tx_timestamp_setup.py',
             'operator-xbox.sh','run-disabled.py','load-echo-order-driver.sh']:
    m['files'][name]=hashlib.sha256((s/name).read_bytes()).hexdigest()
assert Path('/sys/module/kcan/srcversion').read_text().strip()==m['driver_srcversion']
(s/'build-manifest.json').write_text(json.dumps(m,indent=2)+'\n')
print('Final no-spinning operator identity recorded; no hardware access.')
