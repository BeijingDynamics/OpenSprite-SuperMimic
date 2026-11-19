"""Build the isolated 120-second Xbox profile; no device traffic is emitted."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile

stage=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
old=Path('/home/tony/sprite-sm-walk-r1-xbox120-20261119')
stage.mkdir(exist_ok=False)
with tarfile.open('/home/tony/r1-fast-source.tar.gz') as archive:
    for item in archive.getmembers():
        if not (stage/item.name).resolve().is_relative_to(stage):raise ValueError('Invalid archive path')
    archive.extractall(stage)
shutil.copytree(old/'bundle',stage/'bundle')
(stage/'cpu-replay').mkdir()
shutil.copyfile(old/'cpu-replay/report.json',stage/'cpu-replay/report.json')
(stage/'cpu-replay/reuse.json').write_text(json.dumps(dict(source=str(old/'cpu-replay'),
    rationale='Policy model and native observation/inference core unchanged; new command schedule is separately tested.',
    policy_sha256=hashlib.sha256((stage/'bundle/policy.onnx').read_bytes()).hexdigest()),indent=2)+'\n')
lib='/home/tony/open_sprite_runtime/.venv/lib/python3.10/site-packages/onnxruntime/capi/libonnxruntime.so.1.23.2'
subprocess.run(['cmake','-S',str(stage/'native'),'-B',str(stage/'build'),
    '-DSPRITE_ORT_INCLUDE=/home/tony/open_sprite_runtime/build/policy_native_stage2/include','-DSPRITE_ORT_LIBRARY='+lib],check=True)
(stage/'build/libonnxruntime.so.1').symlink_to(lib)
subprocess.run(['cmake','--build',str(stage/'build'),'-j','2'],check=True)
subprocess.run(['ctest','--test-dir',str(stage/'build'),'--output-on-failure'],check=True)
subprocess.run(['bash','-n',str(stage/'operator-xbox.sh')],check=True)
subprocess.run(['bash','-n',str(stage/'ensure-can-scheduler.sh')],check=True)
subprocess.run(['/home/tony/open_sprite_runtime/.venv/bin/python','-m','py_compile',
    str(stage/'tools/run_supermimic_commissioning.py'),str(stage/'run-disabled.py')],check=True)
files=[stage/'build/sprite_can_shadow',stage/'build/sprite_supermimic_protected_ipc',stage/'bundle/policy.onnx',
       stage/'tools/run_supermimic_commissioning.py',stage/'native/src/sling_window.hpp',stage/'native/src/policy_gamepad.hpp']
(stage/'build-manifest.json').write_text(json.dumps(dict(hardware_tx=0,motor_enable_attempts=0,
    requested_window_s=120,total_duration_s=146,forward_cap_m_s=.40,forward_acceleration_m_s2=.10,feedback_torque_mode='observe_only',
    files={str(p.relative_to(stage)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}),indent=2)+'\n')
print('BUILD_AND_TESTS_PASSED. Realtime capability must be restored before disabled integration.',flush=True)
