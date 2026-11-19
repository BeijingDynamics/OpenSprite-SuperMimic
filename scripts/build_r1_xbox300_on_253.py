"""Build and test the isolated five-minute profile without opening robot devices."""
import hashlib,json,shutil,subprocess,tarfile
from pathlib import Path
s=Path('/home/tony/sprite-sm-walk-r1-xbox300-20261119')
old=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
s.mkdir(exist_ok=False)
with tarfile.open('/home/tony/r1-xbox300-source.tar.gz') as archive:
    for member in archive.getmembers():
        if not (s/member.name).resolve().is_relative_to(s):raise ValueError(member.name)
    archive.extractall(s)
shutil.copytree(old/'bundle',s/'bundle')
shutil.copytree(old/'cpu-replay',s/'cpu-replay')
(s/'cpu-replay/window-extension.json').write_text(json.dumps(dict(source=str(old/'cpu-replay'),
    rationale='Same model and observation/action math; extended Xbox window is covered by native tests.'),indent=2)+'\n')
lib='/home/tony/open_sprite_runtime/.venv/lib/python3.10/site-packages/onnxruntime/capi/libonnxruntime.so.1.23.2'
subprocess.run(['cmake','-S',str(s/'native'),'-B',str(s/'build'),
    '-DSPRITE_ORT_INCLUDE=/home/tony/open_sprite_runtime/build/policy_native_stage2/include','-DSPRITE_ORT_LIBRARY='+lib],check=True)
(s/'build/libonnxruntime.so.1').symlink_to(lib)
subprocess.run(['cmake','--build',str(s/'build'),'-j2'],check=True)
subprocess.run(['ctest','--test-dir',str(s/'build'),'--output-on-failure'],check=True)
for name in ['operator-xbox.sh','ensure-can-scheduler.sh','load-echo-order-driver.sh']:
    subprocess.run(['bash','-n',str(s/name)],check=True)
subprocess.run(['python3','-m','py_compile',str(s/'tools/run_supermimic_commissioning.py'),str(s/'run-disabled.py')],check=True)
names=['build/sprite_can_shadow','build/sprite_supermimic_protected_ipc','bundle/policy.onnx',
       'tools/run_supermimic_commissioning.py','operator-xbox.sh','run-disabled.py',
       'native/src/action_tx_trace.hpp','native/src/sling_window.hpp']
(s/'build-manifest.json').write_text(json.dumps(dict(control_window_s=300,duration_s=326,policy_frames=16301,
    expected_driver_srcversion='404E8D65A6EFF60079EAEC1',
    files={name:hashlib.sha256((s/name).read_bytes()).hexdigest() for name in names}),indent=2)+'\n')
print('Five-minute build and native tests passed. No motor frames sent.')
