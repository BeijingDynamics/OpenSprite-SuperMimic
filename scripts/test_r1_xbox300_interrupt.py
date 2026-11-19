"""Exercise terminal SIGINT during no-enable control and inspect native shutdown."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

stage = Path('/home/tony/sprite-sm-walk-r1-xbox300-20261119')
parser = argparse.ArgumentParser()
parser.add_argument('--label', default='interrupt-01')
label = parser.parse_args().label
assert label.startswith('interrupt-') and label.replace('-', '').isalnum()
out = stage/label
assert not out.exists()
command = json.loads((stage/'disabled-04-job.json').read_text())['command']
assert '--enable-suspended' not in command and '--deadline-fault' not in command
command[command.index('--output')+1] = str(out)
status = stage/(label+'-status.log')
command[command.index('--operator-status')+1] = str(status)
env = {**os.environ, 'PYTHONPATH': str(stage/'runtime/src'),
       'LD_LIBRARY_PATH': str(stage/'build'), 'SPRITE_INFERENCE_THREADS': '2',
       'SPRITE_NO_ORT_SPINNING': '1', 'SPRITE_INFERENCE_DIAGNOSTICS': '0'}
with (stage/(label+'.log')).open('x') as stream:
    process = subprocess.Popen(command, env=env, stdout=stream, stderr=subprocess.STDOUT,
                               start_new_session=True)
    try:
        deadline = time.monotonic()+45
        while True:
            if process.poll() is not None:
                raise RuntimeError('No-enable control exited before interrupt test')
            if status.exists() and '300-second Xbox window opens next' in status.read_text():
                break
            if time.monotonic()>deadline:
                raise RuntimeError('No-enable control did not reach Xbox window')
            time.sleep(.2)
        time.sleep(2)
        os.killpg(process.pid, signal.SIGINT)
        code = process.wait(timeout=25)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=25)
can = json.loads((out/'can.json').read_text())
disabled = sum(m['final_disabled'] for m in can['motors'].values())
freq = json.loads((out/'cpu-frequency.json').read_text())
assert disabled == 31 and can['motor_enable_attempts'] == 0
assert freq['released']['status'] == 'restored'
active = [name.strip() for name in subprocess.check_output(['ps', '-eo', 'comm='], text=True).splitlines()
          if name.strip().startswith(('sprite_can', 'sprite_super', 'sprite_icm'))]
assert not active, active
result = dict(passed=True, signal='SIGINT to terminal process group', exit_code=code,
              final_disabled_count=disabled, motor_enable_attempts=0,
              cpu_frequency=freq, active_control_processes=active)
(out/'interrupt-verification.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result), flush=True)
