"""Compare shared and dedicated CPU placement using frozen R1 recorded inputs only."""
import json
import os
from pathlib import Path
import subprocess
import sys

STAGE=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
ROOT=Path('/home/tony/inference-diagnosis-20261117')
sys.path.insert(0,str(STAGE/'tools'))
from cpu_frequency_client import CpuFrequencyLease

active=subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
if any(p.strip().startswith(('sprite_can','sprite_super','sprite_icm','probe')) for p in active):
    raise RuntimeError('Control or probe process active')
out=STAGE/'affinity-probe';out.mkdir(exist_ok=False)
lease=CpuFrequencyLease('/run/open-sprite-cpu-frequency/lease.sock')
report={'acquired':lease.acquired,'cases':[]}
try:
    for i,mode in enumerate(['shared','pinned','shared']):
        prefix=out/f'{i:02d}-{mode}'
        command=[str(ROOT/'probe'),str(STAGE/'bundle/policy.onnx'),
                 '/home/tony/sprite-sm-walk-r1-xbox120-20261119/xbox-powered-20261119-144201-5126/native.bin',
                 str(prefix),mode,'1000','20000','1']
        env={**os.environ,'LD_LIBRARY_PATH':str(ROOT)}
        for k in list(env):
            if k.startswith('PROBE_'):env.pop(k)
        run=subprocess.run(command,env=env,text=True,capture_output=True,timeout=60)
        prefix.with_suffix('.log').write_text(run.stdout+run.stderr)
        if run.returncode:raise RuntimeError(run.stderr)
        result=json.loads(prefix.with_suffix('.json').read_text())
        report['cases'].append(result);print(json.dumps(result),flush=True)
finally:
    report['released']=lease.close()
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
