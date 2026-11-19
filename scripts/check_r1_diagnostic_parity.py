"""Verify optional diagnostic sampling leaves recorded policy outputs unchanged."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

stage=Path('/home/tony/sprite-sm-walk-r1-fast-20261119')
out=stage/'diagnostic-parity';out.mkdir(exist_ok=False)
sys.path.insert(0,str(stage/'tools'))
from cpu_frequency_client import CpuFrequencyLease
lease=CpuFrequencyLease('/run/open-sprite-cpu-frequency/lease.sock')
try:
    for name,extra in [('off',[]),('on',['--diagnose-inference'])]:
        command=['taskset','-c','4,5',str(stage/'build/sprite_supermimic_replay'),
                 str(stage/'bundle/policy.onnx'),
                 '/home/tony/sprite-sm-walk-r1-20261119/cpu-replay/env-00-handoff-off/input.bin',
                 str(out/(name+'.bin')),'--threads','2','--warmup','100',*extra]
        with (out/(name+'.log')).open('x') as log:
            subprocess.run(command,env={**os.environ,'LD_LIBRARY_PATH':str(stage/'build')},
                           stdout=log,stderr=subprocess.STDOUT,check=True,timeout=60)
    hashes={name:hashlib.sha256((out/(name+'.bin')).read_bytes()).hexdigest() for name in ['off','on']}
    assert hashes['off']==hashes['on']
    result=dict(passed=True,output_sha256=hashes,hardware_tx=0)
finally:
    restored=lease.close()
result['frequency_restored']=restored
(out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
