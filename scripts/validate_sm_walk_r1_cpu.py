"""R1 native replay with the existing CPU lease; no hardware device access."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

STAGE=Path('/home/tony/sprite-sm-walk-r1-20261119')


def main():
    names=subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
    if any(n.strip().startswith(('sprite_can','sprite_zero','sprite_policy','sprite_super','sprite_icm')) for n in names):
        raise RuntimeError('A live control process is active')
    sys.path.insert(0,str(STAGE/'tools'))
    from cpu_frequency_client import CpuFrequencyLease
    lease=CpuFrequencyLease('/run/open-sprite-cpu-frequency/lease.sock')
    evidence=dict(acquired=lease.acquired,hardware_tx=0,live_devices_opened=0)
    try:
        if lease.acquired['guard_sha256']!=hashlib.sha256((STAGE/'tools/cpu_frequency_guard.py').read_bytes()).hexdigest():
            raise ValueError('Frequency service source mismatch')
        env={**os.environ,'PYTHONPATH':str(STAGE/'runtime/src'),
             'LD_LIBRARY_PATH':'/home/tony/supermimic-pm01-1116/bin'}
        command=[sys.executable,'/home/tony/supermimic-pm01-1116/runtime/tools/verify_supermimic_replay.py',
            '--contract',str(STAGE/'bundle/contract.json'),'--rollout',str(STAGE/'candidate/rollout.npz'),
            '--summary',str(STAGE/'candidate/summary.json'),
            '--binary','/home/tony/supermimic-pm01-1116/bin/sprite_supermimic_replay',
            '--output-dir',str(STAGE/'cpu-replay'),'--handoff','off','--benchmark','--threads','2','--max-frames','1000']
        with (STAGE/'cpu-replay.log').open('x') as stream:
            result=subprocess.run(['taskset','-c','4,5',*command],env=env,stdout=stream,stderr=subprocess.STDOUT,
                pass_fds=(lease.fileno(),),timeout=420)
        evidence['returncode']=result.returncode
        if result.returncode:raise RuntimeError('Native R1 replay failed; inspect cpu-replay.log')
    finally:
        evidence['released']=lease.close()
        (STAGE/'cpu-replay-frequency.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,indent=2))


if __name__=='__main__':main()
