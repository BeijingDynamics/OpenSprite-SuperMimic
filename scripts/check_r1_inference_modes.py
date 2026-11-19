"""Compare recurrence outputs and paced CPU latency without motor or IMU access."""
import hashlib,json,os,subprocess,sys
from pathlib import Path
import numpy as np
s=Path('/home/tony/sprite-sm-walk-r1-fast-20261119');out=s/'inference-mode-parity';out.mkdir(exist_ok=False)
sys.path.insert(0,str(s/'tools'))
from cpu_frequency_client import CpuFrequencyLease
lease=CpuFrequencyLease('/run/open-sprite-cpu-frequency/lease.sock')
reports={}
try:
    for name,threads,flags in [('two-spin',2,[]),('two-sleep',2,['--no-ort-spinning']),('one',1,[])]:
        command=['taskset','-c','4,5',str(s/'build/sprite_supermimic_replay'),str(s/'bundle/policy.onnx'),
                 '/home/tony/sprite-sm-walk-r1-20261119/cpu-replay/env-00-handoff-off/input.bin',str(out/(name+'.bin')),
                 '--threads',str(threads),'--warmup','100','--isolated-inference','--benchmark',*flags]
        with (out/(name+'.log')).open('x') as log:
            subprocess.run(command,env={**os.environ,'LD_LIBRARY_PATH':str(s/'build')},stdout=log,stderr=subprocess.STDOUT,check=True,timeout=90)
        reports[name]=json.loads((out/(name+'.bin.timing.json')).read_text())
        reports[name]['sha256']=hashlib.sha256((out/(name+'.bin')).read_bytes()).hexdigest()
        print(name,reports[name]['inference_ms'],flush=True)
    dtype=np.dtype([('f','<f4',(1024,)),('d','<f8',(62,))])
    original=np.fromfile(out/'two-spin.bin',dtype=dtype)
    for name in ['two-sleep','one']:
        candidate=np.fromfile(out/(name+'.bin'),dtype=dtype)
        assert candidate.shape==original.shape
        difference=max(float(np.max(np.abs(candidate[k]-original[k]))) for k in ['f','d'])
        reports[name]['maximum_recurrence_difference']=difference
        assert difference<1e-4,(name,difference)
finally:
    restored=lease.close()
(out/'report.json').write_text(json.dumps(dict(cases=reports,frequency_restored=restored,hardware_tx=0),indent=2)+'\n')
print('Inference variants preserve the recorded policy recurrence; no motor traffic.')
