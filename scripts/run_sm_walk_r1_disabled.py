"""R1 live feedback and protected-target validation; no enable option exists."""
import argparse
import json
import os
from pathlib import Path
import subprocess

STAGE=Path('/home/tony/sprite-sm-walk-r1-20261119')
OLD=Path('/home/tony/supermimic-pm01-cpufreq-1117')


def main():
    p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--fault',action='store_true');args=p.parse_args()
    if not args.label.replace('-','').isalnum():raise ValueError('Invalid evidence label')
    replay=json.loads((STAGE/'cpu-replay/report.json').read_text())
    if not replay['passed'] or replay['hardware_tx']!=0:raise ValueError('Native CPU replay must pass first')
    command=json.loads((OLD/'fixed-disabled-01-job.json').read_text())['command']
    command[1]=str(STAGE/'tools/run_supermimic_commissioning.py')
    for option,value in [('--bundle',STAGE/'bundle'),('--build',STAGE/'build'),
                          ('--output',STAGE/args.label),('--operator-status',STAGE/'operator-status.log')]:
        command[command.index(option)+1]=str(value)
    if args.fault:command.append('--deadline-fault')
    if '--enable-suspended' in command:raise ValueError('Enable is forbidden in this entry point')
    env={**os.environ,'PYTHONPATH':str(STAGE/'runtime/src'),'LD_LIBRARY_PATH':str(STAGE/'build')}
    (STAGE/(args.label+'-job.json')).write_text(json.dumps(dict(command=command),indent=2)+'\n')
    with (STAGE/(args.label+'.log')).open('x') as stream:
        result=subprocess.run(command,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=140)
    path=STAGE/args.label/'commissioning.json'
    if path.exists():
        report=json.loads(path.read_text())
        print(json.dumps({k:report.get(k) for k in ['mode','passed','failure','motor_enable_attempts','final_disabled_count','cpu_frequency','timing_max_ms']},indent=2))
        if report['motor_enable_attempts']!=0:raise RuntimeError('Unexpected enable attempt')
    else:print((STAGE/(args.label+'.log')).read_text()[-2500:])
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
