"""Run only disabled/fault commissioning comparisons with unchanged binaries."""
import argparse
import json
import os
from pathlib import Path
import subprocess


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--label',required=True)
    p.add_argument('--fixed',action='store_true')
    p.add_argument('--fault',action='store_true')
    args=p.parse_args()
    if not args.label.replace('-','').isalnum():raise ValueError('Invalid evidence label')
    stage=Path('/home/tony/supermimic-pm01-cpufreq-1117')
    old=Path('/home/tony/supermimic-pm01-xbox30-1116')
    command=json.loads((old/'disabled-01-job.json').read_text())['command']
    assert '--enable-suspended' not in command
    command[1]=str(stage/'tools/run_supermimic_commissioning.py')
    command[command.index('--output')+1]=str(stage/args.label)
    command[command.index('--operator-status')+1]=str(stage/'operator-status.log')
    if args.fixed:command+=['--cpu-frequency-socket','/run/open-sprite-cpu-frequency/lease.sock']
    if args.fault:command+=['--deadline-fault']
    env=dict(os.environ,PYTHONPATH='/home/tony/supermimic-pm01-live-1116/runtime/src',LD_LIBRARY_PATH=str(old/'build'))
    log=stage/(args.label+'.log')
    if log.exists():raise FileExistsError(log)
    (stage/(args.label+'-job.json')).write_text(json.dumps(dict(command=command),indent=2)+'\n')
    with log.open('w') as stream:
        result=subprocess.run(command,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=130)
    report_path=stage/args.label/'commissioning.json'
    if report_path.exists():
        r=json.loads(report_path.read_text())
        print(json.dumps({k:r.get(k) for k in ('mode','passed','failure','motor_enable_attempts','final_disabled_count','cpu_frequency','timing_max_ms')}),flush=True)
        if r.get('motor_enable_attempts')!=0:raise RuntimeError('Unexpected enable attempt')
    else:print(log.read_text()[-3000:],flush=True)
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
