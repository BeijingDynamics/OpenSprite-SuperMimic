"""Run isolated Jetson CPU replay trials; never opens control devices or IPC."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time


def read(path):
    try:
        return Path(path).read_text().strip().strip('\0')
    except OSError:
        return None


def environment():
    paths = list(Path('/sys/devices/system/cpu/cpufreq').glob('policy*/*'))
    selected = {'scaling_cur_freq','scaling_min_freq','scaling_max_freq','scaling_governor','related_cpus'}
    return dict(monotonic_ns=time.monotonic_ns(),
                board=read('/proc/device-tree/model'),
                schedstats=read('/proc/sys/kernel/sched_schedstats'),
                cpu={str(p):read(p) for p in paths if p.name in selected},
                temperatures={read(p/'type'):read(p/'temp') for p in Path('/sys/class/thermal').glob('thermal_zone*')})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--label',required=True)
    parser.add_argument('--frames',type=int,default=1000)
    parser.add_argument('--period-us',type=int,default=20000)
    parser.add_argument('--diagnostic',type=int,choices=[0,1],default=0)
    parser.add_argument('--busy-pacing',action='store_true')
    parser.add_argument('--fixed-input',action='store_true')
    parser.add_argument('--block-base',type=int,choices=[0,4],default=0)
    parser.add_argument('--op-profile',action='store_true')
    parser.add_argument('--modes',nargs='+',default=['single4','shared','pinned','pinned','shared','single4'])
    args = parser.parse_args()
    root = args.root.resolve()
    model = Path('/home/tony/supermimic-pm01-1116/seed45-quarter10000/policy.onnx')
    trace = Path('/home/tony/supermimic-pm01-xbox30-1116/xbox-powered-20261116-153129-5687/native.bin')
    env = os.environ.copy(); env['LD_LIBRARY_PATH']=str(root)
    for key in ('PROBE_BUSY_PACING','PROBE_FIXED_INPUT','PROBE_BLOCK_BASE','PROBE_ORT_PROFILE'):
        env.pop(key,None)
    for enabled,key in [(args.busy_pacing,'PROBE_BUSY_PACING'),(args.fixed_input,'PROBE_FIXED_INPUT'),(args.op_profile,'PROBE_ORT_PROFILE')]:
        if enabled:env[key]='1'
    if args.block_base:env['PROBE_BLOCK_BASE']=str(args.block_base)
    for index, mode in enumerate(args.modes):
        # Fail closed if a hardware controller is active; this probe needs no devices.
        names = subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
        if any(n.strip().startswith(('sprite_can','sprite_zero','sprite_policy','sprite_super','icm45686')) for n in names):
            raise RuntimeError('A controller or live actor may be active; refusing CPU load')
        prefix = root/f'{args.label}-{index:02d}-{mode}'
        if prefix.with_suffix('.json').exists():
            raise FileExistsError(prefix)
        before = environment()
        command = [str(root/'probe'),str(model),str(trace),str(prefix),mode,str(args.frames),str(args.period_us),str(args.diagnostic)]
        print('Starting '+prefix.name,flush=True)
        result = subprocess.run(command,env=env,capture_output=True,text=True,timeout=args.frames*max(abs(args.period_us),20000)/1e6+90)
        prefix.with_suffix('.log').write_text(result.stdout+result.stderr)
        prefix.with_suffix('.environment.json').write_text(json.dumps(dict(before=before,after=environment(),command=command,probe_environment={k:v for k,v in env.items() if k.startswith('PROBE_')},returncode=result.returncode),indent=2)+'\n')
        print(result.stdout.strip(),flush=True)
        if result.returncode:
            raise RuntimeError(result.stderr or 'CPU probe parity failed')
        # Avoid carrying a hot unpaced trial directly into the next warmup.
        time.sleep(1)


if __name__ == '__main__':
    main()
