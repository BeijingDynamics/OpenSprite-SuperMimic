"""Temporary CPU-floor experiment with automatic restoration and clock telemetry.

Only an isolated recorded-input probe is launched, as the unprivileged user.
No controller, device, CAN setup, model or protection file is modified.
"""
import argparse
import csv
from datetime import datetime
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

ROOT = Path('/home/tony/inference-diagnosis-20261117')
POLICY = Path('/sys/devices/system/cpu/cpufreq/policy4')
EMC = Path('/sys/kernel/debug/bpmp/debug/clk/emc/rate')


def read(path):
    try:
        return Path(path).read_text().strip()
    except OSError:
        return 'unavailable'


def set_floor(value):
    (POLICY/'scaling_min_freq').write_text(value+'\n')
    deadline=time.monotonic()+3
    observed=[]
    while True:
        actual=read(POLICY/'scaling_min_freq')
        if not observed or observed[-1]!=actual:observed.append(actual)
        if actual==value:
            print(f'CPU floor readback confirmed: {value} kHz; observed {observed}',flush=True)
            return
        if time.monotonic()>=deadline:
            raise RuntimeError(f'CPU floor did not settle: requested={value}, observed={observed}, cap={read(POLICY/"scaling_max_freq")}')
        time.sleep(.02)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--describe',action='store_true')
    args=parser.parse_args()
    if args.describe:
        print('Runs three 20-second pinned CPU-only replays: original CPU floor, floor raised to the existing cap, restored floor. Records CPU/EMC clocks. Restores the original floor on exit; never changes the cap, governor, model, protection or hardware controls.')
        return
    if os.geteuid()!=0:
        raise SystemExit('sudo is required only for CPU frequency bounds and read-only EMC telemetry')
    names=subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
    if any(n.strip().startswith(('sprite_can','sprite_zero','sprite_policy','sprite_super','icm45686','probe')) for n in names):
        raise SystemExit('A controller or probe is active; refusing to run')
    model=Path('/home/tony/supermimic-pm01-1116/seed45-quarter10000/policy.onnx')
    expected='ec638542ea681da6fcd56a9888f3f09ef13eb92a60ab1d75b9aae9131600abf6'
    if hashlib.sha256(model.read_bytes()).hexdigest()!=expected:
        raise SystemExit('Policy hash mismatch')
    label_prefix='power-'+datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+str(os.getpid())
    telemetry=ROOT/(label_prefix+'-clocks.csv')
    original=(POLICY/'scaling_min_freq').read_text().strip()
    cap=(POLICY/'scaling_max_freq').read_text().strip()
    if not (0<int(original)<=int(cap)) or read(POLICY/'related_cpus')!='4 5':
        raise SystemExit('Unexpected CPU cluster or frequency bounds')
    os.sched_setaffinity(0,{0})
    rows=[];stop=threading.Event();child=None
    def sample():
        while not stop.is_set():
            rows.append((time.monotonic_ns(),read(POLICY/'scaling_cur_freq'),read(EMC)))
            stop.wait(.01)
    def interrupt(signum,frame):
        raise KeyboardInterrupt(f'Signal {signum}')
    signal.signal(signal.SIGTERM,interrupt);signal.signal(signal.SIGINT,interrupt)
    monitor=threading.Thread(target=sample,daemon=True);monitor.start()
    try:
        for suffix,floor in [('before',original),('cpufixed',cap),('restored',original)]:
            label=label_prefix+'-'+suffix
            set_floor(floor)
            print(f'{label}: CPU 4/5 minimum {floor} kHz; cap unchanged at {cap} kHz',flush=True)
            command=['runuser','-u','tony','--','python3',str(ROOT/'run_cpu_replay_probe.py'),
                     '--root',str(ROOT),'--label',label,'--frames','1000','--diagnostic','1','--modes','pinned']
            child=subprocess.Popen(command,start_new_session=True)
            code=child.wait(timeout=100);child=None
            if code:
                raise RuntimeError(f'CPU replay failed: {code}')
    finally:
        try:
            if child is not None and child.poll() is None:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait()
        finally:
            try:
                set_floor(original)
            finally:
                stop.set();monitor.join(timeout=2)
                with telemetry.open('w',newline='') as f:
                    writer=csv.writer(f);writer.writerow(['monotonic_ns','cpu4_khz','emc_hz']);writer.writerows(rows)
            restored=read(POLICY/'scaling_min_freq')
            print(f'CPU minimum restored to {restored} kHz. No motor commands were sent.',flush=True)
            if restored!=original:
                raise RuntimeError('CPU floor restoration readback failed')


if __name__=='__main__':
    main()
