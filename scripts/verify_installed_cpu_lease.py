"""Verify installed frequency lease restoration after killing a dummy client.

No CAN, IMU, policy or motor process is launched.
"""
import json
from pathlib import Path
import subprocess
import sys
import time

tools=Path('/home/tony/supermimic-pm01-cpufreq-1117/tools')
policy=Path('/sys/devices/system/cpu/cpufreq/policy4')
def snapshot():
    return {k:(policy/k).read_text().strip() for k in ['scaling_min_freq','scaling_max_freq','scaling_governor']}
names=subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
if any(n.strip().startswith(('sprite_can','sprite_zero','sprite_super','supermimic','sprite_icm')) for n in names):
    raise SystemExit('A control process is active; refusing lifecycle test')
before=snapshot()
code='import sys,time;sys.path.insert(0,sys.argv[1]);from cpu_frequency_client import CpuFrequencyLease;l=CpuFrequencyLease("/run/open-sprite-cpu-frequency/lease.sock");print("ready",flush=True);time.sleep(60)'
child=subprocess.Popen([sys.executable,'-c',code,str(tools)],stdout=subprocess.PIPE,text=True)
try:
    assert child.stdout.readline().strip()=='ready'
    active=snapshot();assert active['scaling_min_freq']==active['scaling_max_freq']=='1344000'
    child.kill();child.wait(timeout=5)
    until=time.monotonic()+5
    while snapshot()!=before and time.monotonic()<until:time.sleep(.02)
    after=snapshot();assert after==before
    result=dict(passed=True,before=before,active=active,after=after,termination='SIGKILL dummy client',hardware_access=False)
    (tools.parent/'installed-lease-crash-test.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result))
finally:
    if child.poll() is None:child.kill();child.wait()
    child.stdout.close()
