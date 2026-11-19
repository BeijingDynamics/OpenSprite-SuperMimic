"""Create the R1 operator entry from the validated runtime without executing it."""
import json
from pathlib import Path
import subprocess

STAGE=Path('/home/tony/sprite-sm-walk-r1-20261119')
OLD=Path('/home/tony/supermimic-pm01-cpufreq-1117')


def main():
    for label,mode in [('fixed-disabled-01','disabled'),('fixed-fault-01','deadline_fault')]:
        r=json.loads((STAGE/label/'commissioning.json').read_text())
        if not (r['passed'] and r['mode']==mode and r['motor_enable_attempts']==0 and r['final_disabled_count']==31):
            raise ValueError('Matching disabled and deadline evidence must pass before creating the operator entry')
    text=(OLD/'operator-xbox.sh').read_text()
    text=text.replace(str(OLD),str(STAGE)).replace('/home/tony/supermimic-pm01-live-1116',str(STAGE))
    text=text.replace('fixed-fault-02','fixed-fault-01').replace('new PM01 policy','Sprite SM-Walk R1 policy')
    anchor="stage, live, base = map(Path, sys.argv[1:])"
    addition="""
from open_sprite_runtime.supermimic_run002 import verify_bundle
binding=verify_bundle(live/'bundle')
if binding['policy_sha256']!='5695b6980746534a11d075befc29bf5f7fe0ef2a7d81e3dadf3147c50f4d6930':
    raise SystemExit('This entry is only for the frozen Sprite SM-Walk R1 export.')
replay=json.loads((stage/'cpu-replay/report.json').read_text())
if not replay['passed'] or any(case['model_sha256']!=binding['policy_sha256'] for case in replay['cases']):
    raise SystemExit('Matching R1 native CPU replay is required.')
"""
    if text.count(anchor)!=1:raise ValueError('Operator patch anchor mismatch')
    text=text.replace(anchor,anchor+addition)
    target=STAGE/'operator-xbox.sh';target.write_text(text)
    subprocess.run(['bash','-n',str(target)],check=True)
    subprocess.run(['bash',str(target),'--check'],check=True)
    print('R1 operator entry prepared and read-only checks passed. No powered test started.')


if __name__=='__main__':main()
