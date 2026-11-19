"""Preserve completed evidence from the renewed rental's bounded comparison."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/targeted-upright-clearance-20261118/evidence'
REMOTE='/root/gpufree-data/supermimic_posture_width_20261117/outputs/targeted-upright-clearance-20261118'
HOST='root@120.209.70.195'

def copy(name,destination,recursive=False):
    subprocess.run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=10','-P','30179',
        *(['-r'] if recursive else []),HOST+':'+REMOTE+'/'+name,str(destination)],check=True,timeout=300)

def main():
    OUT.mkdir(parents=True,exist_ok=True);status=OUT/'backup-status.json'
    old=json.loads(status.read_text()) if status.exists() else {};saved=set(old.get('saved',[]));errors=old.get('errors',[])
    deadline=dt.datetime(2026,11,19,14,55,tzinfo=dt.timezone(dt.timedelta(hours=8))).timestamp()
    while time.time()<deadline:
        state={};ok=False
        try:
            state=json.loads(subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','-p','30179',HOST,
                'cat '+REMOTE+'/status.json'],timeout=30,text=True))
            (OUT/'remote-status.json').write_text(json.dumps(state,indent=2)+'\n')
            if 'diagnosis' not in saved and ('decision' in state or (not state.get('phase','').startswith('waiting-') and state.get('phase')!='failed')):
                copy('diagnosis.json',OUT/'diagnosis.json');saved.add('diagnosis')
            for label,record in state['evaluations'].items():
                if label in saved:continue
                copy(label,OUT,True)
                target=OUT/(label+'-checkpoint.pt');partial=target.with_suffix('.partial')
                copy(record['checkpoint'],partial)
                if hashlib.sha256(partial.read_bytes()).hexdigest()!=record['sha256']:raise ValueError('Checkpoint hash mismatch: '+label)
                partial.replace(target)
                stem=label.removeprefix('eval-');copy(stem+'.json',OUT/(stem+'.json'))
                copy('train-'+stem+'/metrics.jsonl',OUT/(stem+'-metrics.jsonl'))
                saved.add(label)
            for name in state['videos']:
                if name not in saved:copy(name,OUT/name);saved.add(name)
            for name in state['validation_directories']:
                if name not in saved:copy(name,OUT,True);saved.add(name)
            if state['phase'] in ['complete-awaiting-result-review','failed']:
                copy('*.log',OUT);saved.add('terminal-logs')
            ok=True
        except Exception:errors.append(traceback.format_exc())
        status.write_text(json.dumps({'updated_unix':time.time(),'saved':sorted(saved),'errors':errors[-10:],
                                     'remote_phase':state.get('phase')},indent=2)+'\n')
        if ok and state.get('phase') in ['complete-awaiting-result-review','failed']:return
        time.sleep(45)

if __name__=='__main__':main()
