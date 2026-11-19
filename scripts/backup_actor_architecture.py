"""Copy architecture campaign milestones locally with checkpoint hash verification."""
import datetime as dt
import hashlib
import json
from pathlib import Path
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/actor-architecture-20261119/evidence'
REMOTE='/root/gpufree-data/supermimic_architecture_20261119/outputs/actor-architecture-campaign'
HOST='root@120.209.70.195'
TERMINAL={'complete-awaiting-result-review','completed-with-errors','failed'}


def copy(name,destination,recursive=False):
    subprocess.run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=10','-P','30179',
        *(['-r'] if recursive else []),HOST+':'+REMOTE+'/'+name,str(destination)],check=True,timeout=300)


def main():
    OUT.mkdir(parents=True,exist_ok=True);status=OUT/'backup-status.json'
    old=json.loads(status.read_text()) if status.exists() else {};saved=set(old.get('saved',[]));errors=old.get('errors',[])
    deadline=dt.datetime(2026,11,20,14,50,tzinfo=dt.timezone(dt.timedelta(hours=8))).timestamp()
    while time.time()<deadline:
        state={};ok=False
        try:
            state=json.loads(subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','-p','30179',HOST,
                'cat '+REMOTE+'/status.json'],timeout=30,text=True))
            (OUT/'remote-status.json').write_text(json.dumps(state,indent=2)+'\n')
            if 'diagnosis' not in saved and 'decision' in state:
                copy('diagnosis.json',OUT/'diagnosis.json');copy('commands.json',OUT/'commands.json');saved.add('diagnosis')
            checkpoints={label.removeprefix('eval-'):dict(path=record['checkpoint'],sha256=record['sha256'])
                         for label,record in state.get('evaluations',{}).items()}
            for label in state.get('completed',[]):
                if not label.startswith('train-'):continue
                stem=label.removeprefix('train-')
                if stem in checkpoints or 'checkpoint-'+stem in saved:continue
                iteration=int(stem.rsplit('-',1)[1]);relative=f'{label}/model_{iteration:06d}.pt'
                digest=subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','-p','30179',HOST,
                    'sha256sum '+REMOTE+'/'+relative],timeout=30,text=True).split()[0]
                checkpoints[stem]=dict(path=relative,sha256=digest)
            for stem,record in checkpoints.items():
                key='checkpoint-'+stem
                if key in saved:continue
                target=OUT/(stem+'-checkpoint.pt');partial=target.with_suffix('.partial')
                copy(record['path'],partial)
                if hashlib.sha256(partial.read_bytes()).hexdigest()!=record['sha256']:raise ValueError('Checkpoint hash mismatch: '+stem)
                partial.replace(target)
                copy(stem+'.json',OUT/(stem+'.json'))
                copy('train-'+stem+'/metrics.jsonl',OUT/(stem+'-metrics.jsonl'))
                copy('train-'+stem+'/wm_validation.jsonl',OUT/(stem+'-wm-validation.jsonl'))
                saved.add(key)
            for label in state.get('evaluations',{}):
                if label not in saved:copy(label,OUT,True);saved.add(label)
            for name in state.get('videos',[]):
                if name not in saved:copy(name,OUT/name);saved.add(name)
            for name in state.get('validation_directories',[]):
                if name not in saved:copy(name,OUT,True);saved.add(name)
            if state.get('phase') in TERMINAL:
                copy('*.log',OUT);saved.add('terminal-logs')
            ok=True
        except Exception:errors.append(traceback.format_exc())
        status.write_text(json.dumps(dict(updated_unix=time.time(),saved=sorted(saved),errors=errors[-10:],remote_phase=state.get('phase')),indent=2)+'\n')
        if ok and state.get('phase') in TERMINAL:return
        time.sleep(45)


if __name__=='__main__':main()
