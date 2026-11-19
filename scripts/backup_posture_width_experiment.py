"""Bounded local evidence copy for the explicitly launched November 17 job."""
import hashlib,json,subprocess,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/retrain-posture-width-20261117/evidence';OUT.mkdir(exist_ok=True)
REMOTE='/root/gpufree-data/supermimic_posture_width_20261117/outputs/posture-width-experiment'
HOST='root@120.209.70.195'
previous=json.loads((OUT/'backup-status.json').read_text()) if (OUT/'backup-status.json').exists() else {}
saved=set(previous.get('saved',[]));errors=previous.get('errors',[])
def scp(remote,local,recursive=False):
    subprocess.run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=10','-P','30179',*(['-r'] if recursive else []),HOST+':'+remote,str(local)],check=True,timeout=300)
def persist(state):
    (OUT/'backup-status.json').write_text(json.dumps(dict(updated_unix=time.time(),saved=sorted(saved),errors=errors[-10:],remote_state=state),indent=2)+'\n')
deadline=time.monotonic()+7*3600
while time.monotonic()<deadline:
    state={}
    try:
        text=subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','-p','30179',HOST,'cat '+REMOTE+'/status.json'],timeout=30,text=True)
        state=json.loads(text);done=state['completed']
        (OUT/'remote-status.json').write_text(text)
        for label,checkpoint in [('pilot-eval','pilot/model_000200.pt')]+[(f'eval-{n}',f'train-{n}/model_{n:06d}.pt') for n in [500,1000,2000,3000]]:
            if label+'-clearance' not in done or label in saved:continue
            scp(REMOTE+'/'+label,OUT,True)
            expected=json.loads((OUT/label/'summary.json').read_text())['checkpoint_sha256']
            destination=OUT/(label+'-checkpoint.pt');partial=destination.with_suffix('.partial')
            scp(REMOTE+'/'+checkpoint,partial)
            if hashlib.sha256(partial.read_bytes()).hexdigest()!=expected:raise ValueError('Checkpoint transfer hash mismatch: '+label)
            partial.replace(destination);saved.add(label);persist(state)
        for label,name in [('pilot-video','pilot-review.mp4')]+[(f'video-{n}',f'review-{n}.mp4') for n in [500,1000,2000,3000]]:
            if label in done and label not in saved:
                scp(REMOTE+'/'+name,OUT/name);saved.add(label);persist(state)
        persist(state)
        if state['phase'] in ['failed','pilot-needs-review','continuation-needs-review','training-budget-ended','planned-training-complete-awaiting-full-validation']:
            print('Evidence backup reached terminal job state:',state['phase'],flush=True);break
    except Exception:
        errors.append(traceback.format_exc());persist(state)
    time.sleep(45)
