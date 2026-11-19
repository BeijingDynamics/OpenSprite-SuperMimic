"""Back up extended training and validation evidence during this bounded rental job."""
import hashlib
import json
from pathlib import Path
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/retrain-posture-width-20261117/evidence'
REMOTE='/root/gpufree-data/supermimic_posture_width_20261117/outputs/posture-width-experiment'
HOST='root@120.209.70.195'
STATUS=OUT/'day-backup-status.json'

def read_remote(name):
    return json.loads(subprocess.check_output(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10',
        '-p','30179',HOST,'cat '+REMOTE+'/'+name],timeout=30,text=True))

def copy(remote,local,recursive=False):
    subprocess.run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=10','-P','30179',
        *(['-r'] if recursive else []),HOST+':'+REMOTE+'/'+remote,str(local)],check=True,timeout=300)

def main():
    old=json.loads(STATUS.read_text()) if STATUS.exists() else {}
    saved=set(old.get('saved',[]));errors=old.get('errors',[])
    deadline=time.monotonic()+19*3600
    while time.monotonic()<deadline:
        campaign={}
        copied_ok=False
        try:
            state=read_remote('status.json');campaign=read_remote('day-campaign-status.json')
            (OUT/'day-remote-training-status.json').write_text(json.dumps(state,indent=2)+'\n')
            (OUT/'day-remote-campaign-status.json').write_text(json.dumps(campaign,indent=2)+'\n')
            for n in [5000,8000,10000]:
                label=f'eval-{n}'
                if label+'-clearance' in state['completed'] and label not in saved:
                    copy(label,OUT,True)
                    expected=json.loads((OUT/label/'summary.json').read_text())['checkpoint_sha256']
                    target=OUT/(label+'-checkpoint.pt');partial=target.with_suffix('.partial')
                    copy(f'train-{n}/model_{n:06d}.pt',partial)
                    if hashlib.sha256(partial.read_bytes()).hexdigest()!=expected:raise ValueError('Checkpoint hash mismatch')
                    partial.replace(target)
                    for name in [f'continue-{n}.json',f'continue-{n}.log']:
                        copy(name,OUT/name)
                    saved.add(label)
                label=f'video-{n}'
                if label in state['completed'] and label not in saved:
                    copy(f'review-{n}.mp4',OUT/f'review-{n}.mp4');saved.add(label)
            for label in campaign.get('completed',[]):
                if label.endswith('-geometry'):folder=label.removesuffix('-geometry')
                elif label.endswith('-forces'):folder=label
                else:continue
                if folder in saved:continue
                (OUT/'day-validation').mkdir(exist_ok=True)
                copy('day-validation/'+folder,OUT/'day-validation',True);saved.add(folder)
            copied_ok=True
        except Exception:errors.append(traceback.format_exc())
        result={'updated_unix':time.time(),'saved':sorted(saved),'errors':errors[-10:],'campaign_phase':campaign.get('phase')}
        STATUS.write_text(json.dumps(result,indent=2)+'\n')
        if copied_ok and campaign.get('phase') in ['complete-awaiting-human-review','no-surviving-research-candidate','failed']:
            print('Campaign backup finished:',campaign['phase'],flush=True);return
        time.sleep(45)

if __name__=='__main__':main()
