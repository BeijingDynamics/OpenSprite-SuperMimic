"""Finish the current job, extend unchanged training, then audit research candidates."""
import hashlib
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/posture-width-experiment'
OUT=BASE/'day-validation'
STATUS=BASE/'day-campaign-status.json'
TERMINAL={'planned-training-complete-awaiting-full-validation','day-training-complete-awaiting-validation',
          'failed','pilot-needs-review','continuation-needs-review','training-budget-ended'}
ENV={**os.environ,'MUJOCO_GL':'egl','PYTHONUNBUFFERED':'1','OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1'}

def select_candidates(base,limit=2):
    """Rank research candidates; ranking is explicitly not deployment approval."""
    found=[]
    for path in base.glob('eval-*/gait-quality-selection.json'):
        r=json.loads(path.read_text());iteration=int(path.parent.name.split('-')[1])
        ck=base/f'train-{iteration}/model_{iteration:06d}.pt'
        if not ck.exists():continue
        nominal=json.loads((path.parent/'posture-width.json').read_text())
        if not all(x['survived'] for x in nominal['results']):continue
        rank=(len(r['failed_checks']),sum(abs(x['pitch_median_deg']) for x in nominal['results']))
        found.append((rank,iteration,str(ck),r['accepted_for_multiseed_validation']))
    return sorted(found)[:limit]

def main():
    if STATUS.exists():raise RuntimeError('Campaign evidence exists; refusing overwrite')
    state={'phase':'waiting-for-current-training','completed':[],'hardware_access':False,
           'started_unix':time.time(),'deadline_unix':time.time()+18*3600}
    def save(phase):
        state.update(phase=phase,updated_unix=time.time())
        temp=STATUS.with_suffix('.tmp');temp.write_text(json.dumps(state,indent=2)+'\n');temp.replace(STATUS)
    def run(label,args,timeout):
        remaining=state['deadline_unix']-time.time()
        if remaining<60:raise TimeoutError('Bounded campaign deadline reached')
        save(label)
        with (BASE/(label+'.log')).open('x') as stream:
            child=subprocess.Popen([sys.executable,*map(str,args)],cwd=ROOT,env=ENV,stdout=stream,
                                   stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code=child.wait(timeout=min(timeout,remaining))
                if code:raise subprocess.CalledProcessError(code,child.args)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                raise
        state['completed'].append(label);save(label+'-complete')
    try:
        save(state['phase'])
        while True:
            status=json.loads((BASE/'status.json').read_text())
            if status['phase'] in TERMINAL:break
            if time.time()>state['deadline_unix']:raise TimeoutError('Current job did not finish before campaign deadline')
            time.sleep(30)
        if status['phase']=='planned-training-complete-awaiting-full-validation':
            run('extended-training',['scripts/run_posture_width_experiment.py','--continue-job','--day-campaign'],15*3600)
        state['training_terminal_state']=json.loads((BASE/'status.json').read_text())['phase']
        candidates=select_candidates(BASE)
        state['research_shortlist']=[{'iteration':i,'checkpoint':ck,'nominal_screen_passed':passed,
                                     'failed_check_count':rank[0]} for rank,i,ck,passed in candidates]
        if not candidates:save('no-surviving-research-candidate');return
        OUT.mkdir()
        parent=ROOT/'outputs/seed45-lr-continuation/actor-quarter/training/model_010000.pt'
        motion=ROOT/'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz'
        checkpoints=[('parent',parent)]+[(str(i),Path(ck)) for _,i,ck,_ in candidates]
        # Independent paired initial root tilts; nominal evaluation is deterministic.
        # These probes do not randomize actuator or physical model parameters.
        for tag,ck in checkpoints:
            for seed in [202,303]:
                label=f'audit-{tag}-seed{seed}';out=OUT/label
                run(label,['scripts/evaluate_joint.py','--checkpoint',ck,'--motion',motion,'--interface','velocity',
                    '--output',out,'--seconds','30','--seed',seed,'--initial-tilt-noise-deg','2',
                    '--commands-file','configs/posture-width-evaluation.json'],1800)
                run(label+'-posture',['scripts/score_posture_width.py',out],600)
                run(label+'-geometry',['scripts/summarize_locomotion_geometry.py','--evaluation',out,
                    '--output',out/'locomotion-geometry.json'],600)
            label=f'audit-{tag}-forces';out=OUT/label
            run(label,['scripts/evaluate_joint.py','--checkpoint',ck,'--motion',motion,'--interface','velocity',
                '--output',out,'--seconds','20','--seed','404','--commands-file','configs/velocity-recovery-evaluation.json'],1800)
        state['checkpoint_hashes']={tag:hashlib.sha256(ck.read_bytes()).hexdigest() for tag,ck in checkpoints}
        save('complete-awaiting-human-review')
    except Exception:
        state['error']=traceback.format_exc();save('failed');raise

if __name__=='__main__':main()
