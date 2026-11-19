"""Bounded, serial data comparison, physical validation and export before rental expiry."""
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
from assess_teleop_candidate import assess

ROOT=Path(__file__).resolve().parents[1]
OLD=Path('/root/gpufree-data/supermimic_posture_width_20261117')
BASE=ROOT/'outputs/reference-fusion-campaign'
PARENT=OLD/'outputs/targeted-upright-clearance-20261118/train-upright-clearance-3000/model_003000.pt'
PM01=OLD/'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz'
MIX=ROOT/'reference/walk_stand.npz'
PARENT_SHA='884946b4f03a03d53057208bfdafe3b0c116f91fec06c493ec7307a49a9c5e4c'
ENV={**os.environ,'MUJOCO_GL':'egl','OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1','PYTHONUNBUFFERED':'1'}

def main():
    resume_wait=sys.argv[1:]==['--resume-wait']
    if sys.argv[1:] and not resume_wait:raise ValueError('Unsupported arguments')
    BASE.mkdir(parents=True,exist_ok=resume_wait)
    tz=dt.timezone(dt.timedelta(hours=8))
    train_deadline=dt.datetime(2026,11,19,8,30,tzinfo=tz).timestamp()
    deadline=dt.datetime(2026,11,19,10,30,tzinfo=tz).timestamp()
    state=dict(phase='waiting-for-standing-comparison',started_unix=time.time(),deadline_unix=deadline,
               completed=[],evaluations={},videos=[],validation_directories=[],hardware_access=False)
    if resume_wait:
        prior=json.loads((BASE/'status.json').read_text())
        if prior['phase']!='waiting-for-standing-comparison' or prior['completed']:
            raise RuntimeError('Only an idle waiting supervisor may be refreshed')
        (BASE/f'waiting-status-{int(time.time())}.json').write_text(json.dumps(prior,indent=2)+'\n')
        state['started_unix']=prior['started_unix']
    def save(phase):
        state.update(phase=phase,updated_unix=time.time());tmp=BASE/'status.tmp'
        tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(BASE/'status.json')
    def run(label,args,limit=1800):
        remaining=deadline-time.time()
        if remaining<60:raise TimeoutError('Delivery deadline reached')
        save(label)
        with (BASE/(label+'.log')).open('x') as f:
            child=subprocess.Popen([sys.executable,*map(str,args)],cwd=ROOT,env=ENV,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code=child.wait(timeout=min(limit,remaining))
                if code:raise subprocess.CalledProcessError(code,child.args)
            except BaseException:
                if child.poll() is None:
                    os.killpg(child.pid,signal.SIGTERM)
                    try:child.wait(timeout=20)
                    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                raise
        state['completed'].append(label);save(label+'-complete')
    cases=[dict(name=n,command=c) for n,c in [('zero',[0,0,0]),('forward025',[.25,0,0]),
        ('forward050',[.5,0,0]),('left_curve',[.35,0,.2]),('right_curve',[.35,0,-.2]),
        ('start_stop',[0,0,0]),('forward015',[.15,0,0])]]
    cases[5]['stages']=[dict(at_seconds=t,command=c) for t,c in [(2,[0,0,0]),(5,[.5,0,0]),(15,[0,0,0])]]
    commands=BASE/'commands.json';commands.write_text(json.dumps(cases,indent=2)+'\n')
    forces=[]
    for speed in [0,.5]:
        for axis in range(2):
            for direction in [-1,1]:
                f=[0,0,0];f[axis]=direction*10
                forces.append(dict(name=f'force_{speed}_{axis}_{direction}',command=[speed,0,0],forces=[dict(at_seconds=8,duration_seconds=.2,force_world_n=f)]))
        for direction in [-1,1]:
            forces.append(dict(name=f'pull_{speed}_{direction}',command=[speed,0,0],forces=[dict(at_seconds=8,duration_seconds=1,force_world_n=[0,direction*5,0])]))
    forcefile=BASE/'forces.json';forcefile.write_text(json.dumps(forces,indent=2)+'\n')
    def evaluate(label,ck,motion,command_file=commands,seconds=24,seed=101,tilt=0):
        out=BASE/label
        run(label,['scripts/evaluate_joint.py','--checkpoint',ck,'--motion',motion,'--interface','velocity',
            '--output',out,'--seconds',seconds,'--seed',seed,'--commands-file',command_file,
            '--initial-pose','controller-default','--initial-tilt-noise-deg',tilt])
        run(label+'-posture',['scripts/score_posture_width.py',out],300)
        run(label+'-geometry',['scripts/summarize_locomotion_geometry.py','--evaluation',out,'--output',out/'locomotion-geometry.json'],300)
        return out
    try:
        save(state['phase'])
        smoke=ROOT/'outputs/fusion-smoke/model_000003.pt'
        if not smoke.exists():raise RuntimeError('Full-method reference-change smoke has not completed')
        if hashlib.sha256(PARENT.read_bytes()).hexdigest()!=PARENT_SHA:raise ValueError('Parent mismatch')
        while True:
            old=json.loads((OLD/'outputs/evidence-followup-20261118/status.json').read_text())
            if old['phase']=='failed':raise RuntimeError('Preceding comparison failed; inspect before starting')
            if old['phase']=='complete-awaiting-result-review':break
            if time.time()>train_deadline:raise TimeoutError('Prior comparison exceeded training budget')
            time.sleep(30)
        basecfg=json.loads((OLD/'outputs/targeted-upright-clearance-20261118/upright-clearance-3000.json').read_text())
        diagnostic=dict(parent_sha256=PARENT_SHA,reference_sha256=hashlib.sha256(MIX.read_bytes()).hexdigest(),
            hypothesis='PM01-dominant sampling plus a reviewed standing demonstration improves standing without losing useful teleoperation gait.',
            arms=['pm01-control','walk-stand80'],reset_clip_mass=[.8,.2],
            low_speed_policy='Diagnostic only; not a selection veto.',training_deadline_unix=train_deadline)
        (BASE/'diagnosis.json').write_text(json.dumps(diagnostic,indent=2)+'\n');state['decision']=diagnostic
        baseline=evaluate('baseline',PARENT,PM01);br=assess(baseline)
        (baseline/'teleop-screen.json').write_text(json.dumps(br,indent=2)+'\n');state['validation_directories'].append('baseline')
        previous={a:PARENT for a in diagnostic['arms']};active=list(previous)
        for n in [1000,3000,6000]:
            for arm in list(active):
                prior_updates=0 if n==1000 else 1000 if n==3000 else 3000
                conservative_stage_seconds=(n-prior_updates)/.35+900
                if train_deadline-time.time()<conservative_stage_seconds:
                    state['training_cutoff_reached']=True;active=[];break
                mixed=arm=='walk-stand80';motion=MIX if mixed else PM01
                cfg=json.loads(json.dumps(basecfg));cfg.update(seed=49,experiment_id='reference-fusion-'+arm,iterations=n,
                    max_wall_seconds=24000,checkpoint_iterations=[n],save_seconds=600,transfer_parent_sha256=PARENT_SHA,
                    transfer_reset_both_critics=True,transfer_critic_warmup_iterations=200,transfer_wm_warmup_iterations=100)
                if mixed:
                    cfg['reference_clip_weights']={'pm01_clearance_walk':.8,'Neutral_ID_001135_001495_stand':.2}
                    cfg['reference_transfer']=dict(revision='reviewed_reference_transfer_v1',source_motion_sha256=hashlib.sha256(PM01.read_bytes()).hexdigest(),
                        target_motion_sha256=hashlib.sha256(MIX.read_bytes()).hexdigest(),reset_both_critics=True)
                stem=f'{arm}-{n}';config=BASE/(stem+'.json');config.write_text(json.dumps(cfg,indent=2)+'\n')
                out=BASE/('train-'+stem)
                run('train-'+stem,['scripts/train_joint.py','--config',config,'--motion',motion,'--quality',motion.with_suffix('.qa.json'),
                    '--output',out,'--transfer' if n==1000 else '--resume',previous[arm]],min(10800,train_deadline-time.time()))
                ck=out/f'model_{n:06d}.pt'
                if not ck.exists():raise RuntimeError('Requested stage checkpoint missing')
                previous[arm]=ck;label='eval-'+stem;evaluation=evaluate(label,ck,motion);review=assess(evaluation,br)
                (evaluation/'teleop-screen.json').write_text(json.dumps(review,indent=2)+'\n')
                run(label+'-motion',['scripts/evaluate_joint.py','--checkpoint',ck,'--motion',motion,'--interface','motion',
                    '--output',evaluation/'motion-interface'],600)
                state['evaluations'][label]=dict(checkpoint=str(ck.relative_to(BASE)),sha256=hashlib.sha256(ck.read_bytes()).hexdigest(),
                    motion=str(motion),arm=arm,**review);save(label+'-audited')
                video='review-'+stem+'.mp4';run('video-'+stem,['scripts/render_velocity.py','--rollout',evaluation/'rollout.npz',
                    '--output',BASE/video,'--envs','0','1','2','4','--seconds','24','--bright','--camera-distance','1.7','--camera-azimuth','135'],600)
                state['videos'].append(video);save(label+'-video-saved')
                if not review['continue_training']:active.remove(arm)
            if not active:break
        ranked=sorted(state['evaluations'].items(),key=lambda pair:pair[1]['rank'])
        if not ranked:raise RuntimeError('No complete comparison checkpoint')
        selected=ranked[:2];state['selected_for_validation']=[k for k,v in selected];save('validation-selection')
        for label,record in selected:
            ck=BASE/record['checkpoint'];motion=Path(record['motion'])
            for seed in [202,303]:
                name=f'validation-{label}-{seed}';evaluate(name,ck,motion,seconds=30,seed=seed,tilt=2)
                state['validation_directories'].append(name);save(name+'-saved')
            name='forces-'+label;evaluate(name,ck,motion,command_file=forcefile,seconds=24)
            state['validation_directories'].append(name);save(name+'-saved')
            for profile in ['delay_fixed_10ms','delay_fixed_20ms']:
                name=f'{profile}-{label}';out=BASE/name
                run(name,['scripts/evaluate_actuator_sensitivity.py','--profile',profile,'--checkpoint',ck,'--motion',motion,
                    '--output',out,'--commands-file',commands,'--seconds','24','--initial-pose','controller-default'])
                run(name+'-geometry',['scripts/summarize_locomotion_geometry.py','--evaluation',out,'--output',out/'locomotion-geometry.json'],300)
                state['validation_directories'].append(name);save(name+'-saved')
            name='export-'+label;run(name,['scripts/export_joint.py','--checkpoint',ck,'--output',BASE/name],600)
            run(name+'-replay',['scripts/verify_fusion_export.py','--export',BASE/name,'--rollout',BASE/label/'rollout.npz'],300)
            state['validation_directories'].append(name);save(name+'-saved')
        state['delivery_note']='Ranked research candidates exported; inspect all validation reports before runtime deployment. Hardware admission remains false.'
        save('complete-awaiting-result-review')
    except Exception:
        state['error']=traceback.format_exc();save('failed');raise

if __name__=='__main__':main()
