"""Bounded three-arm reward comparison, serialized GPU use and physical audits."""
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

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/targeted-upright-clearance-20261118'
OLD=ROOT/'outputs/posture-width-experiment'
MOTION=ROOT/'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz'
PARENT=OLD/'train-5000/model_005000.pt'
ARMS=['upright-clearance','upright','control']
ENV={**os.environ,'MUJOCO_GL':'egl','OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1','PYTHONUNBUFFERED':'1'}

def gate_and_rank(directory):
    """Allow bounded research, retaining failures separately from deployment selection."""
    data=json.loads((directory/'posture-width.json').read_text())
    rows={r['case_name']:r for r in data['results']}
    quality=json.loads((directory/'gait-quality-selection.json').read_text())
    geometry={r['case_name']:r['geometry'] for r in json.loads((directory/'locomotion-geometry.json').read_text())['results']}
    peaks=geometry['forward025']['event_peak_median_m']
    check={'survival':all(r['survived'] for r in rows.values()),
           'zero_lean_not_severe':abs(rows['zero'].get('pitch_median_deg',99))<6,
           'forward_motion':rows['forward025'].get('mean_vx_m_s',0)>.12,
           'bilateral_forward_clearance':all(x is not None and x>=.010 for x in peaks),
           'spacing':all(r.get('center_p05_m',0)>=.10 for r in rows.values()),
           'turn_directions':rows['left_curve'].get('mean_yaw_rad_s',0)>.07 and rows['right_curve'].get('mean_yaw_rad_s',0)<-.07}
    return {'research_continue':all(check.values()),'checks':check,
            'quality_failures':quality['failed_checks'],'quality_passed':quality['accepted_for_multiseed_validation'],
            'rank':[len(quality['failed_checks']),sum(abs(r['pitch_median_deg']) for r in rows.values())]}

def main():
    BASE.mkdir(exist_ok=True);status=BASE/'status.json'
    if status.exists():raise RuntimeError('Existing comparison state; refusing overwrite')
    deadline=dt.datetime(2026,11,19,14,tzinfo=dt.timezone(dt.timedelta(hours=8))).timestamp()
    state={'phase':'starting','started_unix':time.time(),'deadline_unix':deadline,'completed':[],
           'evaluations':{},'videos':[],'validation_directories':[],'hardware_access':False}
    def save(phase):
        state.update(phase=phase,updated_unix=time.time())
        tmp=status.with_suffix('.tmp');tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(status)
    def run(label,args,timeout=10800):
        remaining=deadline-time.time()
        if remaining<120:raise TimeoutError('Rental backup margin reached')
        save(label)
        with (BASE/(label+'.log')).open('x') as log:
            child=subprocess.Popen([sys.executable,*map(str,args)],cwd=ROOT,env=ENV,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code=child.wait(timeout=min(timeout,remaining))
                if code:raise subprocess.CalledProcessError(code,child.args)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                raise
        state['completed'].append(label);save(label+'-complete')
    def evaluate(arm,n,checkpoint):
        label=f'eval-{arm}-{n}';out=BASE/label
        run(label,['scripts/evaluate_joint.py','--checkpoint',checkpoint,'--motion',MOTION,'--interface','velocity',
            '--output',out,'--seconds','20','--seed','101','--commands-file','configs/posture-width-evaluation.json'],1800)
        run(label+'-posture',['scripts/score_posture_width.py',out],600)
        run(label+'-geometry',['scripts/summarize_locomotion_geometry.py','--evaluation',out,'--output',out/'locomotion-geometry.json'],600)
        run(label+'-selection',['scripts/assess_gait_quality.py',out,'--baseline',OLD/'baseline'],120)
        decision=gate_and_rank(out)
        state['evaluations'][label]={'checkpoint':str(checkpoint.relative_to(BASE)),
            'sha256':hashlib.sha256(checkpoint.read_bytes()).hexdigest(),**decision}
        save(label+'-audited')
        video=f'review-{arm}-{n}.mp4'
        run('video-'+label,['scripts/render_velocity.py','--rollout',out/'rollout.npz','--output',BASE/video,
            '--envs','0','1','3','4','--seconds','20','--bright','--camera-distance','1.7','--camera-azimuth','135'],900)
        state['videos'].append(video);save(label+'-video-saved');return decision
    try:
        save('starting')
        previous={arm:PARENT for arm in ARMS};active=list(ARMS)
        for n in [600,1600,3000]:
            for arm in list(active):
                cfg=json.loads((ROOT/f'configs/targeted-{arm}-20261118.json').read_text())
                cfg.update(iterations=n,max_wall_seconds=18000,checkpoint_iterations=[n])
                config=BASE/f'{arm}-{n}.json';config.write_text(json.dumps(cfg,indent=2)+'\n')
                out=BASE/f'train-{arm}-{n}';resume=n!=600
                run(f'train-{arm}-{n}',['scripts/train_joint.py','--config',config,'--motion',MOTION,'--quality',MOTION.with_suffix('.qa.json'),
                    '--output',out,'--resume' if resume else '--transfer',previous[arm]])
                checkpoint=out/f'model_{n:06d}.pt'
                if not checkpoint.exists():raise RuntimeError('Stage ended without requested checkpoint')
                previous[arm]=checkpoint
                decision=evaluate(arm,n,checkpoint)
                if not decision['research_continue']:active.remove(arm)
            if not active:break
        shortlist=sorted([(r['rank'],label,r) for label,r in state['evaluations'].items() if r['research_continue']])[:2]
        state['shortlist']=[label for _,label,_ in shortlist];save('shortlist-selected')
        for _,label,r in shortlist:
            ck=BASE/r['checkpoint']
            for seed in [202,303]:
                name=f'audit-{label}-{seed}';out=BASE/name
                run(name,['scripts/evaluate_joint.py','--checkpoint',ck,'--motion',MOTION,'--interface','velocity','--output',out,
                    '--seconds','30','--seed',seed,'--initial-tilt-noise-deg','2','--commands-file','configs/posture-width-evaluation.json'],1800)
                run(name+'-posture',['scripts/score_posture_width.py',out],600)
                run(name+'-geometry',['scripts/summarize_locomotion_geometry.py','--evaluation',out,'--output',out/'locomotion-geometry.json'],600)
                state['validation_directories'].append(name);save(name+'-audited')
            name=f'audit-{label}-forces';out=BASE/name
            run(name,['scripts/evaluate_joint.py','--checkpoint',ck,'--motion',MOTION,'--interface','velocity','--output',out,
                '--seconds','20','--seed','404','--commands-file','configs/velocity-recovery-evaluation.json'],1800)
            state['validation_directories'].append(name);save(name+'-audited')
        save('complete-awaiting-result-review')
    except Exception:
        state['error']=traceback.format_exc();save('failed');raise

if __name__=='__main__':main()
