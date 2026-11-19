"""Review the completed comparison and run a bounded one-factor follow-up."""
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
import numpy as np
from run_targeted_comparison import gate_and_rank
from improvement_retention import retain

ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT/'outputs/targeted-upright-clearance-20261118'
BASE=ROOT/'outputs/evidence-followup-20261118'
PARENT_BASE=ROOT/'outputs/posture-width-experiment'
MOTION=ROOT/'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz'
ENV={**os.environ,'MUJOCO_GL':'egl','OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1','PYTHONUNBUFFERED':'1'}

def standing_profile(directory):
    sys.path.insert(0,str(ROOT))
    from opensprite_tracker.robot import BODY_JOINT_NAMES
    summary=json.loads((directory/'summary.json').read_text())
    env=next(r['env'] for r in summary['results'] if r['case_name']=='zero')
    with np.load(directory/'rollout.npz') as z:
        pose=z['qpos'][150:,env]
    ids=[7+list(BODY_JOINT_NAMES).index(s+'_knee_joint') for s in ['left','right']]
    angles=np.rad2deg(np.median(pose[:,ids],axis=0))
    return {'median_knee_joint_degrees':angles.tolist(),'max_median_knee_flexion_deg':float(np.abs(angles).max()),
            'median_root_height_m':float(np.median(pose[:,2]))}

def clearance_score(directory):
    rows={r['case_name']:r['geometry'] for r in json.loads((directory/'locomotion-geometry.json').read_text())['results']}
    low=rows['forward015']['event_peak_median_m'];right=rows['right_curve']['event_peak_median_m']
    return sum(min(1.,max(0.,(v or 0)/target)) for v,target in zip([*low,right[1]],[.0135,.01425,.0158]))/3

def decide(state):
    """Use matched stages for a falsifiable next hypothesis, never a causal claim."""
    anchor=OLD/'eval-upright-clearance-600';candidates=[];reviews={};profiles={}
    for label,record in state['evaluations'].items():
        report=retain(OLD/label,anchor,PARENT_BASE/'eval-5000');reviews[label]=report
        checks=report['preservation_checks']
        preserved=checks['standing_lean_retained'] and all(v for k,v in checks.items() if k.endswith(('/spacing_retained','/no_crossing','/survival')))
        if preserved and record['research_continue']:
            profiles[label]=standing_profile(OLD/label)
            regular=checks['forward015/steps_retained']
            rank=(not regular,not checks['forward015/bilateral_clearance'],profiles[label]['max_median_knee_flexion_deg'],
                  len(report['failed_preservation_checks'])+len(record['quality_failures']),record['rank'][1])
            candidates.append((rank,label,record))
    if not candidates:return {'action':'no_preserved_parent','reviews':reviews}
    _,label,record=min(candidates)
    cfg=json.loads((OLD/(label.removeprefix('eval-')+'.json')).read_text())
    common=[n for n in [600,1600,3000] if all(f'eval-{arm}-{n}' in state['evaluations'] for arm in ['upright','upright-clearance'])]
    matched=None
    if common:
        n=max(common);a=clearance_score(OLD/f'eval-upright-{n}');b=clearance_score(OLD/f'eval-upright-clearance-{n}')
        matched={'update':n,'upright_score':a,'upright_clearance_score':b,'difference':b-a}
    all_passed=reviews[label]['candidate_for_further_validation']
    current=cfg['velocity_reward_extension'].get('swing_weight',0.)
    regular=reviews[label]['preservation_checks']['forward015/steps_retained']
    if regular and profiles[label]['max_median_knee_flexion_deg']>25:
        action='test_zero_command_knee_cost';target=current
    elif all_passed:action='replicate_selected_recipe';target=current
    elif current==0:action='test_small_clearance_weight';target=.075
    elif matched and matched['difference']>=.05:action='test_stronger_clearance_weight';target=min(.3,current*2)
    else:action='test_weaker_clearance_weight';target=current*.5
    return {'action':action,'parent_label':label,'parent_record':record,'parent_config':cfg,
            'matched_stage_evidence':matched,'control_weight':current,'treatment_weight':target,'reviews':reviews,
            'standing_profiles':profiles,'low_speed_alternation_present':regular,
            'user_priority':'More extended standing legs and reliable low-speed alternation; retain useful turning, arm naturalness secondary.',
            'interpretation':'One-factor hypothesis test; matched nominal results do not prove causality or hardware benefit.'}

def main():
    resume_wait=sys.argv[1:]==['--resume-wait']
    if sys.argv[1:] and not resume_wait:raise ValueError('Unsupported arguments')
    BASE.mkdir(exist_ok=resume_wait);status=BASE/'status.json'
    deadline=dt.datetime(2026,11,19,14,tzinfo=dt.timezone(dt.timedelta(hours=8))).timestamp()
    state={'phase':'waiting-for-comparison','started_unix':time.time(),'deadline_unix':deadline,
           'completed':[],'evaluations':{},'videos':[],'validation_directories':[],'hardware_access':False}
    if resume_wait:
        prior=json.loads(status.read_text())
        if prior['phase']!='waiting-for-comparison' or prior['completed']:raise RuntimeError('Can only refresh an idle waiting supervisor')
        (BASE/f'status-before-priority-update-{int(time.time())}.json').write_text(json.dumps(prior,indent=2)+'\n')
        state['started_unix']=prior['started_unix']
    def save(phase):
        state.update(phase=phase,updated_unix=time.time());tmp=status.with_suffix('.tmp')
        tmp.write_text(json.dumps(state,indent=2)+'\n');tmp.replace(status)
    def run(label,args,limit=10800):
        remaining=deadline-time.time()
        if remaining<120:raise TimeoutError('Rental backup deadline reached')
        save(label)
        with (BASE/(label+'.log')).open('x') as f:
            child=subprocess.Popen([sys.executable,*map(str,args)],cwd=ROOT,env=ENV,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                code=child.wait(timeout=min(limit,remaining))
                if code:raise subprocess.CalledProcessError(code,child.args)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=30)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
                raise
        state['completed'].append(label);save(label+'-complete')
    try:
        save(state['phase'])
        while time.time()<deadline:
            old=json.loads((OLD/'status.json').read_text())
            if old['phase']=='failed':raise RuntimeError('Original comparison failed; requires repair before follow-up')
            if old['phase']=='complete-awaiting-result-review':break
            time.sleep(30)
        else:raise TimeoutError('Comparison exceeded follow-up deadline')
        decision=decide(old)
        (BASE/'diagnosis.json').write_text(json.dumps(decision,indent=2)+'\n')
        state['decision']={k:v for k,v in decision.items() if k not in ['reviews','parent_config','parent_record']};save('evidence-reviewed')
        if decision['action']=='no_preserved_parent':save('complete-awaiting-result-review');return
        if deadline-time.time()<3*3600:raise TimeoutError('Insufficient time for paired follow-up and local preservation')
        parent=OLD/decision['parent_record']['checkpoint'];digest=hashlib.sha256(parent.read_bytes()).hexdigest()
        if digest!=decision['parent_record']['sha256']:raise ValueError('Selected parent hash mismatch')
        arms=['replicate'] if decision['action']=='replicate_selected_recipe' else ['matched-control','one-factor']
        previous={arm:parent for arm in arms};active=list(arms)
        for n in [600,1600]:
            for arm in list(active):
                cfg=json.loads(json.dumps(decision['parent_config']))
                cfg.update(seed=48,experiment_id='evidence-followup-'+arm,iterations=n,max_wall_seconds=10800,
                    transfer_parent_sha256=digest,checkpoint_iterations=[n],save_seconds=600,
                    initialization='New transfer from a reviewed posture/spacing-preserving checkpoint; fresh critic and optimizers in both arms.')
                if arm=='one-factor':
                    cfg['velocity_reward_extension'].update(revision='sprite_velocity_posture_width_swing_v2',
                        swing_height_m=.02,swing_speed_m_s=.08,swing_weight=decision['treatment_weight'],command_gate_m_s=.05)
                    if decision['action']=='test_zero_command_knee_cost':
                        cfg['velocity_reward_extension'].update(revision='sprite_velocity_standing_knee_v3',standing_knee_weight=.2,
                            standing_knee_deadband_deg=20.,standing_knee_scale_deg=20.,standing_velocity_scale_m_s=.03,standing_yaw_scale_rad_s=.05)
                stem=f'{arm}-{n}';config=BASE/(stem+'.json');config.write_text(json.dumps(cfg,indent=2)+'\n')
                if arm=='one-factor' and n==600 and decision['action']=='test_zero_command_knee_cost':
                    smoke=dict(cfg,transfer_critic_warmup_iterations=0)
                    smoke_config=BASE/'standing-smoke.json';smoke_config.write_text(json.dumps(smoke,indent=2)+'\n')
                    run('standing-smoke',['scripts/train_joint.py','--config',smoke_config,'--motion',MOTION,
                        '--quality',MOTION.with_suffix('.qa.json'),'--transfer',parent,'--output',BASE/'standing-smoke','--smoke'],600)
                out=BASE/('train-'+stem)
                run('train-'+stem,['scripts/train_joint.py','--config',config,'--motion',MOTION,'--quality',MOTION.with_suffix('.qa.json'),
                    '--output',out,'--transfer' if n==600 else '--resume',previous[arm]])
                ck=out/f'model_{n:06d}.pt';previous[arm]=ck;label='eval-'+stem;evaluation=BASE/label
                run(label,['scripts/evaluate_joint.py','--checkpoint',ck,'--motion',MOTION,'--interface','velocity',
                    '--output',evaluation,'--seconds','20','--seed','101','--commands-file','configs/posture-width-evaluation.json'],1800)
                run(label+'-posture',['scripts/score_posture_width.py',evaluation],600)
                run(label+'-geometry',['scripts/summarize_locomotion_geometry.py','--evaluation',evaluation,'--output',evaluation/'locomotion-geometry.json'],600)
                run(label+'-selection',['scripts/assess_gait_quality.py',evaluation,'--baseline',PARENT_BASE/'baseline'],120)
                review=retain(evaluation,OLD/'eval-upright-clearance-600',PARENT_BASE/'eval-5000')
                (evaluation/'retention-screen.json').write_text(json.dumps(review,indent=2)+'\n')
                research=gate_and_rank(evaluation)
                state['evaluations'][label]={'checkpoint':str(ck.relative_to(BASE)),'sha256':hashlib.sha256(ck.read_bytes()).hexdigest(),
                    **research,'retention_passed':review['preservation_passed'],'candidate_for_further_validation':review['candidate_for_further_validation']}
                save(label+'-audited')
                video='review-'+stem+'.mp4'
                run('video-'+stem,['scripts/render_velocity.py','--rollout',evaluation/'rollout.npz','--output',BASE/video,
                    '--envs','0','1','3','4','--seconds','20','--bright','--camera-distance','1.7','--camera-azimuth','135'],900)
                state['videos'].append(video);save(label+'-video-saved')
                if not research['research_continue']:active.remove(arm)
            if not active:break
        save('complete-awaiting-result-review')
    except Exception:
        state['error']=traceback.format_exc();save('failed');raise

if __name__=='__main__':main()
