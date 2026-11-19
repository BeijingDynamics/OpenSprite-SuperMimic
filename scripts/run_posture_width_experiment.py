"""Serialize transfer training and physical audits; never access robot hardware."""
import json,os,subprocess,sys,time,traceback,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];os.chdir(ROOT)
BASE=ROOT/'outputs/posture-width-experiment'
flags=set(sys.argv[1:])
if flags-{'--continue-job','--day-campaign'}:raise ValueError('Unsupported argument')
continuing='--continue-job' in flags
day_campaign='--day-campaign' in flags
if day_campaign and not continuing:raise ValueError('Day campaign requires an existing reviewed job')
BASE.mkdir(exist_ok=continuing)
STATUS=BASE/'status.json'
state=json.loads(STATUS.read_text()) if continuing else {'phase':'initializing','completed':[],'hardware_access':False}
if continuing:
    (BASE/('status-before-restart-'+str(int(time.time()))+'.json')).write_text(json.dumps(state,indent=2)+'\n')
    state.pop('error',None)
env={**os.environ,'MUJOCO_GL':'egl','PYTHONUNBUFFERED':'1','OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1'}
parent=ROOT/'outputs/seed45-lr-continuation/actor-quarter/training/model_010000.pt'
motion=ROOT/'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz';quality=motion.with_suffix('.qa.json')
def save(phase):
    state.update(phase=phase,updated_unix=time.time());temp=STATUS.with_suffix('.tmp');temp.write_text(json.dumps(state,indent=2)+'\n');temp.replace(STATUS)
def run(label,args,timeout=10800):
    if label in state['completed']:return
    save(label)
    with (BASE/(label+'.log')).open('w') as stream:
        subprocess.run([sys.executable,*map(str,args)],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=timeout)
    state['completed'].append(label);save(label+'-complete')
def evaluate(label,checkpoint):
    path=BASE/label
    run(label,['scripts/evaluate_joint.py','--checkpoint',checkpoint,'--motion',motion,'--interface','velocity','--output',path,'--seconds','20','--seed','101','--commands-file','configs/posture-width-evaluation.json'],1800)
    run(label+'-geometry',['scripts/score_posture_width.py',path],600)
    run(label+'-clearance',['scripts/summarize_locomotion_geometry.py','--evaluation',path,'--output',path/'locomotion-geometry.json'],600)
    return json.loads((path/'posture-width.json').read_text())
def gate(baseline,candidate):
    b={r['case_name']:r for r in baseline['results']};c={r['case_name']:r for r in candidate['results']}
    checks={'all_cases_survived':all(r['survived'] for r in c.values()),
        'zero_pitch_not_worse':abs(c['zero'].get('pitch_median_deg',999))<=abs(b['zero']['pitch_median_deg'])+1.5,
        'forward025_speed':c['forward025'].get('mean_vx_m_s',0)>=.12,
        'forward025_no_new_crossing':c['forward025'].get('crossed_fraction',1)<=b['forward025']['crossed_fraction']+.005,
        'forward025_width_not_worse':c['forward025'].get('center_p05_m',0)>=b['forward025']['center_p05_m']-.005}
    return dict(checks=checks,passed=all(checks.values()),purpose='Pilot continuation only; not target attainment or hardware qualification')
try:
    run('tests',['-m','pytest','tests/test_posture_width.py','tests/test_sole_tracking_extension.py','tests/test_joint_v2_contract.py','-q'],600)
    baseline=evaluate('baseline',parent)
    run('pilot',['scripts/train_joint.py','--config','configs/posture-width-pilot.json','--motion',motion,'--quality',quality,'--output',BASE/'pilot','--transfer',parent],7200)
    ck=BASE/'pilot/model_000200.pt'
    result=evaluate('pilot-eval',ck);state['pilot_gate']=gate(baseline,result);save('pilot-reviewed')
    run('pilot-video',['scripts/render_velocity.py','--rollout',BASE/'pilot-eval/rollout.npz','--output',BASE/'pilot-review.mp4','--envs','0','1','3','4','--seconds','20','--bright','--camera-distance','1.7','--camera-azimuth','135'],900)
    if not state['pilot_gate']['passed']:
        save('pilot-needs-review');sys.exit(0)
    milestones=[500,1000,2000,3000,5000,8000,10000] if day_campaign else [500,1000,2000,3000]
    for milestone in milestones:
        cfg=json.loads((ROOT/'configs/posture-width-pilot.json').read_text())
        cfg.update(iterations=milestone,max_wall_seconds=43200 if day_campaign else 21600,checkpoint_iterations=[milestone],save_seconds=900)
        config=BASE/f'continue-{milestone}.json'
        if f'continue-{milestone}' not in state['completed']:
            config.write_text(json.dumps(cfg,indent=2)+'\n')
        out=BASE/f'train-{milestone}'
        run(f'continue-{milestone}',['scripts/train_joint.py','--config',config,'--motion',motion,'--quality',quality,'--output',out,'--resume',ck],10800)
        ck=out/f'model_{milestone:06d}.pt'
        if not ck.exists():save('training-budget-ended');sys.exit(0)
        result=evaluate(f'eval-{milestone}',ck);assessment=gate(baseline,result)
        state.setdefault('continuation_checks',{})[str(milestone)]=assessment
        state['latest_checkpoint']=str(ck);state['latest_checkpoint_sha256']=hashlib.sha256(ck.read_bytes()).hexdigest()
        run(f'video-{milestone}',['scripts/render_velocity.py','--rollout',BASE/f'eval-{milestone}/rollout.npz','--output',BASE/f'review-{milestone}.mp4','--envs','0','1','3','4','--seconds','20','--bright','--camera-distance','1.7','--camera-azimuth','135'],900)
        if not assessment['passed']:save('continuation-needs-review');sys.exit(0)
    save('day-training-complete-awaiting-validation' if day_campaign else 'planned-training-complete-awaiting-full-validation')
except Exception:
    state['error']=traceback.format_exc();save('failed');raise
