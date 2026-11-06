"""Full physical command evaluation only after the authorized run completes."""
from pathlib import Path
import os,sys,json,subprocess,time,hashlib,shutil,datetime,math
ROOT=Path(__file__).resolve().parents[1]
TRAIN=ROOT/'outputs/joint-waist-v5-002'
OUT=ROOT/'outputs/final-evaluation-002'
STATE=ROOT/'post_training_state.json'
MOTION='data/motions/contact_v5_v11/v5_locomotion.npz'

def write_state(stage,**extra):
    payload={'stage':stage,'updated_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
             'hardware_qualified':False,'hardware_tx':0,**extra}
    tmp=STATE.with_suffix('.json.tmp');tmp.write_text(json.dumps(payload,indent=2)+'\n');tmp.replace(STATE)
    print(json.dumps(payload),flush=True)

def run(command,log):
    with log.open('w') as stream:
        subprocess.run([sys.executable,*command],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=1200)

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def command_bank():
    rows=[('stand',[0,0,0]),('forward010',[.10,0,0]),('forward015',[.15,0,0]),
          ('forward030',[.30,0,0]),('forward045',[.45,0,0]),('forward060',[.60,0,0]),
          ('reverse015',[-.15,0,0]),('reverse030',[-.30,0,0]),
          ('slow_left',[.15,0,.15]),('slow_right',[.15,0,-.15]),
          ('left',[.30,0,.15]),('right',[.30,0,-.15]),
          ('sharp_left',[.30,0,.30]),('sharp_right',[.30,0,-.30]),
          ('turn_in_place_left',[0,0,.30]),('turn_in_place_right',[0,0,-.30]),
          ('lateral_left',[0,.15,0]),('lateral_right',[0,-.15,0]),
          ('diagonal_left',[.30,.15,0]),('diagonal_right',[.30,-.15,0])]
    return [dict(name=name,command=value,category='within_training_random_range') for name,value in rows]

def grade(report):
    rows=report['results'];survival=sum(x['survived'] for x in rows)/len(rows)
    # Rank actual command errors, not reward or an arbitrary foot-lift threshold.
    planar=sum(x.get('last5s_velocity_error_m_s',10.) for x in rows)/len(rows)
    yaw=sum(x.get('last5s_yaw_error_rad_s',10.) for x in rows)/len(rows)
    return dict(survival=survival,planar_error_m_s=planar,yaw_error_rad_s=yaw,
                rank_cost=100*(1-survival)+planar+.3*yaw)

def evaluate(checkpoint,output,interface,seed=101,commands=None,seconds=30):
    args=['scripts/evaluate_joint.py','--checkpoint',str(checkpoint),'--motion',MOTION,
          '--interface',interface,'--output',str(output),'--seconds',str(seconds),'--seed',str(seed)]
    if interface=='velocity':args+=['--initial-pose','urdf-zero']
    if commands:args+=['--commands-file',str(commands)]
    run(args,output.parent/(output.name+'.log'))
    return json.loads((output/'summary.json').read_text())

def main():
    os.chdir(ROOT);os.environ['MUJOCO_GL']='egl'
    os.environ['LD_LIBRARY_PATH']=str(ROOT/'.venv/lib/python3.12/site-packages/nvidia/cuda_nvrtc/lib')
    training_state=json.loads((ROOT/'run002_state.json').read_text())
    if training_state.get('status')!='training_completed':raise RuntimeError('Training has not completed successfully')
    OUT.mkdir(exist_ok=False)
    source=OUT/'evaluation-source';source.mkdir()
    for name in ('post_training_run002.py','evaluate_joint.py','render_velocity.py','export_joint.py'):
        shutil.copyfile(ROOT/'scripts'/name,source/name)
    commands=OUT/'commands.json';commands.write_text(json.dumps(command_bank(),indent=2)+'\n')
    latest=sorted(TRAIN.glob('model_*.pt'))[-1]
    candidates=[];seen=set()
    for label,path in [('final',latest),('prior_best_velocity',TRAIN/'best_velocity.pt'),('prior_best_motion',TRAIN/'best_motion.pt')]:
        if not path.exists():continue
        digest=sha(path)
        if digest in seen:continue
        seen.add(digest);candidates.append(dict(name=label,path=path,sha256=digest))
    ranked=[]
    for candidate in candidates:
        write_state('candidate_evaluation',candidate=candidate['name'])
        folder=OUT/candidate['name'];folder.mkdir()
        report=evaluate(candidate['path'],folder/'velocity-seed101','velocity',commands=commands)
        score=grade(report)
        ranked.append(dict(name=candidate['name'],path=str(candidate['path']),sha256=candidate['sha256'],
                           iteration=report['iteration'],primary_score=score,seed_reports=[str(folder/'velocity-seed101/summary.json')]))
    ranked.sort(key=lambda x:x['primary_score']['rank_cost'])
    for candidate in ranked[:2]:
        folder=OUT/candidate['name'];scores=[candidate['primary_score']]
        for seed in (202,303):
            write_state('multiseed_confirmation',candidate=candidate['name'],seed=seed)
            output=folder/f'velocity-seed{seed}'
            report=evaluate(Path(candidate['path']),output,'velocity',seed=seed,commands=commands)
            scores.append(grade(report));candidate['seed_reports'].append(str(output/'summary.json'))
        candidate['mean_rank_cost']=sum(x['rank_cost'] for x in scores)/len(scores)
        candidate['confirmation_scores']=scores
    confirmed=sorted(ranked[:2],key=lambda x:x['mean_rank_cost'])
    selected=confirmed[0]
    selection=dict(candidates=ranked,selected=selected,selection='Physical survival plus planar and yaw tracking across3seeds; no gait/hardware qualification',hardware_qualified=False)
    (OUT/'selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    selected_checkpoint=Path(selected['path']);folder=OUT/'selected';folder.mkdir()
    write_state('selected_motion_evaluation',selected=selected)
    for seed in (101,202,303):
        evaluate(selected_checkpoint,folder/f'motion-seed{seed}','motion',seed=seed)
    # Continuous tests expose low-speed initiation versus moving-to-slow hysteresis.
    sequences=[
        ('speed_ladder',[[.15,0,0],[.30,0,0],[.45,0,0],[.60,0,0],[.15,0,0],[0,0,0]]),
        ('turn_switch',[[.30,0,0],[.30,0,.30],[.30,0,0],[.30,0,-.30],[.15,0,-.15],[0,0,0]]),
        ('reverse_stop_forward',[[-.15,0,0],[-.30,0,0],[0,0,0],[.15,0,0],[.30,0,0],[0,0,0]]),
        ('low_speed_restart',[[.15,0,0],[.30,0,0],[.15,0,0],[0,0,0],[.15,0,0],[.30,0,0]])]
    sequence_path=OUT/'sequences.json'
    sequence_path.write_text(json.dumps([dict(name=name,command=commands_[0],stages=[dict(at_seconds=2+10*i,command=c) for i,c in enumerate(commands_)]) for name,commands_ in sequences],indent=2)+'\n')
    write_state('continuous_evaluation')
    evaluate(selected_checkpoint,folder/'continuous','velocity',commands=sequence_path,seconds=62)
    write_state('export_selected')
    run(['scripts/export_joint.py','--checkpoint',str(selected_checkpoint),'--output',str(folder/'deploy')],folder/'export.log')
    export=json.loads((folder/'deploy/contract.json').read_text())
    assert export['checkpoint_sha256']==selected['sha256'] and not export['hardware_qualified']
    write_state('rendering_videos')
    primary=OUT/selected['name']/'velocity-seed101'
    for source_path,output,indices in [
        (primary/'rollout.npz',folder/'stop-slow-forward-turn.mp4',[0,2,10,11]),
        (primary/'rollout.npz',folder/'speed-and-turn.mp4',[2,3,10,11]),
        (primary/'rollout.npz',folder/'reverse-lateral.mp4',[6,7,16,17]),
        (folder/'motion-seed101/rollout.npz',folder/'motion-tracking.mp4',[0,1,2,4]),
        (folder/'continuous/rollout.npz',folder/'continuous.mp4',[0,1,2,3])]:
        run(['scripts/render_velocity.py','--rollout',str(source_path),'--output',str(output),'--envs',*[str(x) for x in indices]],output.with_suffix('.log'))
    manifest={p.relative_to(OUT).as_posix():sha(p) for p in OUT.rglob('*') if p.is_file()}
    (OUT/'SHA256SUMS.json').write_text(json.dumps(manifest,indent=2)+'\n')
    write_state('physical_evaluation_complete',selected=selected,output=str(OUT),
                next='Native993/10frame replay, constrained-drive simulation, offline runtime preparation and video review; Jetson stays off')

if __name__=='__main__':
    try:main()
    except Exception as exc:
        write_state('post_training_failed',error=repr(exc));raise
