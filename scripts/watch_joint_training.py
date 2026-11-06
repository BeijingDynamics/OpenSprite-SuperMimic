"""Evaluate saved joint policies periodically without asking or resetting failures.

Checks only complete atomic checkpoints, writes actual-physics rollouts and video.
Does not interpret an optimizer reward as a walking/turning success.
"""
import argparse,json,os,subprocess,sys,time,shutil,signal
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main(args):
    env={**os.environ,'MUJOCO_GL':'egl'};done=set();last=0.;started=time.monotonic();gait_rejections=0
    state=args.training/'automatic_evaluation.json'
    if state.exists():done=set(json.loads(state.read_text()).get('evaluated_iterations',[]))
    while time.monotonic()-started<args.max_seconds:
        checkpoints=sorted(args.training.glob('model_*.pt'))
        if checkpoints:
            checkpoint=checkpoints[-1];iteration=int(checkpoint.stem.split('_')[-1])
            if iteration not in done and (not done or time.monotonic()-last>=args.interval_seconds):
                destination=args.training/f'audit-{iteration:06d}';destination.mkdir(exist_ok=True)
                for interface in ['motion','velocity']:
                    output=destination/interface
                    if not (output/'summary.json').exists():
                        with (destination/(interface+'.log')).open('w') as log:
                            subprocess.run([sys.executable,'scripts/evaluate_joint.py','--checkpoint',str(checkpoint),'--motion',str(args.motion),
                                '--interface',interface,'--output',str(output),'--seconds','30','--seed','101'],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                    if not (output/'evaluation.mp4').exists():
                        selected=['0','1','2','4'] if interface=='motion' else ['0','1','3','4']
                        with (destination/(interface+'-render.log')).open('w') as log:
                            subprocess.run([sys.executable,'scripts/render_velocity.py','--rollout',str(output/'rollout.npz'),'--output',str(output/'evaluation.mp4'),'--envs',*selected],
                                cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                    gait_path=output/'gait_quality.json'
                    with (destination/(interface+'-gait.log')).open('w') as log:
                        subprocess.run([sys.executable,'scripts/analyze_joint_gait.py','--rollout',str(output/'rollout.npz'),'--output',str(gait_path)],
                                       cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
                    gait=json.loads(gait_path.read_text())
                    results=json.loads((output/'summary.json').read_text())['results']
                    score=sum(float(row['survived']) for row in results)/len(results)
                    error_key='root_error_m' if interface=='motion' else 'velocity_error_m_s'
                    errors=[row[error_key] for row in results if row[error_key] is not None]
                    score-=.1*(sum(errors)/len(errors) if errors else 10.)
                    best=args.training/('best_'+interface+'.json')
                    qualified=gait['moving_command_walking_accepted']
                    if not qualified:
                        (destination/(interface+'-rejected-gait.json')).write_text(json.dumps(dict(iteration=iteration,task_score=score,walking_qualified=False,
                            reason='Legacy clearance/sole-center heuristic requires contact-level review; low clearance is not alone a walking failure'),indent=2))
                        if interface=='velocity':gait_rejections+=1
                        continue
                    if interface=='velocity':gait_rejections=0
                    if not best.exists() or not json.loads(best.read_text()).get('walking_qualified',False) or score>json.loads(best.read_text())['score']:
                        target=args.training/('best_'+interface+'.pt');temporary=target.with_suffix('.pt.tmp')
                        shutil.copy2(checkpoint,temporary);temporary.replace(target)
                        best.write_text(json.dumps(dict(iteration=iteration,score=score,audit=str(output),walking_qualified=True),indent=2))
                done.add(iteration);last=time.monotonic()
                state.write_text(json.dumps(dict(evaluated_iterations=sorted(done),latest_iteration=iteration,latest_audit=str(destination),consecutive_velocity_gait_rejections=gait_rejections),indent=2))
                print('AUDITED',iteration,flush=True)
                if iteration>=5000 and gait_rejections>=3:
                    stopped=False
                    (args.training/'gait_stall.json').write_text(json.dumps(dict(iteration=iteration,consecutive_rejections=gait_rejections,training_stop_requested=stopped,
                        reason='Repeated legacy heuristic failures need contact-level review. Automatic stopping based on2cm clearance has been disabled.'),indent=2))
        time.sleep(30)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--training',type=Path,required=True);parser.add_argument('--motion',type=Path,required=True)
    parser.add_argument('--interval-seconds',type=int,default=3600);parser.add_argument('--max-seconds',type=int,default=172800)
    parser.add_argument('--training-pid',type=int);main(parser.parse_args())
