"""Bounded checkpoint evaluation for one PM01 recovery-training job."""
import argparse,json,os,shutil,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.checkpoints import checkpoint_milestones,evaluation_checkpoint


def main():
    p=argparse.ArgumentParser();p.add_argument('--training',type=Path,required=True)
    p.add_argument('--motion',type=Path,required=True);p.add_argument('--training-pid',type=int,required=True)
    p.add_argument('--max-seconds',type=int,default=46800);p.add_argument('--interval-seconds',type=int,default=3600)
    p.add_argument('--sole-audit',action='store_true',help='Audit reference, motion-interface and velocity-interface sole geometry')
    p.add_argument('--required-iterations',type=int,nargs='*',default=[],help='Evaluate these saved update counts before regular latest-checkpoint reviews')
    a=p.parse_args();required=checkpoint_milestones(a.required_iterations);started=time.monotonic();last=0;done=set();env={**os.environ,'MUJOCO_GL':'egl'}
    status_path=a.training/'recovery_evaluation_status.json'
    if status_path.exists():
        prior=json.loads(status_path.read_text());done=set(prior['evaluated_iterations'])
        last=started-max(0.,time.time()-status_path.stat().st_mtime)
    def run(cmd,log):
        with log.open('w') as stream:subprocess.run([sys.executable,*cmd],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=1800)
    while time.monotonic()-started<a.max_seconds:
        proc=Path('/proc',str(a.training_pid),'cmdline')
        alive=proc.exists() and str(a.training).encode() in proc.read_bytes()
        checkpoints=sorted(a.training.glob('model_*.pt'))
        checkpoint=evaluation_checkpoint(checkpoints,done,required,alive=alive,regular_due=not done or time.monotonic()-last>=a.interval_seconds)
        if checkpoint is not None:
            iteration=int(checkpoint.stem.split('_')[-1]);due=True
            if due:
                audit=a.training/f'recovery-audit-{iteration:06d}';audit.mkdir(exist_ok=True)
                frozen=audit/'checkpoint.pt'
                if not frozen.exists():shutil.copy2(checkpoint,frozen)
                try:
                    for seed in (101,202,303):
                        output=audit/f'velocity-seed{seed}'
                        if not (output/'summary.json').exists():
                            run(['scripts/evaluate_joint.py','--checkpoint',str(frozen),'--motion',str(a.motion),'--interface','velocity',
                                '--output',str(output),'--seconds','20','--seed',str(seed),'--commands-file','configs/velocity-recovery-evaluation.json'],audit/f'evaluate-{seed}.log')
                        run(['scripts/summarize_recovery.py','--evaluation',str(output)],audit/f'metrics-{seed}.log')
                    output=audit/'velocity-seed101'
                    run(['scripts/render_velocity.py','--rollout',str(output/'rollout.npz'),'--output',str(audit/'walking-recovery.mp4'),
                         '--envs','0','13','23','24','--seconds','20'],audit/'render.log')
                    if a.sole_audit:
                        motion_output=audit/'motion-seed101'
                        if not (motion_output/'summary.json').exists():
                            run(['scripts/evaluate_joint.py','--checkpoint',str(frozen),'--motion',str(a.motion),'--interface','motion',
                                 '--output',str(motion_output),'--seed','101'],audit/'evaluate-motion.log')
                        if not (audit/'sole-tracking-audit.json').exists():
                            run(['scripts/audit_sole_tracking.py','--motion',str(a.motion),
                                 '--motion-rollout',str(motion_output/'rollout.npz'),
                                 '--velocity-rollout',str(output/'rollout.npz'),
                                 '--output',str(audit/'sole-tracking-audit.json')],audit/'sole-audit.log')
                    done.add(iteration);last=time.monotonic()
                    status_path.write_text(json.dumps(dict(evaluated_iterations=sorted(done),required_iterations=sorted(required),latest_audit=str(audit),sole_audit_enabled=a.sole_audit),indent=2)+'\n')
                    print('RECOVERY_AUDIT_COMPLETE',iteration,flush=True)
                except Exception as exc:
                    (audit/'evaluation_error.json').write_text(json.dumps(dict(error=str(exc)),indent=2)+'\n')
                    print('RECOVERY_AUDIT_ERROR',iteration,str(exc),flush=True)
                    if not alive:break
                    time.sleep(30)
        if not alive:
            if evaluation_checkpoint(checkpoints,done,required,alive=False,regular_due=True) is None:break
            continue
        time.sleep(30)
    print('RECOVERY_WATCH_FINISHED',flush=True)


if __name__=='__main__':main()
