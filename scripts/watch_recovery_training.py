"""Bounded checkpoint evaluation for one PM01 recovery-training job."""
import argparse,json,os,shutil,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser();p.add_argument('--training',type=Path,required=True)
    p.add_argument('--motion',type=Path,required=True);p.add_argument('--training-pid',type=int,required=True)
    p.add_argument('--max-seconds',type=int,default=46800);p.add_argument('--interval-seconds',type=int,default=3600)
    a=p.parse_args();started=time.monotonic();last=0;done=set();env={**os.environ,'MUJOCO_GL':'egl'}
    def run(cmd,log):
        with log.open('w') as stream:subprocess.run([sys.executable,*cmd],cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,check=True,timeout=1800)
    while time.monotonic()-started<a.max_seconds:
        proc=Path('/proc',str(a.training_pid),'cmdline')
        alive=proc.exists() and str(a.training).encode() in proc.read_bytes()
        checkpoints=sorted(a.training.glob('model_*.pt'))
        if checkpoints:
            checkpoint=checkpoints[-1];iteration=int(checkpoint.stem.split('_')[-1])
            due=iteration not in done and (not alive or (iteration>=1000 and (not done or time.monotonic()-last>=a.interval_seconds)))
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
                    done.add(iteration);last=time.monotonic()
                    (a.training/'recovery_evaluation_status.json').write_text(json.dumps(dict(evaluated_iterations=sorted(done),latest_audit=str(audit)),indent=2)+'\n')
                    print('RECOVERY_AUDIT_COMPLETE',iteration,flush=True)
                except Exception as exc:
                    (audit/'evaluation_error.json').write_text(json.dumps(dict(error=str(exc)),indent=2)+'\n')
                    print('RECOVERY_AUDIT_ERROR',iteration,str(exc),flush=True)
                    if not alive:break
                    time.sleep(30)
        if not alive:break
        time.sleep(30)
    print('RECOVERY_WATCH_FINISHED',flush=True)


if __name__=='__main__':main()
