"""Longer simulation command transitions and force probes for the preserved candidate."""
import json,os,subprocess,sys,time,traceback
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'outputs/final-readiness-20261119'
CK=ROOT/'outputs/reference-fusion-campaign/train-walk-stand80-6000/model_006000.pt'
MOTION=ROOT/'reference/walk_stand.npz'

def main():
    OUT.mkdir(exist_ok=False);state=dict(phase='preparing',started_unix=time.time(),hardware_access=False,completed=[])
    def save(p):
        state.update(phase=p,updated_unix=time.time());(OUT/'status.json').write_text(json.dumps(state,indent=2)+'\n')
    def run(label,args):
        save(label)
        with (OUT/(label+'.log')).open('x') as f:
            subprocess.run([sys.executable,*map(str,args)],cwd=ROOT,env={**os.environ,'MUJOCO_GL':'egl','OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1'},stdout=f,stderr=subprocess.STDOUT,check=True,timeout=900)
        state['completed'].append(label)
    cases=[dict(name='joystick_sequence',command=[0,0,0],stages=[dict(at_seconds=t,command=c) for t,c in [(2,[0,0,0]),(5,[.25,0,0]),(15,[.5,0,0]),(25,[.35,0,-.2]),(35,[.35,0,.2]),(45,[0,0,0])]]),
        dict(name='long_forward_stop',command=[.5,0,0],stages=[dict(at_seconds=2,command=[.5,0,0]),dict(at_seconds=45,command=[0,0,0])]),
        dict(name='standing_sustained_push',command=[0,0,0],forces=[dict(at_seconds=12,duration_seconds=1,force_world_n=[10,0,0]),dict(at_seconds=35,duration_seconds=1,force_world_n=[-10,0,0])]),
        dict(name='walking_sustained_lateral',command=[.5,0,0],forces=[dict(at_seconds=12,duration_seconds=1,force_world_n=[0,10,0]),dict(at_seconds=35,duration_seconds=1,force_world_n=[0,-10,0])])]
    commands=OUT/'commands.json';commands.write_text(json.dumps(cases,indent=2)+'\n')
    try:
        run('evaluate',['scripts/evaluate_joint.py','--checkpoint',CK,'--motion',MOTION,'--interface','velocity','--seconds','60','--seed','404','--initial-pose','controller-default','--commands-file',commands,'--output',OUT/'eval'])
        run('posture',['scripts/score_posture_width.py',OUT/'eval'])
        run('geometry',['scripts/summarize_locomotion_geometry.py','--evaluation',OUT/'eval','--output',OUT/'eval/locomotion-geometry.json'])
        run('video',['scripts/render_velocity.py','--rollout',OUT/'eval/rollout.npz','--output',OUT/'readiness.mp4','--envs','0','1','2','3','--bright','--camera-distance','1.7','--camera-azimuth','135'])
        save('complete')
    except Exception:state['error']=traceback.format_exc();save('failed');raise

if __name__=='__main__':main()
