"""Test the frozen policy's command envelope, then show live command changes."""
import json,os,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/speed-envelope-001'
CHECKPOINT=ROOT/'outputs/joint-paper-v5-001/model_026951.pt'
MOTION=ROOT/'data/motions/contact_v5_v11/v5_locomotion.npz'

def main():
    OUT.mkdir(exist_ok=True);cases=[]
    def add(name,command,category):cases.append(dict(name=name,command=command,category=category))
    for vx in [0,.15,.3,.45,.6,.75,.9,1.2]:add(f'fwd_{vx:g}',[vx,0,0],'forward')
    for vx in [-.15,-.3,-.45]:add(f'back_{vx:g}',[vx,0,0],'backward')
    for vy in [-.3,-.2,-.1,.1,.2,.3]:add(f'side_{vy:g}',[0,vy,0],'lateral')
    for yaw in [-.9,-.6,-.3,.3,.6,.9]:add(f'spin_{yaw:g}',[0,0,yaw],'turn_in_place')
    for vx in [.3,.6,.9]:
        for yaw in [-.9,-.6,-.3,.3,.6,.9]:add(f'curve_{vx:g}_{yaw:g}',[vx,0,yaw],'curved_forward')
    for vx in [.3,.6]:
        for vy in [-.2,.2]:
            add(f'diag_{vx:g}_{vy:g}',[vx,vy,0],'diagonal')
            add(f'combo_{vx:g}_{vy:g}',[vx,vy,.3 if vy>0 else -.3],'combined')
    path=OUT/'commands.json';path.write_text(json.dumps(cases,indent=2))
    env={**os.environ,'MUJOCO_GL':'egl','PYTHONUNBUFFERED':'1'}
    nvrtc=ROOT/'.venv/lib/python3.12/site-packages/nvidia/cuda_nvrtc/lib'
    env['LD_LIBRARY_PATH']=str(nvrtc)+':'+env.get('LD_LIBRARY_PATH','')
    common=[sys.executable,'scripts/evaluate_joint.py','--checkpoint',str(CHECKPOINT),'--motion',str(MOTION),'--interface','velocity','--initial-pose','urdf-zero']
    for seed in [101,202,303]:
        output=OUT/f'seed-{seed}'
        if not (output/'summary.json').exists():
            with (OUT/f'seed-{seed}.log').open('w') as log:
                subprocess.run([*common,'--commands-file',str(path),'--seed',str(seed),'--seconds','30','--output',str(output)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        result=json.loads((output/'summary.json').read_text());print('SPEED_GRID',seed,sum(row['survived'] for row in result['results']),len(cases),flush=True)
    schedules=[]
    sequences=[
        ('speed_steps',[[.15,0,0],[.3,0,0],[.45,0,0],[.6,0,0],[.3,0,0],[0,0,0]]),
        ('left_right_turns',[[.3,0,0],[.3,0,.3],[.3,0,.6],[.3,0,-.3],[.3,0,-.6],[0,0,0]]),
        ('lateral_diagonal',[[0,.1,0],[0,.2,0],[.3,.2,0],[0,-.2,0],[.3,-.2,0],[0,0,0]]),
        ('extrap_speed',[[.3,0,0],[.6,0,0],[.75,0,0],[.9,0,0],[1.2,0,0],[0,0,0]])]
    for name,commands in sequences:
        schedules.append(dict(name=name,command=[0,0,0],category='live_command_changes',stages=[dict(at_seconds=2+5*i,command=command) for i,command in enumerate(commands)]))
    schedule_path=OUT/'schedules.json';schedule_path.write_text(json.dumps(schedules,indent=2))
    output=OUT/'live-command-changes'
    if not (output/'summary.json').exists():
        with (OUT/'live-command-changes.log').open('w') as log:
            subprocess.run([*common,'--commands-file',str(schedule_path),'--seed','101','--seconds','32','--output',str(output)],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    names=[c['name'] for c in cases]
    videos=[('forward-speeds',[names.index(f'fwd_{v:g}') for v in [.3,.6,.9,1.2]],OUT/'seed-101/rollout.npz'),
            ('turning-speeds',[names.index(f'curve_{v:g}_{w:g}') for v,w in [(.3,.6),(.6,.6),(.3,-.6),(.6,-.6)]],OUT/'seed-101/rollout.npz'),
            ('side-and-back',[names.index(n) for n in ['side_0.2','side_0.3','back_-0.3','diag_0.6_0.2']],OUT/'seed-101/rollout.npz'),
            ('live-command-changes',[0,1,2,3],output/'rollout.npz')]
    for name,selected,rollout in videos:
        video=OUT/(name+'.mp4')
        if video.exists():continue
        with (OUT/(name+'-render.log')).open('w') as log:
            subprocess.run([sys.executable,'scripts/render_velocity.py','--rollout',str(rollout),'--output',str(video),'--envs',*[str(i) for i in selected]],cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
        print('VIDEO',video,flush=True)
    print('VELOCITY_ENVELOPE_COMPLETE',len(cases),'cases x3seeds plus4live sequences',flush=True)

if __name__=='__main__':main()
