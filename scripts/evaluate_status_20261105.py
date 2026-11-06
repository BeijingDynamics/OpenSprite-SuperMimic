"""One user-triggered intermediate physical evaluation; no training changes."""
from pathlib import Path
import os,subprocess,sys,json,datetime
ROOT=Path(__file__).resolve().parents[1]
os.chdir(ROOT)
os.environ['MUJOCO_GL']='egl'
os.environ['LD_LIBRARY_PATH']=str(ROOT/'.venv/lib/python3.12/site-packages/nvidia/cuda_nvrtc/lib')
training=ROOT/'outputs/joint-waist-v5-002'
checkpoint=sorted(training.glob('model_*.pt'))[-1]
dest=training/('status-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
dest.mkdir(exist_ok=False)
(ROOT/'status_evaluation_latest.json').write_text(json.dumps({'output':str(dest),'checkpoint':str(checkpoint),'state':'running'},indent=2))
for interface in ('velocity','motion'):
    output=dest/interface
    command=[sys.executable,'scripts/evaluate_joint.py','--checkpoint',str(checkpoint),
        '--motion','data/motions/contact_v5_v11/v5_locomotion.npz','--interface',interface,
        '--output',str(output),'--seconds','30','--seed','101']
    if interface=='velocity':command+=['--initial-pose','urdf-zero']
    with (dest/(interface+'.log')).open('w') as log:
        subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True)
    with (dest/(interface+'-render.log')).open('w') as log:
        subprocess.run([sys.executable,'scripts/render_velocity.py','--rollout',str(output/'rollout.npz'),
            '--output',str(output/'evaluation.mp4'),'--envs',*(['0','1','3','4'] if interface=='velocity' else ['0','1','2','4'])],
            stdout=log,stderr=subprocess.STDOUT,check=True)
(ROOT/'status_evaluation_latest.json').write_text(json.dumps({'output':str(dest),'checkpoint':str(checkpoint),'state':'complete'},indent=2))
print('STATUS_EVALUATION_COMPLETE',dest,flush=True)
