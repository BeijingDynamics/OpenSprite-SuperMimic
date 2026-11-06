"""Convert public Sprite joint walk into explicitly labelled v5 reset/style seeds.

Kinematic root paths are constructed, not dynamics-validated demonstrations.
"""
import sys,json,hashlib
from pathlib import Path
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import xml.etree.ElementTree as ET

def build():
    model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/opensprite.xml'))
    urdf=ET.parse(ROOT/'assets/opensprite/vendor/sprite0825_v5/sprite0825_float.urdf').getroot()
    names=[j.get('name') for j in urdf.findall('joint') if j.get('type')!='fixed']
    ids=[model.joint(n).id for n in names]
    qids=model.jnt_qposadr[ids]
    src=ROOT/'data/sources/01_native_walk.npz'
    with np.load(src,allow_pickle=False) as d:
        source_names=d['joint_names'].astype(str).tolist()
        walk=d['joint_pos'][:,[source_names.index(n) for n in names]]
    default=walk.mean(0)
    # Symmetric grounded bent-knee stance from public walk, not v4 zero pose.
    default[[i for i,n in enumerate(names) if 'waist_' in n]]=0
    qlo,qhi=model.jnt_range[ids].T
    default=np.clip(default,qlo+.001,qhi-.001)
    commands=[(0.,0.,0.),(.15,0.,0.),(.3,0.,0.),(.15,0.,.15),(.15,0.,-.15),(.3,0.,.15),(.3,0.,-.15)]
    fields={k:[] for k in ['joint_pos','joint_vel','body_pos_w','body_quat_w','body_lin_vel_w','body_ang_vel_w','commands']}
    body_names=[model.body(i).name for i in range(1,model.nbody)]
    feet=[model.geom('left_foot_collision').id,model.geom('right_foot_collision').id]
    N=1000;dt=.02
    for cmd in commands:
        q=np.repeat(default[None],N,axis=0) if cmd[0]==0 else walk[(np.arange(N)*2)%len(walk)].copy()
        q=np.clip(q,qlo+.001,qhi-.001)
        qd=np.gradient(q,dt,axis=0)
        pos=np.zeros((N,len(body_names),3));quat=np.zeros((N,len(body_names),4))
        data=mujoco.MjData(model);xy=np.zeros(2)
        for t in range(N):
            yaw=cmd[2]*t*dt
            data.qpos[:3]=[*xy,.5];data.qpos[3:7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
            data.qpos[qids]=q[t]
            mujoco.mj_forward(model,data)
            bottoms=[data.geom_xpos[g,2]-abs(data.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g] for g in feet]
            data.qpos[2]+=.002-min(bottoms)
            mujoco.mj_forward(model,data)
            pos[t]=data.xpos[1:];quat[t]=data.xquat[1:]
            xy+=dt*np.array([np.cos(yaw)*cmd[0],np.sin(yaw)*cmd[0]])
        vel=np.gradient(pos,dt,axis=0)
        # Orientation angular velocities use quaternion logarithmic differences.
        ang=np.zeros_like(pos)
        for t in range(N-1):
            for b in range(len(body_names)):
                conjugate=quat[t,b]*np.array([1,-1,-1,-1]);delta=np.zeros(4);axis=np.zeros(3)
                mujoco.mju_mulQuat(delta,quat[t+1,b],conjugate)
                if delta[0]<0:delta=-delta
                mujoco.mju_quat2Vel(axis,delta,dt);ang[t,b]=axis
        ang[-1]=ang[-2]
        for k,v in zip(fields,[q,qd,pos,quat,vel,ang,np.tile(cmd,(N,1))]):fields[k].append(v)
    result={k:np.concatenate(v).astype(np.float32) for k,v in fields.items()}
    result.update(fps=np.array(50.,np.float32),joint_names=np.array(names),body_names=np.array(body_names),
                  clip_starts=np.arange(len(commands))*N,clip_lengths=np.full(len(commands),N),
                  clip_names=np.array([f'command_vx{c[0]}_yaw{c[2]}' for c in commands]),
                  robot_model=np.array('opensprite0825-v5'),retarget_method=np.array('official_joint_walk_seed_v1'),
                  dataset_revision=np.array('sprite-v5-command-seed-20261104'))
    out=ROOT/'data/motions/v5_command_seed.npz';out.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(out,**result)
    (ROOT/'data/command_default_pose.json').write_text(json.dumps(dict(zip(names,default.tolist())),indent=2))
    print(json.dumps({'output':str(out),'frames':len(result['joint_pos']),'commands':commands,
                      'sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),
                      'limitations':'Kinematic initialization/style prior, not UMR or proven feasible tracking.'},indent=2))
if __name__=='__main__':build()
