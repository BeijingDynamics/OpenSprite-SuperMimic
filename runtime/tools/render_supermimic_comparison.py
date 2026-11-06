"""Render recorded closed-loop poses; protection trips are labelled explicitly."""
import argparse
from pathlib import Path
import json
import numpy as np
import mujoco
import imageio.v2 as imageio
from PIL import Image,ImageDraw


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--contract',type=Path,required=True)
    p.add_argument('--rollouts',type=Path,nargs=4,required=True);p.add_argument('--names',nargs=4,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--seconds',type=int,default=10)
    a=p.parse_args();c=json.loads(a.contract.read_text());m=mujoco.MjModel.from_binary_path(str(a.scene));d=mujoco.MjData(m)
    qi=[m.jnt_qposadr[next(i for i in range(m.njnt) if m.joint(i).name.rsplit('/',1)[-1]==n)] for n in c['joint_names']]
    data=[]
    for path in a.rollouts:
        with np.load(path,allow_pickle=False) as f:data.append({k:f[k] for k in ('qpos','commands','velocity','yaw_rate','failed')})
    renderer=mujoco.Renderer(m,height=360,width=480);camera=mujoco.MjvCamera();camera.distance=2.2;camera.azimuth=135;camera.elevation=-20
    a.output.parent.mkdir(parents=True,exist_ok=True)
    frames=min(a.seconds*50,min(len(x['qpos']) for x in data))
    with imageio.get_writer(str(a.output),fps=50,codec='libx264',quality=7,macro_block_size=1) as writer:
        for t in range(frames):
            panels=[]
            for i,x in enumerate(data):
                pose=x['qpos'][t,0];d.qpos[:7]=pose[:7];d.qpos[qi]=pose[7:];mujoco.mj_forward(m,d)
                camera.lookat[:]=pose[:3];camera.lookat[2]=.52;renderer.update_scene(d,camera=camera)
                scene=renderer.scene;start=np.floor((pose[:2]-1.5)/.25)*.25
                for k in range(14):
                    for v1,v2 in (([start[0]+k*.25,start[1],.002],[start[0]+k*.25,start[1]+3.25,.002]),([start[0],start[1]+k*.25,.002],[start[0]+3.25,start[1]+k*.25,.002])):
                        g=scene.geoms[scene.ngeom];mujoco.mjv_initGeom(g,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.38,.42,.46,1]))
                        mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_LINE,1.,np.array(v1),np.array(v2));scene.ngeom+=1
                panel=Image.fromarray(renderer.render());draw=ImageDraw.Draw(panel);cmd=x['commands'][t,0];v=x['velocity'][t,0]
                draw.rectangle((0,0,480,76),fill='black');draw.text((8,3),a.names[i]+f' t={t/50:.2f}s',fill='white')
                draw.text((8,21),f'CMD vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} yaw={cmd[2]:+.2f}',fill='white')
                draw.text((8,39),f'MEAS vx={v[0]:+.2f} vy={v[1]:+.2f} yaw={x["yaw_rate"][t,0]:+.2f}',fill='#7fffd4')
                draw.text((8,57),'PROTECTION TRIP: torque=0, passive physics' if x['failed'][t,0] else 'controller active (simulation)',fill='#ff9c80' if x['failed'][t,0] else 'white')
                panels.append(np.asarray(panel))
            writer.append_data(np.concatenate([np.concatenate(panels[:2],axis=1),np.concatenate(panels[2:],axis=1)],axis=0))
    renderer.close();print(json.dumps({'video':str(a.output),'frames':frames,'fps':50}))


if __name__=='__main__':main()
