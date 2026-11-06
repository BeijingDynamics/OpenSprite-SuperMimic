"""Render actual recorded qpos with command/failure HUD; no reference substitution."""
import argparse,sys,json
from pathlib import Path
import numpy as np,mujoco,imageio.v2 as imageio
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ROBOT_XML,BODY_JOINT_NAMES

def render(args):
    with np.load(args.rollout,allow_pickle=False) as archive:
        data={key:archive[key] for key in ['qpos','commands','failed','velocity','yaw_rate','case_names'] if key in archive}
    spec=mujoco.MjSpec.from_file(str(ROBOT_XML))
    spec.worldbody.add_geom(name='floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[0,0,.05],rgba=[.28,.3,.32,1])
    model=spec.compile();state=mujoco.MjData(model)
    ids=[model.joint(n).qposadr[0] for n in BODY_JOINT_NAMES]
    renderer=mujoco.Renderer(model,height=360,width=480)
    camera=mujoco.MjvCamera();camera.distance=2.2;camera.azimuth=135;camera.elevation=-20
    selected=args.envs;args.output.parent.mkdir(parents=True,exist_ok=True)
    writer=imageio.get_writer(str(args.output),fps=50,codec='libx264',quality=7,macro_block_size=1)
    try:
        frames=len(data['qpos']) if args.seconds is None else min(len(data['qpos']),round(args.seconds*50))
        for t in range(frames):
            panels=[]
            for env in selected:
                recorded=data['qpos'][t,env]
                state.qpos[:7]=recorded[:7];state.qpos[ids]=recorded[7:]
                mujoco.mj_forward(model,state)
                camera.lookat[:]=recorded[:3];camera.lookat[2]=.52
                renderer.update_scene(state,camera=camera)
                # Fixed world grid (25cm cells) makes real translation visible
                # even though the camera follows the robot. Visualization only.
                scn=renderer.scene;step=.25;radius=1.5
                x0=np.floor((recorded[0]-radius)/step)*step;y0=np.floor((recorded[1]-radius)/step)*step
                for k in range(14):
                    for first,last in [([x0+k*step,y0,.002],[x0+k*step,y0+13*step,.002]),
                                       ([x0,y0+k*step,.002],[x0+13*step,y0+k*step,.002])]:
                        geom=scn.geoms[scn.ngeom]
                        mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.38,.42,.46,1.]))
                        mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_LINE,1.,np.array(first),np.array(last));scn.ngeom+=1
                image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
                cmd=data['commands'][t,env]
                name=str(data['case_names'][env]) if 'case_names' in data else f'env{env}'
                label=f'{name} t={t/50:.2f}s '+('FAILED / physics continues' if data['failed'][t,env] else 'alive')
                draw.rectangle((0,0,480,58),fill='black');draw.text((8,3),label,fill='white')
                draw.text((8,20),f'CMD  vx={cmd[0]:+.2f} vy={cmd[1]:+.2f} yaw={cmd[2]:+.2f}',fill='white')
                if 'velocity' in data:
                    measured=data['velocity'][t,env];yaw=data['yaw_rate'][t,env]
                    draw.text((8,37),f'MEAS vx={measured[0]:+.2f} vy={measured[1]:+.2f} yaw={yaw:+.2f}',fill='#7fffd4')
                panels.append(np.asarray(image))
            writer.append_data(np.concatenate((np.concatenate(panels[:2],1),np.concatenate(panels[2:],1)),0))
    finally:writer.close();renderer.close()
    print(json.dumps({'video':str(args.output),'frames':frames,'fps':50,'env_indices':selected}))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--rollout',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--envs',type=int,nargs=4,default=[0,1,3,4]);p.add_argument('--seconds',type=float);render(p.parse_args())
