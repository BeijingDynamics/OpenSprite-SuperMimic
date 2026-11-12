"""Render front and side views of a reference, explicitly without a policy."""
import argparse
import os
from pathlib import Path
import sys
os.environ.setdefault('MUJOCO_GL','egl')
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ROBOT_XML


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--motion',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--speed',type=float,default=1.)
    args=parser.parse_args()
    if not 0<args.speed<=2:parser.error('Playback speed must be in (0,2]')
    with np.load(args.motion,allow_pickle=False) as data:poses=data['qpos'];fps=float(data['fps'])
    spec=mujoco.MjSpec.from_file(str(ROBOT_XML))
    spec.worldbody.add_geom(name='review_floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[0,0,.1],rgba=[.35,.39,.43,1])
    spec.worldbody.add_light(pos=[0,0,3],dir=[0,0,-1],type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    model=spec.compile();model.vis.global_.offwidth=512;model.vis.global_.offheight=480
    # Cover the entire translated reference with the directional shadow map.
    # Otherwise later frames can look airborne after leaving its initial bounds.
    travel=float(np.linalg.norm(poses[:,:2]-poses[0,:2],axis=-1).max())
    model.stat.extent=max(float(model.stat.extent),travel+1.5)
    model.vis.quality.shadowsize=4096
    model.vis.headlight.ambient[:]=.45;state=mujoco.MjData(model)
    state.qpos[:]=poses[0];mujoco.mj_forward(model,state)
    R=state.xmat[model.body('pelvis_link').id].reshape(3,3)
    initial_yaw=np.rad2deg(np.arctan2(R[1,0],R[0,0]))
    cameras=[]
    for offset in (0,90):
        camera=mujoco.MjvCamera();camera.azimuth=initial_yaw+offset;camera.elevation=-12;camera.distance=1.4;cameras.append(camera)
    try:font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
    except OSError:font=ImageFont.load_default()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with mujoco.Renderer(model,height=480,width=512) as renderer:
        with imageio.get_writer(args.output,fps=fps/2*args.speed,codec='libx264',quality=8,macro_block_size=1) as writer:
            for frame in range(0,len(poses),2):
                state.qpos[:]=poses[frame];mujoco.mj_forward(model,state)
                composite=Image.new('RGB',(1024,520),'#151b24')
                for index,camera in enumerate(cameras):
                    camera.lookat[:]=[*state.qpos[:2],.40]
                    renderer.update_scene(state,camera=camera)
                    composite.paste(Image.fromarray(renderer.render()),(512*index,40))
                draw=ImageDraw.Draw(composite)
                draw.text((12,9),f'Front | Reference playback, no policy | {frame/fps:.2f}s',font=font,fill='white')
                draw.text((525,9),f'Side | Open Sprite v5 | {args.speed:g}x',font=font,fill='white')
                writer.append_data(np.asarray(composite))
                if frame==len(poses)//2//2*2:composite.save(args.output.with_suffix('.png'))
    print('Rendered reference views:',args.output,flush=True)


if __name__=='__main__':main()
