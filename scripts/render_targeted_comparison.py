"""Render saved physical qpos locally; no new simulation or training GPU use."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ROBOT_XML,BODY_JOINT_NAMES

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--preview-only',action='store_true');parser.add_argument('--iteration',type=int,default=600)
    parser.add_argument('--include-forward025',action='store_true');args=parser.parse_args()
    base=ROOT/'outputs/targeted-upright-clearance-20261118/evidence'
    if hashlib.sha256(ROBOT_XML.read_bytes()).hexdigest()!='2cdfb00df9867076271d49dd699c95e2a8ec2f4c6d88be25d5c9ccc6d73fd9e1':
        raise ValueError('Local rendering asset differs from the verified training asset')
    arms=['control','upright','upright-clearance'];labels=['Original reward','Upright only','Upright + clearance']
    cases=['zero','forward015','forward025','right_curve'] if args.include_forward025 else ['zero','forward015','right_curve']
    stem=f'three-arms-{args.iteration}-'+('four-cases' if args.include_forward025 else 'focus')
    height=360*len(cases);traces=[];indices=[];hashes={}
    for arm in arms:
        folder=base/f'eval-{arm}-{args.iteration}'
        with np.load(folder/'rollout.npz') as z:traces.append({k:z[k] for k in ['qpos','commands','failed','velocity','yaw_rate','case_names']})
        names=list(traces[-1]['case_names']);indices.append([names.index(n) for n in cases])
        hashes[arm]=json.loads((folder/'summary.json').read_text())['checkpoint_sha256']
    spec=mujoco.MjSpec.from_file(str(ROBOT_XML));spec.worldbody.add_geom(name='review_floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[0,0,.05],rgba=[.55,.57,.6,1])
    model=spec.compile();data=mujoco.MjData(model);model.vis.headlight.active=1
    model.vis.headlight.ambient[:]=.45;model.vis.headlight.diffuse[:]=.8
    ids=[int(model.joint(n).qposadr[0]) for n in BODY_JOINT_NAMES]
    renderer=mujoco.Renderer(model,width=480,height=360)
    camera=mujoco.MjvCamera();camera.distance=1.7;camera.azimuth=135;camera.elevation=-20
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',16)
    def frame(t):
        canvas=Image.new('RGB',(1440,height))
        for row,case in enumerate(cases):
            for col,trace in enumerate(traces):
                env=indices[col][row];q=trace['qpos'][t,env]
                data.qpos[:7]=q[:7];data.qpos[ids]=q[7:];mujoco.mj_forward(model,data)
                camera.lookat[:]=q[:3];camera.lookat[2]=.52;renderer.update_scene(data,camera=camera)
                scene=renderer.scene;x0=np.floor((q[0]-1.5)/.25)*.25;y0=np.floor((q[1]-1.5)/.25)*.25
                for k in range(14):
                    for start,end in [([x0+k*.25,y0,.002],[x0+k*.25,y0+3.25,.002]),([x0,y0+k*.25,.002],[x0+3.25,y0+k*.25,.002])]:
                        geom=scene.geoms[scene.ngeom];mujoco.mjv_initGeom(geom,mujoco.mjtGeom.mjGEOM_LINE,np.zeros(3),np.zeros(3),np.eye(3).ravel(),np.array([.12,.15,.18,1]))
                        mujoco.mjv_connector(geom,mujoco.mjtGeom.mjGEOM_LINE,1.,np.array(start),np.array(end));scene.ngeom+=1
                panel=Image.fromarray(renderer.render());draw=ImageDraw.Draw(panel);draw.rectangle((0,0,480,72),fill='black')
                cmd=trace['commands'][t,env];v=trace['velocity'][t,env,0];yaw=trace['yaw_rate'][t,env]
                draw.text((7,2),f'{labels[col]} | update {args.iteration} | {case}',font=font,fill='white')
                draw.text((7,25),f't={t/50:04.1f}s  CMD vx={cmd[0]:+.2f}, yaw={cmd[2]:+.2f}',font=font,fill='white')
                draw.text((7,48),f'MEAS vx={v:+.2f}, yaw={yaw:+.2f}  '+('FAILED' if trace['failed'][t,env] else 'alive'),font=font,fill='#7fffd4')
                canvas.paste(panel,(480*col,360*row))
        return np.asarray(canvas)
    try:
        Image.fromarray(frame(500)).save(base/f'{stem}-preview.png')
        if args.preview_only:return
        output=base/f'{stem}.mp4';ffmpeg=shutil.which('ffmpeg')
        if not ffmpeg:raise RuntimeError('ffmpeg is required')
        if output.exists():raise RuntimeError('Refusing to overwrite prior comparison video')
        log=base/f'{stem}-render.log'
        with log.open('w') as stream:
            proc=subprocess.Popen([ffmpeg,'-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'1440x{height}','-r','25','-i','pipe:0',
                '-an','-c:v','libx264','-preset','fast','-crf','21','-threads','2','-pix_fmt','yuv420p','-movflags','+faststart',str(output)],stdin=subprocess.PIPE,stderr=stream)
            try:
                for t in range(0,min(len(z['qpos']) for z in traces),2):proc.stdin.write(frame(t).tobytes())
            finally:proc.stdin.close()
            if proc.wait()!=0:raise RuntimeError('Video encoding failed; inspect render log')
        (base/f'{stem}-manifest.json').write_text(json.dumps({'columns':arms,'rows':cases,'checkpoint_sha256':hashes,
            'playback_fps':25,'physical_source_fps':50,'frame_stride':2,'playback_speed':1,'physics_recomputed':False,'joint_names':list(BODY_JOINT_NAMES),
            'note':'Recorded qpos rendered with tracking camera and world grid. All commands start after the same two-second zero period.'},indent=2)+'\n')
        print('Saved',output)
    finally:renderer.close()

if __name__=='__main__':main()
