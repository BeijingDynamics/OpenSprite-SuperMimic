"""Contact-focused native v5 motion previews for inspection before policy training."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
import argparse,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,mujoco
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
def render(bank):
    d=np.load(bank,allow_pickle=False)
    spec=mujoco.MjSpec.from_file(str(ROOT/'assets/opensprite/opensprite.xml'))
    spec.worldbody.add_geom(name='floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[0,0,.1],rgba=[.32,.34,.36,1])
    m=spec.compile();state=mujoco.MjData(m);renderer=mujoco.Renderer(m,height=300,width=320)
    names=d['joint_names'].astype(str).tolist();qids=[int(m.joint(n).qposadr[0]) for n in names]
    roots=d['body_names'].astype(str).tolist().index('pelvis_link')
    feet=[m.geom(s+'_foot_collision').id for s in ['left','right']]
    camera=mujoco.MjvCamera();camera.distance=1.9;camera.azimuth=130;camera.elevation=-15
    out=bank.parent/'previews';out.mkdir(exist_ok=True)
    manifest=[]
    try:
        for name,start,length in zip(d['motion_names'].astype(str),d['motion_starts'],d['motion_lengths']):
            ids=np.arange(start,start+length);heights=[]
            for f in ids:
                state.qpos[:3]=d['body_pos_w'][f,roots];state.qpos[3:7]=d['body_quat_w'][f,roots];state.qpos[qids]=d['joint_pos'][f]
                mujoco.mj_forward(m,state)
                heights.append([state.geom_xpos[g,2]-abs(state.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g] for g in feet])
            heights=np.array(heights)
            chosen=list(np.linspace(0,length-1,8).astype(int))
            for side in range(2):chosen.extend(np.argsort(heights[:,side])[-2:].tolist())
            chosen=sorted(chosen)[:12]
            sheet=Image.new('RGB',(4*320,3*300),'white')
            for slot,local in enumerate(chosen):
                f=int(start+local)
                state.qpos[:3]=d['body_pos_w'][f,roots];state.qpos[3:7]=d['body_quat_w'][f,roots];state.qpos[qids]=d['joint_pos'][f]
                mujoco.mj_forward(m,state);camera.lookat[:]=state.qpos[:3];camera.lookat[2]=.47
                renderer.update_scene(state,camera=camera)
                image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
                draw.rectangle((0,0,320,35),fill='black')
                draw.text((5,4),f'{name} t={local/50:.2f}s',fill='white')
                draw.text((5,18),f'sole L/R={heights[local,0]:.3f}/{heights[local,1]:.3f} m',fill='white')
                sheet.paste(image,((slot%4)*320,(slot//4)*300))
            path=out/(name+'.png');sheet.save(path);manifest.append(dict(name=name,path=str(path),local_frames=[int(x) for x in chosen]))
    finally:renderer.close()
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest),flush=True)
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--motion',type=Path,required=True);render(p.parse_args().motion)
