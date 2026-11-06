"""Inspect a single contact-refined candidate before the full-bank QA gate."""
import argparse,json,sys
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from retarget.convert_umr import pack
from render_reference_preview import render

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--motion',type=Path,required=True);args=parser.parse_args()
    data=np.load(args.motion,allow_pickle=False)
    m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/opensprite.xml'));d=mujoco.MjData(m)
    feet=[m.geom(s+'_foot_collision').id for s in ['left','right']];positions=[];height=[]
    for pose in data['qpos']:
        d.qpos[:]=pose;mujoco.mj_forward(m,d);positions.append(d.geom_xpos[feet].copy())
        height.append([d.geom_xpos[g,2]-abs(d.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g] for g in feet])
    positions=np.array(positions);height=np.array(height);contact=height<.008;stable=contact[1:]&contact[:-1]
    speed=np.linalg.norm(np.diff(positions[...,:2],axis=0)*50,axis=-1);lift=height>.02
    report=dict(clip=str(args.motion),stance_foot_rms_speed_m_s=float(np.sqrt(np.mean(speed[stable]**2))) if stable.any() else None,
                lift_events=((~lift[:-1])&lift[1:]).sum(0).tolist(),maximum_lift_m=height.max(0).tolist())
    print(json.dumps(report),flush=True)
    if 'retarget_method' in data:
        output=args.motion.parent/'single-clip-review.npz';pack([args.motion],output);render(output)
