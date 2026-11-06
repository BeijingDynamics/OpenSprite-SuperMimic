import sys,json
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from refine_reference_contacts import source_targets
from scipy.spatial.transform import Rotation

if __name__=='__main__':
    m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/opensprite.xml'));d=mujoco.MjData(m)
    for name in ['v5_straight','v5_left','v5_right']:
        prior=np.load(ROOT/'data/motions/native_v5_v3'/(name+'.npz'))['qpos']
        targets,info,adapted=source_targets(ROOT/'data/sources/lafan1-resolved'/(name+'.bvh'),prior,ROOT/'outputs/umr-v5-v3'/name/'native.npz')
        print(json.dumps(info),flush=True)
        rootR=Rotation.from_quat(adapted[:,[4,5,6,3]]).as_matrix()
        yaw=np.unwrap(np.arctan2(rootR[:,1,0],rootR[:,0,0]))
        print(json.dumps(dict(name=name,source_initial_yaw=float(yaw[0]),source_final_yaw=float(yaw[-1]),source_foot_initial_yaw=float(targets[0][1][0]),actual_source_yaw_change_rad=float(yaw[-1]-yaw[0]),maximum_source_tilt_rad=float(np.arccos(np.clip(rootR[:,2,2],-1,1)).max()))),flush=True)
        rows=[]
        for frame in [0,100,200,300]:
            d.qpos[:]=prior[frame];mujoco.mj_forward(m,d)
            root=Rotation.from_quat(prior[frame,[4,5,6,3]]).as_matrix()
            row=dict(frame=frame,root_yaw=float(np.arctan2(root[1,0],root[0,0])),root_tilt=float(np.arccos(np.clip(root[2,2],-1,1))),feet=[])
            for side,prefix in enumerate(['left','right']):
                g=m.geom(prefix+'_foot_collision').id;R=d.geom_xmat[g].reshape(3,3)
                row['feet'].append(dict(side=prefix,native_pos=d.geom_xpos[g].tolist(),target_pos=targets[side][0][frame].tolist(),
                    native_yaw=float(np.arctan2(R[1,0],R[0,0])),target_yaw=float(targets[side][1][frame]),native_up=R[:,2].tolist()))
            rows.append(row)
        print(json.dumps(dict(motion=name,rows=rows)),flush=True)
