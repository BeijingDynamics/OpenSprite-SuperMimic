"""Apply the upstream waist inertial block to the preserved training scene only."""
import argparse
import json
from pathlib import Path
import numpy as np
import mujoco
from open_sprite_runtime.imu import matrix_to_quaternion_wxyz,quaternion_wxyz_to_matrix
from open_sprite_runtime.supermimic_replay import file_sha256


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scene',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    values=json.loads(a.manifest.read_text());assert values['changed_link']=='waist_yaw_link'
    m=mujoco.MjModel.from_binary_path(str(a.scene));body=next(i for i in range(m.nbody) if m.body(i).name.rsplit('/',1)[-1]=='waist_yaw_link')
    old={'mass':float(m.body_mass[body]),'com':m.body_ipos[body].tolist(),'inertia':m.body_inertia[body].tolist(),'iquat':m.body_iquat[body].tolist()}
    v=values['inertia_kg_m2'];inertia=np.array([[v['ixx'],v['ixy'],v['ixz']],[v['ixy'],v['iyy'],v['iyz']],[v['ixz'],v['iyz'],v['izz']]])
    principal,rotation=np.linalg.eigh(inertia)
    if np.linalg.det(rotation)<0:rotation[:,0]*=-1
    assert np.min(principal)>0
    m.body_mass[body]=values['mass_kg'];m.body_ipos[body]=values['com_m'];m.body_inertia[body]=principal
    m.body_iquat[body]=matrix_to_quaternion_wxyz(rotation)
    mujoco.mj_setConst(m,mujoco.MjData(m));a.output.parent.mkdir(parents=True,exist_ok=True)
    mujoco.mj_saveModel(m,str(a.output));r=quaternion_wxyz_to_matrix(tuple(m.body_iquat[body]))
    error=float(np.max(abs(r@np.diag(m.body_inertia[body])@r.T-inertia)));assert error<1e-12
    report={'old':old,'new':values,'scene_before_sha256':file_sha256(a.scene),'scene_after_sha256':file_sha256(a.output),
        'manifest_sha256':file_sha256(a.manifest),'reconstructed_inertia_max_error':error,
        'scope':'Only waist_yaw_link mass/COM/full inertia replaced; other training-scene geometry/armature preserved; no hardware qualification.'}
    a.output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'output':str(a.output),'inertia_error':error}))


if __name__=='__main__':main()
