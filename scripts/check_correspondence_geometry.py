"""Prove unchanged centered geometry before reusing surface correspondences."""
import argparse,json,hashlib
from pathlib import Path
import mujoco,numpy as np
ROOT=Path(__file__).resolve().parents[1]
def check(v4):
    v5=ROOT/'assets/opensprite/vendor/sprite0825_v5/mujoco/sprite0825_v5_external_pd.xml'
    models=[mujoco.MjModel.from_xml_path(str(p)) for p in [v4,v5]]
    a,b=models
    fields=['body_quat','jnt_pos','jnt_axis','jnt_range','mesh_vert','mesh_face','geom_pos','geom_quat','geom_size','geom_type']
    for key in fields:np.testing.assert_array_equal(getattr(a,key),getattr(b,key))
    # Nominal free-root world height is different; local body transforms aren't.
    np.testing.assert_array_equal(a.body_pos[2:],b.body_pos[2:])
    np.testing.assert_array_equal(a.qpos0[7:],b.qpos0[7:])
    centered=[]
    for model in models:
        d=mujoco.MjData(model)
        d.qpos[model.joint('left_shoulder_roll_joint').qposadr[0]]=np.pi/2
        d.qpos[model.joint('right_shoulder_roll_joint').qposadr[0]]=-np.pi/2
        mujoco.mj_forward(model,d)
        centered.append(d.geom_xpos-d.xpos[model.body('pelvis_link').id])
    np.testing.assert_allclose(centered[0],centered[1],rtol=0,atol=1e-12)
    report=dict(compatible_centered_surface_geometry=True,verified_fields=fields,
        free_root_height_is_not_surface_geometry=True,v4_source_sha256=hashlib.sha256(v4.read_bytes()).hexdigest(),
        v5_source_sha256=hashlib.sha256(v5.read_bytes()).hexdigest(),
        reason='Same visual meshes, axes, joint origins, limits and centered T-pose geometry; native optimization still uses v5 model and velocity limits')
    (ROOT/'data/correspondence_v5_geometry_proof.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--v4',type=Path,required=True);check(p.parse_args().v4)
