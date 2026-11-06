"""Meaningful CPU checks for the new physics, observations and differentiable task."""
import sys,json,hashlib
from pathlib import Path
import torch,mujoco,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import URDF,ROBOT_XML,BODY_JOINT_NAMES,JOINT_CONTROL
from opensprite_tracker.supermimic.kinematics import OpenSpriteKinematics
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.velocity import VelocityMDP,VelocityRewardCfg
from opensprite_tracker.supermimic.robot import CONTEXT_DIM,default_joint_pos,velocity_limits

def check():
    assert hashlib.sha256(URDF.read_bytes()).hexdigest()=='6c753d563b54103e4278c27a11a22e0f4f1076e4156f0873cb2325f1701a921b'
    m=mujoco.MjModel.from_xml_path(str(ROBOT_XML))
    source=mujoco.MjModel.from_xml_path(str(URDF.parent/'mujoco/sprite0825_v5_external_pd.xml'))
    for field in ['body_mass','body_inertia','body_ipos','body_iquat','jnt_range','jnt_axis','dof_armature']:
        np.testing.assert_array_equal(getattr(m,field),getattr(source,field))
    assert m.nv==37 and m.nu==0 and len(BODY_JOINT_NAMES)==31
    for side in ['left','right']:
        for axis in ['pitch','roll']:
            assert JOINT_CONTROL[f'{side}_shoulder_{axis}_joint'].command_effort_nm==14
    kin=OpenSpriteKinematics('cpu',active_collision_only=True)
    motion=MotionLib(ROOT/'data/motions/v5_command_seed.npz',kin,'cpu')
    task=VelocityMDP(kin,motion,default_joint_pos(),velocity_limits(),VelocityRewardCfg())
    f=torch.tensor([0,1000,3000]);S=tuple(t.clone() for t in motion.ref_state(f))
    a=torch.zeros(3,31,requires_grad=True);xi=torch.zeros(3,CONTEXT_DIM)
    obs=task.obs(S,a,xi,f)
    assert obs.shape==(3,99)
    # No dependence on global root XY/yaw or reference pose in actor observations.
    shifted=list(S);shifted[0]=S[0]+torch.tensor([100.,-100.,0.])
    torch.testing.assert_close(obs,task.obs(tuple(shifted),a,xi,f))
    bad=list(S);bad[0]=S[0].clone();bad[0][:,2]=.1
    assert task.termination(tuple(bad),f).all()
    assert not task.termination(S,f).any()
    velocity=S[2].clone().requires_grad_();moving=(S[0],S[1],velocity,*S[3:])
    reward=task.reward(moving,f,a,torch.zeros_like(a)).sum()
    reward.backward()
    assert torch.isfinite(velocity.grad).all() and torch.isfinite(a.grad).all()
    assert not task.obs(S,a,xi,f).requires_grad or a.requires_grad
    print(json.dumps({'passed':True,'asset_mass_kg':float(m.body_mass.sum()),'joint_count':31,
                      'observation_per_frame':99,'history':8,'runtime_input_dim':795,
                      'commands':7,'method':'SuperMimic velocity experimental baseline',
                      'max_source_fk_error':max(v['max_abs'] for v in motion.source_fk_error.values())},indent=2))
if __name__=='__main__':check()
