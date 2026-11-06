"""Assert the selected asset, effective torque limits/gains, and500Hz stepping."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
import mujoco,torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import (ROBOT_XML,URDF,ASSET_REVISION,CONTROL_PROFILE_SHA256,
    BODY_JOINT_NAMES,JOINT_CONTROL,CONTROL_PROFILE)
from opensprite_tracker.supermimic.robot import make_env_cfg,physical_contract
from mjlab.envs import ManagerBasedRlEnv


def main():
    torch.set_num_threads(4);torch.set_float32_matmul_precision('highest')
    torch.backends.cuda.matmul.allow_tf32=False
    model=mujoco.MjModel.from_xml_path(str(ROBOT_XML))
    assert hashlib.sha256(URDF.read_bytes()).hexdigest()=='f873954bf60d9ff2bb4e49d3ec68ba2bf8cf0bdfb1144aaa1103f6a302663bd9'
    assert CONTROL_PROFILE_SHA256=='184ec989fd24b4bc6ea5034e6fdae796b5b9ce83ef2c884dfe4fc61d8d6987dc'
    assert abs(model.body_mass[model.body('waist_yaw_link').id]-3.7391)<1e-10
    controls=[JOINT_CONTROL[n] for n in BODY_JOINT_NAMES]
    limits=np.array([c.command_effort_nm for c in controls])
    joint_ids=[model.joint(n).id for n in BODY_JOINT_NAMES]
    np.testing.assert_allclose(model.jnt_actfrcrange[joint_ids],np.c_[-limits,limits],atol=1e-10,rtol=0)
    assert model.jnt_actfrclimited[joint_ids].all()
    cfg=make_env_cfg(32,seed=43,randomize=True,full_body_collision=True,torque_lag_enabled=False)
    assert 'pd_gains' not in cfg.events and 'motor_strength' not in cfg.events
    assert cfg.sim.mujoco.timestep==.002 and cfg.decimation==10
    env=ManagerBasedRlEnv(cfg,device='cuda:0')
    robot=env.scene['robot'];act=robot.actuators[0]
    order=[n.rsplit('/',1)[-1] for n in act.target_names]
    actual_ids=[order.index(n) for n in BODY_JOINT_NAMES]
    checks={}
    for field,attribute in [('stiffness','kp'),('damping','kd'),('force_limit','command_effort_nm')]:
        expected=np.array([getattr(c,attribute) for c in controls])
        actual=getattr(act,field)[:,actual_ids].cpu().numpy()
        np.testing.assert_allclose(actual,np.broadcast_to(expected,actual.shape),atol=1e-6,rtol=1e-6)
        checks[field]=float(np.max(abs(actual-expected)))
    names=[env.sim.mj_model.joint(i).name.rsplit('/',1)[-1] for i in range(env.sim.mj_model.njnt)]
    ids=[names.index(n) for n in BODY_JOINT_NAMES]
    np.testing.assert_allclose(env.sim.mj_model.jnt_actfrcrange[ids],np.c_[-limits,limits],atol=1e-10,rtol=0)
    a_names=[env.sim.mj_model.actuator(i).name.rsplit('/',1)[-1] for i in range(env.sim.mj_model.nu)]
    # Map by transmission joint rather than guessing actuator names/order.
    effective=[]
    for jid in ids:
        aid=int(np.flatnonzero(env.sim.mj_model.actuator_trnid[:,0]==jid)[0])
        effective.append(env.sim.mj_model.actuator_forcerange[aid])
    np.testing.assert_allclose(effective,np.c_[-limits,limits],atol=1e-10,rtol=0)
    before=env.sim.data.time.clone();env.step(torch.zeros(32,31,device='cuda:0'))
    elapsed=(env.sim.data.time-before).cpu().numpy()
    np.testing.assert_allclose(elapsed,.02,atol=1e-6,rtol=0)
    report={'passed':True,'asset_revision':ASSET_REVISION,'profile_sha256':CONTROL_PROFILE_SHA256,
        'derived_mjcf_sha256':hashlib.sha256(ROBOT_XML.read_bytes()).hexdigest(),
        'source_urdf_sha256':hashlib.sha256(URDF.read_bytes()).hexdigest(),
        'waist_mass_kg':3.7391,'training_pd_hz':500,'physics_dt':.002,'policy_dt':.02,'decimation':10,
        'effective_joint_caps':limits.tolist(),'actual_gpu_parameter_max_errors':checks,
        'gain_randomization':False,'effort_randomization':False,'outer_step_seconds':elapsed.tolist(),
        'physical_contract':physical_contract(torque_lag_enabled=False),'hardware_qualified':False}
    out=ROOT/'outputs/preflight-run002';out.mkdir(parents=True,exist_ok=True)
    (out/'effective_contract.json').write_text(json.dumps(report,indent=2)+'\n')
    env.close();print(json.dumps({'passed':True,'asset':ASSET_REVISION,'pd_hz':500,'joint_caps_match':True,'actual_step':float(elapsed[0])}),flush=True)


if __name__=='__main__':main()
