"""Preserve the actual new training scene for CPU motor-envelope simulation."""
from pathlib import Path
import sys,json,hashlib
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
import mujoco
from mjlab.envs import ManagerBasedRlEnv
from opensprite_tracker.supermimic.robot import make_env_cfg
from opensprite_tracker.robot import ASSET_REVISION,CONTROL_PROFILE_SHA256
out=ROOT/'outputs/runtime-run002';out.mkdir(exist_ok=True)
cfg=make_env_cfg(1,seed=43,randomize=False,full_body_collision=True,torque_lag_enabled=False)
env=ManagerBasedRlEnv(cfg,device='cuda:0');model=env.sim.mj_model
path=out/'training-scene.mjb';mujoco.mj_saveModel(model,str(path))
report={'asset_revision':ASSET_REVISION,'control_profile_sha256':CONTROL_PROFILE_SHA256,
    'scene_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'training_pd_hz':500,
    'body_mass_kg':float(model.body_mass.sum()),'nq':model.nq,'nv':model.nv,
    'source':'actual mjlab new-model environment, randomization disabled','hardware_qualified':False}
(out/'scene.json').write_text(json.dumps(report,indent=2)+'\n');env.close();print(json.dumps(report),flush=True)
