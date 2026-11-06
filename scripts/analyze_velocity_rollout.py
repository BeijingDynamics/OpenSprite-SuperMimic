"""Recorded-state gait diagnostics; geometric support is not a measured contact force."""
import argparse,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parents[1]
def analyze(path):
    d=np.load(path,allow_pickle=False);m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/opensprite.xml'));state=mujoco.MjData(m)
    names=[j.get('name') for j in ET.parse(ROOT/'assets/opensprite/vendor/sprite0825_v5/sprite0825_float.urdf').getroot().findall('joint') if j.get('type')!='fixed']
    qids=[int(m.joint(n).qposadr[0]) for n in names]
    gids=[m.geom(s+'_foot_collision').id for s in ['left','right']]
    result=[]
    for env in range(d['qpos'].shape[1]):
        feet=[];height=[]
        for q in d['qpos'][:,env]:
            state.qpos[:7]=q[:7];state.qpos[qids]=q[7:];mujoco.mj_forward(m,state)
            feet.append(state.geom_xpos[gids].copy())
            height.append([state.geom_xpos[g,2]-abs(state.geom_xmat[g].reshape(3,3)[2])@m.geom_size[g] for g in gids])
        feet=np.array(feet)[150:];height=np.array(height)[150:];valid=~d['failed'][150:,env]
        support=height<.008;lift=height>.02
        touchdowns=((~support[:-1])&support[1:]&valid[1:,None]).sum(0)
        lift_events=((~lift[:-1])&lift[1:]&valid[1:,None]).sum(0)
        foot_speed=np.linalg.norm(np.diff(feet[...,:2],axis=0)/.02,axis=-1)
        stable=support[:-1]&support[1:]&valid[1:,None]
        result.append(dict(env=env,command=d['commands'][-1,env].tolist(),
            mean_actual_velocity=d['velocity'][150:,env][valid].mean(0).tolist() if valid.any() else None,
            mean_actual_yaw_rate=float(d['yaw_rate'][150:,env][valid].mean()) if valid.any() else None,
            distance_xy_m=float(np.linalg.norm(d['qpos'][-1,env,:2]-d['qpos'][100,env,:2])),
            touchdown_counts=touchdowns.tolist(),lift_above_2cm_counts=lift_events.tolist(),
            near_ground_foot_speed_m_s=float(foot_speed[stable].mean()) if stable.any() else None,
            minimum_root_height_m=float(d['qpos'][:,env,2].min())))
    report={'rollout':str(path),'scoring_start_seconds':3,'geometric_support_threshold_m':.008,
            'limitations':'Kinematic foot-proxy diagnostics, not measured force/contact labels; speed is heading-local.','per_command':result}
    (path.parent/'gait_diagnostics.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('rollout',type=Path);analyze(p.parse_args().rollout)
