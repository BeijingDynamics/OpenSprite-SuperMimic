"""Reconstruct measured foot velocities and audit reward-only counterfactuals."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos
from opensprite_tracker.supermimic.rotations import quat_to_mat,yaw_mat
from opensprite_tracker.supermimic.posture_width import SWING_DEFAULTS

def component_cost(height,speed,command):
    cfg=SWING_DEFAULTS
    c2=command[...,:2].square().sum(-1)+(.2*command[...,2]).square()
    gate=c2/(c2+cfg['command_gate_m_s']**2)
    moving=speed.square()/(speed.square()+cfg['swing_speed_m_s']**2)
    short=(torch.relu(cfg['swing_height_m']-height)/cfg['swing_height_m']).square()
    return .5*cfg['swing_weight']*gate[:,None]*moving*short

def audit(directory):
    torch.set_num_threads(2);directory=Path(directory)
    summary=json.loads((directory/'summary.json').read_text())
    z=np.load(directory/'rollout.npz');kin=make_kinematics('cpu',True)
    # Same tracked feet as the reward factory, with explicit names.
    feet=[kin.tracked_names.index(side+'_ankle_roll_link') for side in ['left','right']]
    tensor=lambda x:torch.as_tensor(x,dtype=torch.float32)
    rows=[]
    for case in summary['results']:
        i=case['env'];pose=tensor(z['qpos'][:-1,i]);obs=tensor(z['proprioception'][1:,i])
        if not torch.allclose(obs[:,:31]+default_joint_pos(),pose[:,7:],atol=1e-5,rtol=0):
            raise ValueError('Arrived-at joint feedback alignment failed')
        if not np.allclose(z['proprioception'][1:,i,68:99],z['policy_action'][:-1,i],atol=1e-6,rtol=0):
            raise ValueError('Previous raw policy action alignment failed')
        R=quat_to_mat(pose[:,3:7]);v=(yaw_mat(R)@tensor(z['velocity'][:-1,i])[...,None]).squeeze(-1)
        w=(R@obs[:,62:65,None]).squeeze(-1)
        p,bR,bv,bw=kin.forward(pose[:,:3],R,v,w,pose[:,7:],obs[:,31:62])
        offset=(bR[:,feet]@kin.sole_centers[...,None]).squeeze(-1)
        cv=bv[:,feet]+torch.linalg.cross(bw[:,feet],offset,dim=-1)
        h=kin.sole_bottom_z(p[:,feet],bR[:,feet]);speed=cv[...,:2].norm(dim=-1)
        valid=(torch.arange(len(pose))>=150)&~tensor(z['failed'][:-1,i]).bool()
        if not valid.any():rows.append({'case':case['case_name'],'valid_samples':0});continue
        h=h[valid];speed=speed[valid];command=tensor(z['commands'][:-1,i])[valid]
        force=tensor(z['contact_force_mean_n'][:-1,i])[valid];loaded=force>5
        cost=component_cost(h,speed,command)
        raised=component_cost(h+.005,speed,command);slowed=component_cost(h,speed*.5,command)
        rows.append({'case':case['case_name'],'valid_samples':int(valid.sum()),
            'sole_height_p95_m':torch.quantile(h,.95,dim=0).tolist(),
            'foot_horizontal_speed_p50_m_s':torch.quantile(speed,.5,dim=0).tolist(),
            'foot_horizontal_speed_p95_m_s':torch.quantile(speed,.95,dim=0).tolist(),
            'loaded_fraction':loaded.float().mean(0).tolist(),
            'low_moving_fraction':((h<.005)&(speed>.08)).float().mean(0).tolist(),
            'mean_per_foot_added_cost':cost.mean(0).tolist(),
            'mean_cost_reduction_raise_5mm':(cost-raised).mean(0).tolist(),
            'mean_cost_reduction_half_speed':(cost-slowed).mean(0).tolist(),
            'unloaded_median_speed_m_s':[float(speed[:,j][~loaded[:,j]].median()) if (~loaded[:,j]).any() else None for j in range(2)]})
    return {'checkpoint_sha256':summary['checkpoint_sha256'],'cases':rows,
        'protocol':'Arrived-at qpos plus next observation qdot and body gyro; raw previous-action alignment checked. Same analytic sole velocity as the reward.',
        'counterfactual_limit':'Reward input changes only. Raising height or halving speed is not a realizable policy rollout or causal proof.',
        'reward_parameter_source':SWING_DEFAULTS,'hardware_access':False}

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.directory)
    with args.output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result,indent=2))
