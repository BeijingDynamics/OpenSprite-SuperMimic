"""Measure command coverage using the exact per-clip training command extractor."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from train_joint import reference_commands
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.robot import make_kinematics


def summarize(values):
    return dict(mean=values.mean(0).tolist(),minimum=values.min(0).tolist(),maximum=values.max(0).tolist(),
                p05_p50_p95=np.quantile(values,[.05,.5,.95],axis=0).tolist(),
                quiet_fraction=float(((np.linalg.norm(values[:,:2],axis=-1)<.05)&(abs(values[:,2])<.1)).mean()),
                forward_fraction=float((values[:,0]>.1).mean()),backward_fraction=float((values[:,0]<-.1).mean()),
                lateral_left_fraction=float((values[:,1]>.1).mean()),lateral_right_fraction=float((values[:,1]<-.1).mean()),
                positive_yaw_fraction=float((values[:,2]>.1).mean()),negative_yaw_fraction=float((values[:,2]<-.1).mean()))


def main():
    p=argparse.ArgumentParser();p.add_argument('--motion',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    torch.set_num_threads(2);motion=MotionLib(a.motion,make_kinematics('cpu',True),'cpu');commands=reference_commands(motion).numpy()
    clips=[]
    for name,start,length in zip(motion.clip_names,motion.clip_start.tolist(),motion.clip_len.tolist()):
        clips.append(dict(name=name,frames=length,duration_s=(length-1)/50,**summarize(commands[start:start+length])))
    report=dict(motion_sha256=hashlib.sha256(a.motion.read_bytes()).hexdigest(),command_layout=['vx_m_s','vy_m_s','yaw_rad_s'],
                extractor='Root-heading-frame velocity; centered25-frame mean independently within each source clip, exactly as joint training',
                clips=clips,total_duration_s=sum(row['duration_s'] for row in clips),**summarize(commands),
                limitations='Reference command coverage, not learned command tracking or dynamic feasibility; no time-scaled copy counts as an independent source.')
    from opensprite_tracker.supermimic.sampling import ReturnSampler
    sampler=ReturnSampler(motion)
    probabilities=sampler.probabilities().numpy()
    weights=np.zeros(motion.num_frames,dtype=np.float64)
    for probability,begin,end in zip(probabilities,motion.bin_start.tolist(),motion.bin_end.tolist()):
        weights[begin:end]+=probability/(end-begin)
    weights/=weights.sum()
    predicates=dict(quiet=(np.linalg.norm(commands[:,:2],axis=-1)<.05)&(abs(commands[:,2])<.1),
        forward=commands[:,0]>.1,backward=commands[:,0]<-.1,left=commands[:,1]>.1,right=commands[:,1]<-.1,
        positive_yaw=commands[:,2]>.1,negative_yaw=commands[:,2]<-.1)
    report['initial_reset_sampling']=dict(clip_mass=1/motion.num_clips,
        mean=(commands*weights[:,None]).sum(0).tolist(),fractions={k:float(weights[v].sum()) for k,v in predicates.items()},
        limitation='Exact initial reset distribution; return-adaptive bin weights and episode occupancy change during training')
    with a.output.open('x') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
