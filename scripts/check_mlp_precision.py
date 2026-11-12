"""Compare BF16 MLP evaluation with FP32 on identical reference-history fixtures."""
import argparse
import json
from pathlib import Path
import sys
import torch

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic import networks
from opensprite_tracker.supermimic.joint_interfaces import SharedController,proprioception
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,CONTEXT_DIM,MEM_DIM
from opensprite_tracker.supermimic.revisions import revision_contract


@torch.no_grad()
def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--motion',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    torch.set_float32_matmul_precision('highest');torch.backends.cuda.matmul.allow_tf32=False
    device='cuda:0';ck=torch.load(args.checkpoint,map_location=device,weights_only=False)
    cfg=ck['config'];revision=revision_contract(cfg)
    motion=MotionLib(args.motion,make_kinematics(device,True),device,goal_mode=revision['goal_mode'],goal_stride=revision['goal_stride'])
    actor=SharedController(hidden=tuple(cfg['network_hidden']),normalized_gelu=revision['normalized_gelu']).to(device).eval()
    actor.load_state_dict(ck['shared'])
    wm=networks.WorldModel(hidden=(cfg['wm_width'],)*3,context_dim=CONTEXT_DIM,memory_dim=MEM_DIM,
                          dynamics_mode=revision['dynamics_mode'],dt=.02,substeps=10).to(device).eval()
    wm.load_state_dict(ck['wm'])
    frames=torch.linspace(0,len(motion.q)-1,256,device=device).long()
    hframes=torch.maximum(frames[:,None]-torch.arange(9,-1,-1,device=device),motion.clip_start[motion.clip_id[frames]][:,None])
    hist=proprioception(motion.ref_state(hframes),torch.zeros(256,10,31,device=device),
                       torch.zeros(256,10,CONTEXT_DIM,device=device),default_joint_pos(device))
    command=torch.tensor([.25,0,0],device=device).expand(256,-1)
    initial=motion.ref_state(frames)+(torch.zeros(256,MEM_DIM,device=device),)
    context=torch.zeros(256,CONTEXT_DIM,device=device)
    trajectories=[];actions=[]
    for dtype in (None,torch.bfloat16):
        networks.AMP_DTYPE=dtype
        action=actor.decode(hist,actor.velocity_latent(hist,command)).clamp(-20,20)
        actions.append(action)
        state=initial;trajectory=[]
        # Fixed FP32 actions isolate numerical dynamics drift from policy drift.
        for step in range(32):
            state=wm.step(state,actions[0],context)
            if step in (0,7,31):trajectory.append(state)
        trajectories.append(trajectory)
    networks.AMP_DTYPE=None
    delta=(actions[1]-actions[0]).abs()
    rows=[]
    for step,a,b in zip((1,8,32),*trajectories):
        error=wm.state_error(a,b)
        rows.append(dict(horizon=step,finite=bool(torch.isfinite(error).all()),
                         root_position_max_m=float(error[:,:3].abs().max()),
                         joint_position_max_rad=float(error[:,12:43].abs().max()),
                         normalized_state_rms=float((error/wm.nominal_state_scale()).square().mean().sqrt())))
    report=dict(checkpoint=str(args.checkpoint),fixture_count=256,
                action_max_difference=float(delta.max()),target_max_difference_rad=float(.25*delta.max()),
                action_rms_difference=float(delta.square().mean().sqrt()),world_model_precision_difference=rows,
                limitations='Reference-derived histories, zero previous actions and memory, nominal normalized context; numerical smoke only, not on-policy prediction accuracy or hardware qualification')
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    if not all(row['finite'] for row in rows):raise RuntimeError('Nonfinite numerical precision comparison')


if __name__=='__main__':main()
