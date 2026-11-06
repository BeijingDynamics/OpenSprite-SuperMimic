import json,sys,torch
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from opensprite_tracker.supermimic.joint_interfaces import SharedController,proprioception
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,CONTEXT_DIM
from train_joint import reference_commands

@torch.no_grad()
def main():
    torch.set_num_threads(4);torch.set_float32_matmul_precision('highest');device='cuda:0'
    run=ROOT/'outputs/joint-paper-v5-001';checkpoint=sorted(run.glob('model_*.pt'))[-1]
    ck=torch.load(checkpoint,map_location=device,weights_only=False)
    shared=SharedController(hidden=tuple(ck['config']['network_hidden'])).to(device);shared.load_state_dict(ck['shared']);shared.eval()
    motion=MotionLib(ROOT/'data/motions/contact_v5_v11/v5_locomotion.npz',make_kinematics(device,True),device)
    frames=torch.arange(motion.num_frames,device=device);goal=motion.goal(frames,10)
    obs=proprioception(motion.ref_state(frames),torch.zeros(len(frames),31,device=device),torch.zeros(len(frames),CONTEXT_DIM,device=device),default_joint_pos(device))
    history=obs[:,None].expand(-1,10,-1);teacher=shared.motion_latent(goal)
    student=shared.velocity_latent(history,reference_commands(motion))
    fixed_history=history[:1].expand_as(history);actions=shared.decode(fixed_history,teacher)
    output=dict(iteration=ck['iteration'],motion_latent_mean_dimension_std=float(teacher.std(0).mean()),
                velocity_latent_mean_dimension_std=float(student.std(0).mean()),reference_latent_mse=float((teacher-student).square().mean()),
                decoder_action_std_from_varied_motion_latent_at_fixed_history=float(actions.std(0).mean()),
                limitations='Reference histories are constant-frame fixtures, not actual on-policy histories; sensitivity diagnostic only')
    (run/'latent_diagnostic.json').write_text(json.dumps(output,indent=2));print(json.dumps(output),flush=True)
main()
