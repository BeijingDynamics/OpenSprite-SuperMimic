"""Verify recovery disturbances against actual MuJoCo Warp state and forces."""
import argparse,json,sys
from pathlib import Path
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.robot import make_kinematics,default_joint_pos,velocity_limits
from opensprite_tracker.supermimic.motion import MotionLib
from opensprite_tracker.supermimic.env import TrackingSim
from opensprite_tracker.supermimic.joint_interfaces import VelocityMDP
from opensprite_tracker.supermimic.recovery import RecoveryImpulses


def main():
    p=argparse.ArgumentParser();p.add_argument('--motion',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();torch.set_num_threads(4)
    kin=make_kinematics('cuda:0',True);motion=MotionLib(a.motion,kin,'cuda:0',goal_mode='robot_relative_v2',goal_stride=5)
    mdp=VelocityMDP(kin,motion,default_joint_pos('cuda:0'),velocity_limits('cuda:0'),imu_frame='root')
    env=TrackingSim(8,motion,mdp,'cuda:0',auto_reset=False,randomize=False,reset_noise=False,full_body_collision=True,torque_lag_enabled=False)
    try:
        env.reset(frames=torch.full((8,),50,device='cuda:0',dtype=torch.long))
        action=(env.state()[4]-env.default_q)/.25
        mdp.active_command=torch.zeros(8,3,device='cuda:0')
        for _ in range(3):env.step(action)
        before=env.snapshot(True)
        curriculum=RecoveryImpulses(8,'cuda:0',dict(start_iteration=0,full_iteration=1,
            max_linear_delta_m_s=.35,max_angular_delta_rad_s=.5,interval_seconds=[3,6]),571)
        delta=curriculum.apply(env,1)
        after=env.snapshot(True)
        for key in ('a_prev','ep_step','f','h'):torch.testing.assert_close(after[key],before[key],atol=1e-6,rtol=0)
        for i in (0,1,4,5):torch.testing.assert_close(after['S'][i],before['S'][i],atol=2e-6,rtol=0)
        torch.testing.assert_close(after['S'][2]-before['S'][2],delta[:,:3],atol=2e-6,rtol=0)
        torch.testing.assert_close(after['S'][3]-before['S'][3],delta[:,3:],atol=2e-6,rtol=0)
        assert delta[[0,4]].abs().sum()==0 and (delta[[1,2,3,5,6,7]].norm(dim=-1)>0).all()
        assert delta[:,:3].norm(dim=-1).max()<=.350001
        body=env.robot.find_bodies('waist_yaw_link')[0]
        forces=torch.zeros(8,1,3,device='cuda:0');forces[0,0,1]=10
        env.robot.write_external_wrench_to_sim(forces,torch.zeros_like(forces),body_ids=body)
        torch.testing.assert_close(env.robot.data.body_external_wrench[:,body,:3],forces)
        start=env.state()[0].clone()
        for _ in range(10):env.step(action)
        displacement=env.state()[0]-start
        differential=float(displacement[0,1]-displacement[4,1])
        assert differential>1e-4,'Positive lateral force must produce a measurable differential response'
        env.robot.write_external_wrench_to_sim(torch.zeros_like(forces),torch.zeros_like(forces),body_ids=body)
        env.step(action)
        assert env.robot.data.body_external_wrench[:,body].abs().sum()==0
        report=dict(passed=True,physical_simulator=True,impulse_events=6,nominal_controls=2,
            impulse_velocity_match=True,policy_action_history_unchanged=True,episode_not_reset=True,
            force_n=[0,10,0],force_duration_s=.2,lateral_displacement_vs_control_m=differential,
            force_release_confirmed=True,hardware_operation=False,
            limitation='Simulator plumbing test using pose-hold targets; not trained-policy robustness evidence.')
        a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    finally:env.close()


if __name__=='__main__':main()
