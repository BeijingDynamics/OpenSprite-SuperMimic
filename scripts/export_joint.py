"""Export the joint run's ten-frame velocity interface with ONNX parity.

Only a policy/observation artifact is exported. Hardware admission stays false.
"""
import argparse,json,sys,hashlib
from pathlib import Path
import torch,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from opensprite_tracker.supermimic.joint_interfaces import SharedController
from opensprite_tracker.supermimic.robot import physical_contract
from opensprite_tracker.robot import BODY_JOINT_NAMES,JOINT_CONTROL,URDF,ROBOT_XML,ASSET_REVISION,CONTROL_PROFILE_SHA256

class RuntimeActor(torch.nn.Module):
    def __init__(self,shared):super().__init__();self.shared=shared
    def forward(self,x):
        batch=x.shape[0]
        blocks=[];cursor=0
        for dim in [31,31,31,3,3]:
            blocks.append(x[:,cursor:cursor+10*dim].reshape(batch,10,dim));cursor+=10*dim
        # Runtime term-major order: q, qd, previous actions, gyro, gravity.
        # The trained per-frame order is q, qd, gyro, gravity, previous actions.
        history=torch.cat([blocks[i] for i in [0,1,3,4,2]],-1)
        latent=self.shared.velocity_latent(history,x[:,990:993])
        return self.shared.decode(history,latent).clamp(-20,20)

def export(args):
    import onnxruntime as ort
    ck=torch.load(args.checkpoint,map_location='cpu',weights_only=False)
    if ck.get('schema')!='sprite_v5_joint_supermimic_v1':raise ValueError('Not a joint-interface checkpoint')
    if ck['physical_contract']!=physical_contract(torque_lag_enabled=False):raise ValueError('Physics contract changed')
    shared=SharedController(hidden=tuple(ck['config']['network_hidden']));shared.load_state_dict(ck['shared'])
    actor=RuntimeActor(shared).eval();args.output.mkdir(parents=True,exist_ok=False)
    sample=torch.randn(64,993)*.1
    sample[:,990:]=torch.tensor([.15,0,.1])
    target=args.output/'policy.onnx'
    torch.onnx.export(actor,sample[:1],str(target),input_names=['obs'],output_names=['actions'],opset_version=17,
                      dynamic_axes={'obs':{0:'batch'},'actions':{0:'batch'}},dynamo=False)
    session=ort.InferenceSession(str(target),providers=['CPUExecutionProvider'])
    with torch.no_grad():reference=actor(sample).numpy()
    actual=session.run(None,{'obs':sample.numpy()})[0]
    np.testing.assert_allclose(actual,reference,rtol=1e-4,atol=1e-5)
    terms=[];cursor=0
    for name,dim,hist in [('joint_pos',31,10),('joint_vel',31,10),('actions',31,10),('base_ang_vel',3,10),('projected_gravity',3,10),('velocity_commands',3,0)]:
        length=dim*max(hist,1);terms.append(dict(name=name,start=cursor,end=cursor+length,frame_dim=dim,history_length=hist,
                   shape=[length],history_order='oldest_to_newest',flatten_history_dim=True,scale=None,clip=None));cursor+=length
    controls=[JOINT_CONTROL[n] for n in BODY_JOINT_NAMES]
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    contract=dict(schema='sprite_supermimic_joint_velocity_v1',label='v5_joint_supermimic_iteration_'+str(ck['iteration']),
                  policy_onnx='policy.onnx',policy_onnx_sha256=sha(target),checkpoint_sha256=sha(args.checkpoint),
                  asset_revision=ASSET_REVISION,control_profile_sha256=CONTROL_PROFILE_SHA256,
                  training_run='joint-waist-v5-002',asset_urdf_sha256=sha(URDF),asset_mjcf_sha256=sha(ROBOT_XML),
                  policy_dt=.02,physics_dt=.002,decimation=10,actor_observation_dim=993,action_dim=31,
                  observation_terms=terms,joint_names=list(BODY_JOINT_NAMES),
                  default_joint_pos=[c.default_pos_rad for c in controls],action_offset=[c.default_pos_rad for c in controls],
                  action_scale=[.25]*31,stiffness=[c.kp for c in controls],damping=[c.kd for c in controls],
                  effort_limit=[c.command_effort_nm for c in controls],velocity_limit=[c.urdf_velocity_rad_s for c in controls],
                  action_formula='target_joint_pos = action_offset + action_scale * policy_action',
                  observation_has_horizontal_base_velocity=False,observation_has_global_position=False,observation_has_global_yaw=False,
                  base_ang_vel_frame='root_link_local',projected_gravity_definition='R_world_to_base @ [0, 0, -1]',
                  quaternion_order='wxyz',command_layout=['vx','vy','yaw_rate'],
                  maximum_absolute_action=20,normalization='embedded_running_norm',hardware_qualified=False,
                  physical_ankle_differential=dict(enabled=False,reason='serial-joint simulation only; motor-space envelope not yet integrated'),
                  deployment_handoff_seconds=2.,deployment_handoff_mode='smoothstep_from_pose_equivalent_action',deployment_initial_velocity_mode='zero',
                  limitations='ONNX parity is not hardware admission. Runtime hardware validator must reject until differential/motor contract is qualified.',
                  parity_max_abs_error=float(np.abs(actual-reference).max()))
    (args.output/'contract.json').write_text(json.dumps(contract,indent=2))
    print(json.dumps({'onnx':str(target),'parity_max_abs_error':contract['parity_max_abs_error'],'hardware_qualified':False}))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True);p.add_argument('--output',type=Path,required=True);export(p.parse_args())
