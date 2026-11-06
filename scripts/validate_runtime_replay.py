"""Compare native runtime ten-frame assembly + ONNX against physical traces."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np,onnxruntime as ort
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'runtime_vendor'))
from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.policy_shadow import PolicyObservationHistory

def main(args):
    contract=PolicyContract.load(args.export/'contract.json')
    source=json.loads((args.rollout.parent/'summary.json').read_text())
    assert source['checkpoint_sha256']==contract.data['checkpoint_sha256']
    assert hashlib.sha256((args.export/'policy.onnx').read_bytes()).hexdigest()==contract.data['policy_onnx_sha256']
    options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
    session=ort.InferenceSession(str(args.export/'policy.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
    data=np.load(args.rollout,allow_pickle=False);default=np.asarray(contract.data['default_joint_pos'],np.float32)
    histories=[PolicyObservationHistory(contract) for _ in range(data['proprioception'].shape[1])]
    maximum=0.;squared=0.;count=0
    for t in range(min(500,len(data['proprioception']))):
        for env,history in enumerate(histories):
            obs=data['proprioception'][t,env]
            history.append(obs[:31]+default,obs[31:62],obs[68:99],obs[62:65],obs[65:68])
            if t%25:continue
            runtime_obs=history.observation(data['commands'][t,env]);assert runtime_obs.shape==(993,)
            actual=session.run(None,{'obs':runtime_obs[None]})[0][0]
            expected=data['policy_action'][t,env];error=actual-expected
            maximum=max(maximum,float(abs(error).max()));squared+=float(np.square(error).sum());count+=error.size
    report=dict(passed=maximum<1e-4,input_dim=993,history_frames=10,action_dim=31,max_abs_action_error=maximum,
                rms_action_error=float(np.sqrt(squared/count)),compared_actions=count//31,
                runtime_commit='4515206c24404e18a2d5c168eccdb7468aea6ab7',
                qualification='Offline observation assembly and ONNX replay parity only; policy gait failed; no hardware qualification')
    args.output.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
    assert report['passed']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--rollout',type=Path,required=True);p.add_argument('--export',type=Path,required=True);p.add_argument('--output',type=Path,required=True);main(p.parse_args())
