"""CPU-only export from frozen checkpoints, without importing simulator plugins."""
import ast,hashlib,json,sys
from pathlib import Path
import numpy as np
import torch
import onnxruntime as ort
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.networks import mlp,RunningNorm,run_mlp
from opensprite_tracker.supermimic.revisions import revision_contract

def definition(path,name,scope):
    tree=ast.parse(path.read_text())
    node=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name==name)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]

def main():
    torch.set_num_threads(2)
    base=ROOT/'outputs/retrain-20261114/seed45-lr-continuation/actor-quarter/training'
    template=ROOT/'outputs/retrain-20261112/pm01-clearance-training/training/recovery-audit-006030/export-candidate/contract.json'
    output=ROOT/'outputs/deploy-20261116'
    scope=dict(torch=torch,nn=torch.nn,mlp=mlp,RunningNorm=RunningNorm,run_mlp=run_mlp,HISTORY=10,OBS_DIM=99)
    Shared=definition(ROOT/'opensprite_tracker/supermimic/joint_interfaces.py','SharedController',scope)
    Runtime=definition(ROOT/'scripts/export_joint.py','RuntimeActor',scope)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    for step,expected in [(10000,'aab860df068d5ccc67aa94ea315f5517dd333373c0e7be52fe6344bb5a98ee8a'),(9000,'50a6aa050b9b3f6c78fff4e0dcd60e5cbc87a3cfb5912a6a0f6517c49e1d9492')]:
        audit=base/f'recovery-audit-{step:06d}';checkpoint=audit/'checkpoint.pt'
        assert sha(checkpoint)==expected
        ck=torch.load(checkpoint,map_location='cpu',weights_only=False);rev=revision_contract(ck['config'])
        contract=json.loads(template.read_text());physics=ck['physical_contract']
        assert contract['asset_revision']==physics['asset_revision'] and contract['control_profile_sha256']==physics['control_profile_sha256']
        assert contract['joint_names']==list(physics['joints'])
        controls=dict(physics['control'])
        for field,key in [('stiffness','kp'),('damping','kd'),('effort_limit','command_effort_nm'),('default_joint_pos','default_pos_rad')]:
            np.testing.assert_equal(contract[field],[controls[n][key] for n in contract['joint_names']])
        assert contract['training_revision']==rev
        shared=Shared(hidden=tuple(ck['config']['network_hidden']),normalized_gelu=rev['normalized_gelu']);shared.load_state_dict(ck['shared'])
        actor=Runtime(shared).eval()
        with np.load(audit/'velocity-seed101/rollout.npz') as trace:
            obs=trace['proprioception'];cmd=trace['commands'];actions=trace['policy_action'];valid=np.argwhere(~trace['failed'])
            pick=valid[np.linspace(0,len(valid)-1,512,dtype=int)]
            inputs=[];recorded=[]
            for t,e in pick:
                h=obs[np.maximum(np.arange(t-9,t+1),0),e]
                inputs.append(np.concatenate([h[:,s].reshape(-1) for s in [slice(0,31),slice(31,62),slice(68,99),slice(62,65),slice(65,68)]]+[cmd[t,e]]));recorded.append(actions[t,e])
        x=np.array(inputs,np.float32);recorded=np.array(recorded)
        with torch.no_grad():reference=actor(torch.from_numpy(x)).numpy()
        np.testing.assert_allclose(reference,recorded,atol=1e-5,rtol=1e-4)
        out=output/f'seed45-quarter{step}';out.mkdir(parents=True,exist_ok=False)
        model=out/'policy.onnx'
        torch.onnx.export(actor,torch.from_numpy(x[:1]),str(model),input_names=['obs'],output_names=['actions'],opset_version=17,dynamic_axes={'obs':{0:'batch'},'actions':{0:'batch'}},dynamo=False)
        options=ort.SessionOptions();options.intra_op_num_threads=2
        actual=ort.InferenceSession(str(model),sess_options=options,providers=['CPUExecutionProvider']).run(None,{'obs':x})[0]
        np.testing.assert_allclose(actual,recorded,atol=1e-5,rtol=1e-4)
        contract.update(label=f'pm01-seed45-quarter-{step}-20261116',training_run=ck['config']['experiment_id'],checkpoint_sha256=expected,policy_onnx_sha256=sha(model),hardware_qualified=False,parity_max_abs_error=float(abs(actual-recorded).max()))
        (out/'contract.json').write_text(json.dumps(contract,indent=2)+'\n')
        report=dict(passed=True,recorded_samples=512,torch_recorded_max_abs=float(abs(reference-recorded).max()),onnx_recorded_max_abs=float(abs(actual-recorded).max()),torch_version=torch.__version__,ort_version=ort.__version__,hardware_qualified=False)
        (out/'export-parity.json').write_text(json.dumps(report,indent=2)+'\n')
        print(step,json.dumps(report),flush=True)

if __name__=='__main__':main()
