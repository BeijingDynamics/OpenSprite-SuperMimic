"""Replay recorded physical histories through the exported native observation order."""
import argparse,json
from pathlib import Path
import numpy as np
import onnxruntime as ort

def main():
    p=argparse.ArgumentParser();p.add_argument('--export',type=Path,required=True);p.add_argument('--rollout',type=Path,required=True);a=p.parse_args()
    with np.load(a.rollout) as z:
        obs=z['proprioception'];cmd=z['commands'];target=z['policy_action']
    options=ort.SessionOptions();options.intra_op_num_threads=2
    session=ort.InferenceSession(str(a.export/'policy.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
    maximum=0.;count=0
    for t in range(0,len(obs),5):
        history=obs[np.maximum(0,np.arange(t-9,t+1))].transpose(1,0,2)
        blocks=[history[:,:,sl].reshape(len(cmd[t]),-1) for sl in [slice(0,31),slice(31,62),slice(68,99),slice(62,65),slice(65,68)]]
        x=np.concatenate([*blocks,cmd[t]],axis=-1).astype(np.float32)
        actual=session.run(None,{'obs':x})[0]
        np.testing.assert_allclose(actual,target[t],rtol=2e-4,atol=2e-4)
        maximum=max(maximum,float(abs(actual-target[t]).max()));count+=len(x)
    report=dict(samples=count,max_absolute_action_error=maximum,history_length=10,observation_dimension=993,
        source=str(a.rollout),passed=True,hardware_qualified=False)
    (a.export/'recorded-replay-parity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))

if __name__=='__main__':main()
