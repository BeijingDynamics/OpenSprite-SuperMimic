"""Verify native ONNX history/command ordering on recorded physical rollouts."""
import argparse,hashlib,json,platform,sys,time
from pathlib import Path
import numpy as np
import torch
import onnxruntime as ort
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from export_joint import RuntimeActor
from opensprite_tracker.supermimic.joint_interfaces import SharedController
from opensprite_tracker.supermimic.revisions import revision_contract


def main():
    p=argparse.ArgumentParser();p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--export-dir',type=Path,required=True);p.add_argument('--rollout',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--samples',type=int,default=512)
    args=p.parse_args()
    if args.samples<1:raise ValueError('At least one replay sample is required')
    sha=lambda f:hashlib.sha256(f.read_bytes()).hexdigest()
    contract=json.loads((args.export_dir/'contract.json').read_text());onnx=args.export_dir/'policy.onnx'
    if contract['checkpoint_sha256']!=sha(args.checkpoint) or contract['policy_onnx_sha256']!=sha(onnx):
        raise ValueError('Export provenance does not match the supplied checkpoint and ONNX file')
    with np.load(args.rollout,allow_pickle=False) as z:
        obs=z['proprioception'];commands=z['commands'];actions=z['policy_action'];valid=np.argwhere(~z['failed'])
    if obs.shape[-1]!=99 or commands.shape[-1]!=3 or actions.shape[-1]!=31 or not len(valid):
        raise ValueError('Replay does not contain valid native velocity-interface samples')
    selected=valid[np.linspace(0,len(valid)-1,min(args.samples,len(valid)),dtype=int)]
    inputs=[];recorded=[]
    for t,env in selected:
        history=obs[np.maximum(np.arange(t-9,t+1),0),env]
        inputs.append(np.concatenate([history[:,sl].reshape(-1) for sl in [slice(0,31),slice(31,62),slice(68,99),slice(62,65),slice(65,68)]]+[commands[t,env]]))
        recorded.append(actions[t,env])
    inputs=np.asarray(inputs,np.float32);recorded=np.asarray(recorded,np.float32)
    ck=torch.load(args.checkpoint,map_location='cpu',weights_only=False);revision=revision_contract(ck['config'])
    torch.set_num_threads(2)
    shared=SharedController(hidden=tuple(ck['config']['network_hidden']),normalized_gelu=revision['normalized_gelu'])
    shared.load_state_dict(ck['shared']);actor=RuntimeActor(shared).eval()
    with torch.no_grad():expected=actor(torch.from_numpy(inputs)).numpy()
    options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
    session=ort.InferenceSession(str(onnx),sess_options=options,providers=['CPUExecutionProvider'])
    actual=session.run(None,{'obs':inputs})[0]
    np.testing.assert_allclose(expected,recorded,rtol=1e-4,atol=1e-5)
    np.testing.assert_allclose(actual,recorded,rtol=1e-4,atol=1e-5)
    for _ in range(24):session.run(None,{'obs':inputs[:1]})
    times=[]
    for i in range(256):
        index=i%len(inputs);start=time.perf_counter_ns();session.run(None,{'obs':inputs[index:index+1]});times.append((time.perf_counter_ns()-start)/1e6)
    cpu=platform.machine()
    info=Path('/proc/cpuinfo')
    if info.exists():cpu=next((line.split(':',1)[1].strip() for line in info.read_text().splitlines() if line.startswith('model name')),cpu)
    report=dict(passed=True,samples=len(inputs),sampled_environments=sorted(set(int(i) for _,i in selected)),
                torch_to_recorded_max_abs_error=float(abs(expected-recorded).max()),onnx_to_recorded_max_abs_error=float(abs(actual-recorded).max()),
                history='Ten frames, oldest to newest; repeat initial observed frame at startup; previous raw clipped policy action before target saturation',
                runtime_term_order=['q_offset','qd','previous_action','gyro_rad_s','projected_unit_gravity','vx_vy_yaw_command'],imu_frame=revision['imu_frame'],
                onnx_cpu_timing_ms=dict(zip(['p50','p95','p99','maximum'],np.quantile(times,[.5,.95,.99,1]).tolist())),
                timing_scope=dict(cpu=cpu,provider='CPUExecutionProvider',intra_op_threads=1,inter_op_threads=1,batch=1,warmup_calls=24,measured_calls=256),
                hashes={str(f):sha(f) for f in [args.checkpoint,onnx,args.rollout]},hardware_qualified=False,
                limitations='Offline recorded-input parity and this host CPU only. No Jetson timing, live IPC, motor-space dynamics or hardware admission.')
    with args.output.open('x') as f:json.dump(report,f,indent=2);f.write('\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
