"""Real-bundle tamper tests, including consistent manifest rehash attempts."""
from pathlib import Path
import argparse,json,tempfile,shutil
from open_sprite_runtime.supermimic_run002 import verify_bundle
from open_sprite_runtime.supermimic_replay import file_sha256
from open_sprite_runtime.contracts import PolicyContract,RuntimeTiming

def main():
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();verify_bundle(args.bundle);cases=[]
    for name in ('model_bytes','gain','torque_cap','offset','history','hardware_flag','published_profile'):
        with tempfile.TemporaryDirectory() as tmp:
            folder=Path(tmp)/'bundle';shutil.copytree(args.bundle,folder)
            if name=='model_bytes':
                model=folder/'policy.onnx';blob=bytearray(model.read_bytes());blob[-1]^=1;model.write_bytes(blob)
            elif name=='hardware_flag':
                path=folder/'bundle.json';q=json.loads(path.read_text());q['allow_hardware_tx']=True;path.write_text(json.dumps(q))
            elif name=='published_profile':
                path=folder/'training_control_profile.json';q=json.loads(path.read_text());next(iter(q['joints'].values()))['kp']+=1;path.write_text(json.dumps(q))
            else:
                path=folder/'contract.json';q=json.loads(path.read_text())
                if name=='history':q['observation_terms'][0]['history_length']=8
                else:q[{'gain':'stiffness','torque_cap':'effort_limit','offset':'action_offset'}[name]][0]+=.1
                path.write_text(json.dumps(q))
            if name not in ('model_bytes','hardware_flag'):
                path=folder/'bundle.json';q=json.loads(path.read_text())
                q['files_sha256']={n:file_sha256(folder/n) for n in q['files_sha256']};path.write_text(json.dumps(q))
            try:verify_bundle(folder);cases.append(dict(name=name,passed=False,error='tamper accepted'))
            except ValueError as exc:cases.append(dict(name=name,passed=True,error=str(exc)))
    try:PolicyContract.load(args.bundle/'contract.json').validate_for_hardware(RuntimeTiming(50,500,1000))
    except ValueError as exc:cases.append(dict(name='hardware_admission_blocked',passed=True,error=str(exc)))
    else:cases.append(dict(name='hardware_admission_blocked',passed=False))
    report={'passed':all(c['passed'] for c in cases),'cases':cases,'hardware_tx':0,'live_devices_opened':0}
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)
if __name__=='__main__':main()
