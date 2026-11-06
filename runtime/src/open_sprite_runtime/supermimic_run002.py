"""Version-bound run002 replay bundle. This module has no device/TX interface."""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import shutil

from .contracts import PolicyContract
from .supermimic_replay import checked_model, file_sha256

ASSET_REVISION = 'sprite0825_v5_4340_shoulders_waist20261105'
PROFILE_SHA256 = '184ec989fd24b4bc6ea5034e6fdae796b5b9ce83ef2c884dfe4fc61d8d6987dc'
SCHEMA = 'sprite_supermimic_run002_offline_bundle_v1'

def validate_binding(contract: PolicyContract, profile_path: Path) -> dict:
    checked_model(contract)
    data=contract.data
    if data.get('training_run')!='joint-waist-v5-002' or data.get('asset_revision')!=ASSET_REVISION:
        raise ValueError('run002 training/model identity mismatch')
    digest=file_sha256(profile_path)
    if digest!=PROFILE_SHA256 or data.get('control_profile_sha256')!=digest:
        raise ValueError('published control profile hash mismatch')
    profile=json.loads(profile_path.read_text(encoding='utf-8'))
    if profile['model_binding']['asset_revision']!=ASSET_REVISION:
        raise ValueError('profile asset revision mismatch')
    urdf_name=profile['model_binding']['asset_directory']+'/sprite0825_float.urdf'
    if data.get('asset_urdf_sha256')!=profile['model_binding']['files_sha256'][urdf_name]:
        raise ValueError('source URDF hash mismatch')
    if data.get('hardware_qualified') is not False or profile.get('hardware_qualified') is not False:
        raise ValueError('training artifacts cannot grant hardware qualification')
    names=data['joint_names']
    if set(names)!=set(profile['joints']):raise ValueError('joint names differ from published profile')
    for output,source in [('stiffness','kp'),('damping','kd'),('effort_limit','effort_nm'),
                          ('default_joint_pos','default_pos_rad'),('action_offset','default_pos_rad'),
                          ('velocity_limit','velocity_rad_s')]:
        for name,value in zip(names,data[output],strict=True):
            if not math.isclose(value,profile['joints'][name][source],rel_tol=1e-7,abs_tol=1e-8):
                raise ValueError(f'{output} differs from profile at {name}')
    if any(not math.isclose(x,.25,abs_tol=1e-12) for x in data['action_scale']):
        raise ValueError('run002 action scale mismatch')
    return {'profile_sha256':digest,'source_published_pd_hz':200,'effective_training_pd_hz':500,
            'policy_hz':50,'history_frames':10,'observation_size':993,'action_size':31,
            'hardware_qualified':False,'hardware_tx':0}

def prepare_bundle(contract_path: Path,profile_path: Path,output: Path) -> dict:
    contract=PolicyContract.load(contract_path)
    binding=validate_binding(contract,profile_path)
    output.mkdir(parents=True,exist_ok=False)
    shutil.copyfile(checked_model(contract),output/'policy.onnx')
    data=dict(contract.data);data['policy_onnx']='policy.onnx'
    (output/'contract.json').write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    shutil.copyfile(profile_path,output/'training_control_profile.json')
    manifest={'schema':SCHEMA,'binding':binding,'checkpoint_sha256':data['checkpoint_sha256'],
        'asset_revision':ASSET_REVISION,'permitted_modes':['recorded_replay','simulated_state_ipc_receive_only'],
        'hardware_qualified':False,'allow_hardware_tx':False,
        'motor_enable_supported':False,'jetson_latency_qualified':False,
        'files_sha256':{name:file_sha256(output/name) for name in ('policy.onnx','contract.json','training_control_profile.json')}}
    (output/'bundle.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
    return verify_bundle(output)

def verify_bundle(folder: Path) -> dict:
    manifest=json.loads((folder/'bundle.json').read_text(encoding='utf-8'))
    if manifest.get('schema')!=SCHEMA:raise ValueError('unexpected bundle schema')
    for flag in ('hardware_qualified','allow_hardware_tx','motor_enable_supported','jetson_latency_qualified'):
        if manifest.get(flag) is not False:raise ValueError('offline bundle cannot grant '+flag)
    expected={'policy.onnx','contract.json','training_control_profile.json'}
    files=manifest.get('files_sha256',{})
    if set(files)!=expected:raise ValueError('bundle file set differs')
    for name,digest in files.items():
        path=folder/name
        if path.is_symlink() or file_sha256(path)!=digest:raise ValueError('bundle file hash mismatch: '+name)
    contract=PolicyContract.load(folder/'contract.json')
    binding=validate_binding(contract,folder/'training_control_profile.json')
    if manifest.get('binding')!=binding or manifest.get('checkpoint_sha256')!=contract.data['checkpoint_sha256']:
        raise ValueError('bundle binding differs from contract')
    return {'passed':True,'bundle':str(folder.resolve()),'checkpoint_sha256':contract.data['checkpoint_sha256'],
            'policy_sha256':contract.data['policy_onnx_sha256'],**binding}

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='operation',required=True)
    prepare=sub.add_parser('prepare');prepare.add_argument('--contract',type=Path,required=True)
    prepare.add_argument('--profile',type=Path,required=True);prepare.add_argument('--output',type=Path,required=True)
    verify=sub.add_parser('verify');verify.add_argument('--bundle',type=Path,required=True)
    args=parser.parse_args()
    result=prepare_bundle(args.contract,args.profile,args.output) if args.operation=='prepare' else verify_bundle(args.bundle)
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
