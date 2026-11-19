"""Create an isolated R1 stage; no CAN, IMU or motor operation is performed."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

STAGE=Path('/home/tony/sprite-sm-walk-r1-20261119')
OLD=Path('/home/tony/supermimic-pm01-cpufreq-1117')
LIVE=Path('/home/tony/supermimic-pm01-live-1116')


def main():
    payload=Path(sys.argv[1]);STAGE.mkdir(exist_ok=False)
    with tarfile.open(payload) as archive:
        for item in archive.getmembers():
            resolved=(STAGE/item.name).resolve()
            if not resolved.is_relative_to(STAGE) or not item.isfile():
                raise ValueError('Unexpected payload entry')
        archive.extractall(STAGE)
    shutil.copytree(LIVE/'runtime',STAGE/'runtime',ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(OLD/'tools',STAGE/'tools',ignore=shutil.ignore_patterns('__pycache__'))
    (STAGE/'build').symlink_to((OLD/'build').resolve())
    source=STAGE/'runtime/src/open_sprite_runtime/supermimic_run002.py'
    if hashlib.sha256(source.read_bytes()).hexdigest()!='74da75441521d6aa1ab758ec2ba3b76500e9e42a214fd88a814426b04454879b':
        raise ValueError('Runtime binding source changed')
    text=source.read_text()
    anchor="    if (data.get('training_run')!='joint-waist-v5-002' and not pm01) or data.get('asset_revision')!=ASSET_REVISION:"
    replacement="""    sm_walk_r1 = (data.get('schema') == 'sprite_supermimic_joint_velocity_v2'
                  and data.get('training_run') == 'reference-fusion-walk-stand80'
                  and data.get('checkpoint_sha256') == '70cf3e70354c8b574777709f56f24f83864329d589c256cfa97d6a1236afdedd'
                  and data.get('policy_onnx_sha256') == '5695b6980746534a11d075befc29bf5f7fe0ef2a7d81e3dadf3147c50f4d6930')
    if (data.get('training_run')!='joint-waist-v5-002' and not pm01 and not sm_walk_r1) or data.get('asset_revision')!=ASSET_REVISION:"""
    if text.count(anchor)!=1:raise ValueError('Binding patch anchor mismatch')
    source.write_text(text.replace(anchor,replacement))
    old=json.loads((LIVE/'bundle/contract.json').read_text());new=json.loads((STAGE/'candidate/contract.json').read_text())
    unchanged=['asset_revision','control_profile_sha256','asset_urdf_sha256','asset_mjcf_sha256','policy_dt','physics_dt','decimation',
        'actor_observation_dim','action_dim','observation_terms','joint_names','default_joint_pos','action_offset','action_scale',
        'stiffness','damping','effort_limit','velocity_limit','base_ang_vel_frame','projected_gravity_definition',
        'previous_action_definition','deployment_handoff_seconds','deployment_handoff_mode']
    for key in unchanged:
        if old[key]!=new[key]:raise ValueError('Unexpected interface/control change: '+key)
    sys.path.insert(0,str(STAGE/'runtime/src'))
    from open_sprite_runtime.supermimic_run002 import prepare_bundle
    result=prepare_bundle(STAGE/'candidate/contract.json',LIVE/'bundle/training_control_profile.json',STAGE/'bundle')
    scheduler=(OLD/'ensure-can-scheduler.sh').read_text().replace(str(OLD),str(STAGE))
    (STAGE/'ensure-can-scheduler.sh').write_text(scheduler)
    subprocess.run([STAGE/'build/sprite_can_shadow','--check-commissioning-scheduler','2'],check=True)
    files=[STAGE/'build/sprite_can_shadow',STAGE/'build/sprite_supermimic_protected_ipc',
           STAGE/'tools/run_supermimic_commissioning.py',source,STAGE/'bundle/policy.onnx']
    evidence=dict(name='Sprite SM-Walk R1',unchanged_contract_fields=unchanged,bundle=result,
        native_binaries_reused=True,hardware_tx=0,motor_enable_attempts=0,
        fingerprints={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (STAGE/'staging.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps(evidence,indent=2))


if __name__=='__main__':main()
