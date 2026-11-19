"""Audit completed R1 evidence and archive it only after control processes exit."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tarfile

STAGE=Path('/home/tony/sprite-sm-walk-r1-20261119')


def main():
    sys.path.insert(0,str(STAGE))
    from compare_live_cpu_frequency import audit
    names=subprocess.check_output(['ps','-eo','comm='],text=True).splitlines()
    active=[n.strip() for n in names if n.strip().startswith(('sprite_can','sprite_zero','sprite_policy','sprite_super','sprite_icm'))]
    if active:raise RuntimeError('Control processes still present: '+str(active))
    timing,_,fingerprints=audit(STAGE/'fixed-disabled-01')
    cpu=json.loads((STAGE/'cpu-replay/report.json').read_text())
    fault=json.loads((STAGE/'fixed-fault-01/commissioning.json').read_text())
    old=json.loads(Path('/home/tony/supermimic-pm01-cpufreq-1117/fixed-disabled-01/commissioning.json').read_text())
    unchanged=['sprite_icm45686','sprite_can_shadow','sprite_supermimic_protected_ipc','motors.tsv',
        'kinematics.tsv','joints.tsv','feedback.bin','target.bin','input.bin','loaded_kcan_srcversion',
        'run_supermimic_commissioning.py','cpu_frequency_client.py','cpu_frequency_guard.py']
    for key in unchanged:
        if fingerprints[key]!=old['fingerprints'][key]:raise ValueError('Control implementation changed: '+key)
    import numpy as np
    inputs=np.fromfile(STAGE/'fixed-disabled-01/native.bin.inputs',dtype='<f8').reshape(-1,71)
    if inputs.shape!=(2701,71) or np.any(inputs[:,68:71]!=0):raise ValueError('Expected complete zero-command validation')
    gamepad=np.genfromtxt(STAGE/'fixed-disabled-01/native.bin.gamepad.csv',delimiter=',',names=True)
    report=dict(name='Sprite SM-Walk R1',cpu_replay_passed=cpu['passed'],cpu_replay_frames=sum(c['frames'] for c in cpu['cases']),
        cpu_max_recorded_action_error=max(c['native_vs_recorded_simulation_max_abs_error']['raw_action'] for c in cpu['cases']),
        live=timing,deadline_fault_passed=fault['passed'],deadline_final_disabled_count=fault['final_disabled_count'],
        unchanged_fingerprints={key:fingerprints[key] for key in unchanged},all_commands_zero=True,
        final_control_processes=active,powered_test_performed=False,
        gamepad_connected_during_validation=bool(np.any(gamepad['connected']!=0)))
    (STAGE/'deployment-summary.json').write_text(json.dumps(report,indent=2)+'\n')
    archive=STAGE/'validation-evidence-v2.tar.gz'
    members=['staging.json','deployment-summary.json','operator-xbox.sh','run-disabled.py',
        'cpu-replay','cpu-replay.log','cpu-replay-frequency.json','fixed-disabled-01','fixed-fault-01',
        'fixed-disabled-01.log','fixed-fault-01.log','fixed-disabled-01-job.json','fixed-fault-01-job.json',
        'bundle/bundle.json','bundle/contract.json','bundle/training_control_profile.json',
        'runtime/src/open_sprite_runtime/supermimic_run002.py','tools','operator-status.log']
    with tarfile.open(archive,'w:gz',compresslevel=1) as stream:
        for name in members:stream.add(STAGE/name,arcname=name,filter=lambda item:None if '__pycache__' in item.name else item)
    meta=dict(archive=str(archive),bytes=archive.stat().st_size,sha256=hashlib.sha256(archive.read_bytes()).hexdigest())
    (STAGE/'validation-evidence-v2-manifest.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(report,indent=2));print(json.dumps(meta))


if __name__=='__main__':main()
