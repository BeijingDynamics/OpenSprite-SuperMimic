"""Reproduce the fixed-input strict simulation recipe; no hardware backend."""
from pathlib import Path
import argparse,json,hashlib,subprocess,sys
import math
import numpy as np

def checked_arguments(profile,contract,hardware,scene,commands,output,timing,seeds):
    p=json.loads(profile.read_text())
    if p.get('schema')!='sprite_run002_strict_startup_sim_v1' or p.get('scope')!='closed_loop_simulation_only':
        raise ValueError('unsupported simulation profile')
    if p.get('hardware_qualified') is not False or p.get('motor_enable_supported') is not False:
        raise ValueError('simulation profile cannot enable hardware')
    for key,path in [('contract_sha256',contract),('hardware_sha256',hardware),('scene_sha256',scene)]:
        if hashlib.sha256(path.read_bytes()).hexdigest()!=p[key]:raise ValueError('frozen input changed: '+key)
    c=json.loads(contract.read_text());h=json.loads(hardware.read_text())
    if c['policy_onnx_sha256']!=p['policy_sha256'] or h['controller']['damiao_embedded_kd_max']!=p['maximum_embedded_kd']:
        raise ValueError('actor or qualified embedded damping ceiling changed')
    allowed=np.asarray(p['tested_requested_commands'],float)
    for case in json.loads(commands.read_text()):
        last=-math.inf
        for stage in case.get('stages',[]):
            stamp=float(stage['at_seconds'])
            if not math.isfinite(stamp) or stamp<p['zero_command_seconds'] or stamp<=last:
                raise ValueError('stages must preserve the zero-command startup prefix and increase')
            last=stamp
        for command in [case['command'],*[s['command'] for s in case.get('stages',[])]]:
            value=np.asarray(command,float)
            if value.shape!=(3,) or not np.isfinite(value).all() or not np.any(np.all(np.isclose(allowed,value,rtol=0,atol=1e-9),axis=1)):
                raise ValueError('requested command is outside the explicitly tested set')
    settings=p['timing_cases'][timing]
    args=[sys.executable,str(Path(__file__).with_name('simulate_supermimic_drives.py')),
        '--contract',str(contract),'--hardware',str(hardware),'--scene',str(scene),
        '--commands',str(commands),'--output-dir',str(output),'--modes','runtime_strict',
        '--seconds','50','--workers','4','--seeds',*[str(x) for x in seeds],
        '--handoff','--handoff-seconds',str(p['handoff_seconds']),
        '--command-start-seconds',str(p['zero_command_seconds']),
        '--command-linear-accel',str(p['linear_command_acceleration_m_s2']),
        '--command-yaw-accel',str(p['yaw_command_acceleration_rad_s2'])]
    if p['residual_damping_compensation']:args+=['--damping-compensation']
    for name in ('delay_ms','feedback_ms','lag_ms'):args+=['--'+name.replace('_','-'),str(settings[name])]
    return args

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('profile','contract','hardware','scene','commands','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--timing',choices=('nominal','pressure'),default='nominal')
    parser.add_argument('--seeds',nargs='+',type=int,default=[101,202,303]);parser.add_argument('--dry-run',action='store_true')
    a=parser.parse_args();args=checked_arguments(a.profile,a.contract,a.hardware,a.scene,a.commands,a.output,a.timing,a.seeds)
    if a.dry_run:print(json.dumps(args,indent=2))
    else:subprocess.run(args,check=True)
if __name__=='__main__':main()
