"""File-only admission cache for the isolated full-body 0.25 m/s stage.

Never opens CAN/IMU or enables motors. Powered commissioning independently
repeats the original fingerprint gates before any hardware transmission.
"""
import argparse
import contextlib
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_reports(session, expected, direct_policy_start=False, coherent_action_send=False):
    """Fail closed on mismatched, failed, incomplete or modified evidence."""
    for directory, mode in (('dry', 'disabled'), ('fault', 'deadline_fault')):
        folder = session/directory
        prior = json.loads((folder/'commissioning.json').read_text())
        if (prior.get('passed') is not True or prior.get('mode') != mode or
            prior.get('duration_s') != 49.5 or prior.get('walk_speed_m_s') != .25 or
            prior.get('training_full_body') is not True or prior.get('tracked_startup') is not True or
            prior.get('sling_window') is not True or prior.get('action_history') != 'policy_output' or
            prior.get('motor_enable_attempts') != 0 or prior.get('final_disabled_count') != 31 or
            prior.get('can_schedule') != dict(cpu=2, policy='SCHED_FIFO', priority=50) or
            prior.get('fingerprints') != expected):
            raise ValueError(f'{directory}: validation identity or result mismatch')
        if prior.get('direct_policy_start', False) != direct_policy_start:
            raise ValueError(f'{directory}: startup mode mismatch')
        if prior.get('coherent_action_send', False) != coherent_action_send:
            raise ValueError(f'{directory}: action send mode mismatch')
        for name in ('motors.tsv','kinematics.tsv','joints.tsv','feedback.bin','target.bin','input.bin'):
            if sha(folder/name) != expected[name]:
                raise ValueError(f'{directory}: archived {name} modified')


def current_fingerprints(stage, coherent_action_send=False):
    base = stage.parent
    sys.path.insert(0, str(base/'live-shadow-1700/src'))
    launcher = stage/'tools/run_supermimic_commissioning.py'
    spec = importlib.util.spec_from_file_location('commission_cache', launcher)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    args = SimpleNamespace(runtime_root=base/'live-shadow-1700',
                           hardware=base/'live-shadow-1700/hardware.json',
                           joint_limits=base/'strict-sm-20261109/sprite0825_urdf_limit_candidates.json',
                           duration=49.5)
    if sha(args.hardware) != module.HARDWARE_SHA or sha(args.joint_limits) != module.LIMITS_SHA:
        raise ValueError('sealed hardware or limits changed; review required')
    bundle = base/'jetson-run002-cpu-only/bundle'
    module.verify_bundle(bundle)
    contract = module.PolicyContract.load(bundle/'contract.json')
    model = module.checked_model(contract)
    with tempfile.TemporaryDirectory(prefix='validation-preflight-',dir=stage) as temp:
        out = Path(temp)
        # Generate exactly the upcoming configuration, not a stale cached TSV.
        with contextlib.redirect_stdout(sys.stderr):
            module.prepare(args, out, contract)
        for name, digest in module.MATCHED_PROFILE_SHA.items():
            source = stage/'reviewed-profile'/name
            if sha(source) != digest:
                raise ValueError('reviewed profile changed: '+name)
            (out/name).write_bytes(source.read_bytes())
        module.set_peak_feedback_caps(out/'motors.tsv')
        for name, digest in module.MATCHED_MAPPING_SHA.items():
            if sha(out/name) != digest:
                raise ValueError('transmission changed: '+name)
        target = module.apply_training_gains(out/'joints.tsv',(out/'target.bin').read_bytes(),contract)
        (out/'target.bin').write_bytes(b'SPMTGT02'+target[8:]+module.startup_joint_roles(contract.data['joint_names']))
        paths = [launcher, model, base/'zero-gain-1618/build/icm/sprite_icm45686',
                 stage/'build/sprite_can_shadow',stage/'build/sprite_supermimic_protected_ipc']
        if coherent_action_send: paths.append(stage/'tools/action_tx_audit.py')
        paths += [out/name for name in ('motors.tsv','kinematics.tsv','joints.tsv','feedback.bin','target.bin','input.bin')]
        result = {path.name: sha(path) for path in paths}
        if coherent_action_send:
            result['loaded_kcan_srcversion'] = Path('/sys/module/kcan/srcversion').read_text().strip()
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage',type=Path,required=True)
    parser.add_argument('--report',type=Path,required=True)
    parser.add_argument('--direct-policy-start',action='store_true')
    parser.add_argument('--coherent-action-send',action='store_true')
    args = parser.parse_args()
    stage = args.stage.resolve()
    report = dict(selected=None, file_only=True, rejected=[])
    try:
        # prepare() child processes write to stdout; reserve selection in JSON.
        expected = current_fingerprints(stage,args.coherent_action_send)
        report['fingerprints'] = expected
        for session in sorted((stage/'runs').glob('*'),reverse=True):
            if not session.is_dir() or session.is_symlink():
                continue
            try:
                check_reports(session,expected,args.direct_policy_start,args.coherent_action_send)
                result = subprocess.run([sys.executable,str(stage/'check-records.py'),str(session)],
                                        capture_output=True,text=True,timeout=30)
                if result.returncode:
                    raise ValueError('retained command/history/disable trace check failed')
                report['selected'] = str(session.resolve())
                report['trace_validation'] = result.stdout
                break
            except (ValueError, OSError, KeyError, subprocess.TimeoutExpired) as error:
                report['rejected'].append(dict(session=session.name,reason=str(error)))
        args.report.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        return 0 if report['selected'] else 1
    except Exception as error:
        report['error'] = str(error)
        args.report.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        print(f'File-only preflight failed: {error}',file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
