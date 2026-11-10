"""Freeze the reviewed run001 simulation baseline, without enabling hardware."""
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import shutil
import sys
import tarfile

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parent
LABEL = 'sprite-v5-supermimic-sim-v0.1-20261105'
RUNTIME_COMMIT = '4515206c24404e18a2d5c168eccdb7468aea6ab7'
MODEL_SHA = 'cdc51965fe3aca449c45b808e041424f47f46fefe9f5805def14651f9258e953'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')


def main():
    run = ROOT / 'results/outputs/joint-paper-v5-001'
    envelope = ROOT / 'results/outputs/speed-envelope-001'
    backward = ROOT / 'results/outputs/backward-demo-001'
    contract = json.loads((run / 'export-final/contract.json').read_text())
    assert sha(run / 'latest.pt') == MODEL_SHA == contract['checkpoint_sha256']
    assert sha(run / 'export-final/policy.onnx') == contract['policy_onnx_sha256']
    assert contract['hardware_qualified'] is False
    summary = json.loads((envelope / 'envelope_summary.json').read_text())
    assert summary['checkpoint_sha256'] == MODEL_SHA
    back = json.loads((backward / 'trial/summary.json').read_text())
    assert back['checkpoint_sha256'] == MODEL_SHA
    assert all(x['survived'] for x in back['results'])
    sys.path.insert(0, str(WORKSPACE / 'open_sprite_runtime/src'))
    from open_sprite_runtime.contracts import PolicyContract, RuntimeTiming
    policy = PolicyContract.load(run / 'export-final/contract.json')
    try:
        policy.validate_for_hardware(RuntimeTiming(50, 500, 1000))
    except ValueError as error:
        rejection = str(error).split('; ')
    else:
        raise RuntimeError('Unexpected hardware acceptance; review baseline qualification')
    dest = ROOT / 'releases' / LABEL
    dest.mkdir(parents=True, exist_ok=False)

    def copy(source, target):
        target = dest / target
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)

    files = {
        'model_026951.pt': run / 'latest.pt',
        'deploy/policy.onnx': run / 'export-final/policy.onnx',
        'deploy/contract.json': run / 'export-final/contract.json',
        'training/config.json': run / 'config.json',
        'training/interface.json': run / 'interface.json',
        'training/source-manifest.json': run / 'run001-source-manifest.json',
        'training/v5_locomotion.npz': ROOT / 'results/data/motions/contact_v5_v11/v5_locomotion.npz',
        'evidence/speed-envelope.json': envelope / 'envelope_summary.json',
        'evidence/speed-envelope.csv': envelope / 'envelope_summary.csv',
        'evidence/speed-envelope-report.md': envelope / 'SPEED_ENVELOPE_REPORT.md',
        'evidence/live-command-changes-summary.json': envelope / 'live-command-changes/summary.json',
        'evidence/backward-summary.json': backward / 'trial/summary.json',
        'evidence/backward-commands.json': backward / 'commands.json',
        'evidence/runtime-replay-parity-original.json': run / 'runtime_replay_parity.json',
        'evidence/LOW_SPEED_AND_INITIAL_POSE_REVIEW.md': ROOT / 'LOW_SPEED_AND_INITIAL_POSE_REVIEW.md',
        'evidence/BACKWARD_CONTROL.md': ROOT / 'BACKWARD_CONTROL.md',
        'videos/backward-and-switching.mp4': backward / 'backward-and-switching.mp4',
        'videos/live-command-changes-fullbody.mp4': envelope / 'live-command-changes-fullbody.mp4',
        'source/export_joint.py': ROOT / 'scripts/export_joint.py',
        'source/evaluate_joint.py': ROOT / 'scripts/evaluate_joint.py',
        'source/render_velocity.py': ROOT / 'scripts/render_velocity.py',
    }
    for target, source in files.items():
        copy(source, target)
    with tarfile.open(ROOT / 'artifacts/run001-important.tar') as archive:
        name = 'outputs/joint-paper-v5-001/run001-source-snapshot.tar.gz'
        data = archive.extractfile(name).read()
    (dest / 'training/run001-source-snapshot.tar.gz').write_bytes(data)
    # Read only, with no extraction of filesystem paths from the nested archive.
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as archive:
        original_sources = {m.name: hashlib.sha256(archive.extractfile(m).read()).hexdigest()
                            for m in archive.getmembers() if m.isfile()}
    write(dest / 'training/source-snapshot-contents.json', original_sources)
    assert sha(dest / 'training/v5_locomotion.npz') == 'ccbecf8f37037398ffc62750b6df0508c05f063bb2c38ed0fd596e26de762989'
    audit = {
        'runtime_repository': 'https://github.com/BeijingDynamics/open_sprite_runtime',
        'runtime_commit': RUNTIME_COMMIT,
        'upstream_head_checked': '2026-11-05 via GitHub commits API; matches local clone',
        'read_only_contract_check': True,
        'hardware_tx_performed': False,
        'hardware_contract_accepted': False,
        'contract_rejections': rejection,
        'native_runtime': {'history_frames': 8, 'input_dim': 795, 'policy_hz': 50},
        'this_policy': {'history_frames': 10, 'input_dim': 993, 'policy_hz': 50},
        'original_replay_scope': 'Python PolicyObservationHistory assembly plus ONNX CPU replay; NOT native C++ inference or powered hardware',
        'original_replay_wording_correction': 'The original report contains the superseded phrase policy gait failed; see LOW_SPEED_AND_INITIAL_POSE_REVIEW.md. Numeric parity remains valid.',
        'blocking_work': [
            'Add a separate 10-frame/993-input native actor profile and admission schema; preserve the existing 8-frame candidate.',
            'Validate native C++ observation/action/history/handoff parity and target Jetson CPU inference timing.',
            'Replay the real ankle linkage, motor torque-speed limits, 500Hz ankle/50Hz other-motor schedule, sensor delay and startup protections in closed-loop simulation.',
            'Confirm actual motor zeros/directions, IMU frame, current gains and hardware revision before protected standing and bounded walk tests.',
        ],
        'reverse_hardware_status': 'Existing Xbox powered path blocks reverse; simulation support does not authorize enabling it.',
        'notes': 'Matching URDF and 50Hz policy timing are insufficient for hardware admission. Existing G74 hardware results do not transfer to this SuperMimic actor.',
    }
    write(dest / 'evidence/runtime-readiness.json', audit)
    existing = [ROOT / 'artifacts/run001-important.tar',
                envelope / 'seed-101/rollout.npz', envelope / 'seed-202/rollout.npz',
                envelope / 'seed-303/rollout.npz', backward / 'trial/rollout.npz']
    references = [{'workspace_path': p.relative_to(WORKSPACE).as_posix(),
                   'bytes': p.stat().st_size, 'sha256': sha(p)} for p in existing]
    manifest = {
        'label': LABEL,
        'human_label': 'Open Sprite v5 SuperMimic simulation baseline v0.1',
        'created_at': datetime.now(timezone.utc).isoformat(),
        'status': 'simulation_baseline_frozen_hardware_not_admitted',
        'hardware_qualified': False,
        'kind': 'local artifact label; no upstream Git tag or release published',
        'iteration': 26951,
        'checkpoint_sha256': MODEL_SHA,
        'onnx_sha256': contract['policy_onnx_sha256'],
        'robot_asset_commit': 'afc882bcac3bac551f3c0c80b454b366c836dd83',
        'robot_urdf_sha256': contract['asset_urdf_sha256'],
        'robot_mjcf_sha256': contract['asset_mjcf_sha256'],
        'runtime_commit': RUNTIME_COMMIT,
        'capabilities_observed_in_simulation': ['stand', 'forward', 'backward', 'lateral', 'turn', 'continuous_command_switching'],
        'speed_envelope_trials': {'cases': summary['cases'], 'seeds': summary['seeds'], 'seconds_each': summary['seconds'],
                                  'survived': sum(r['survived'] for r in summary['results']),
                                  'total': sum(r['trials'] for r in summary['results'])},
        'backward_demo_trials': {'survived': sum(r['survived'] for r in back['results']), 'total': len(back['results']), 'seconds_each': back['seconds']},
        'training_reward_revision': 'original run001 a6_root_legacy_v1; subsequent paper_a6_torso_heading_v2 fixes are NOT trained into these weights',
        'training_physics': 'serial-joint simulation, torque lag disabled; no validated physical parallel-ankle envelope',
        'limitations': ['Velocity undershoot and yaw drift/asymmetry', 'Low-clearance contact transitions need synchronized verification',
                        'No physical hardware admission', 'Not a completed paper benchmark replication'],
        'external_preserved_evidence': references,
    }
    write(dest / 'LABEL.json', manifest)
    readme = '''# Open Sprite v5 SuperMimic simulation baseline v0.1

Label: `sprite-v5-supermimic-sim-v0.1-20261105`. Frozen run001 update 26951; this is a local artifact label, not a GitHub publication.

Forward, reverse, lateral, turning and continuous command switching were tested in the recorded simulation conditions. All 147 trials (49 commands, three seeds, 30 seconds each) survived; four independent reverse/switching demonstrations completed 32 seconds. Survival does not establish velocity accuracy, contact quality or hardware qualification. Tracking deficits, yaw drift and left/right turning asymmetry remain.

## Hardware assessment at this baseline

This is a frozen starting point for runtime adaptation and Sim2Real validation. It cannot directly replace the runtime model for hardware walking.

1. This model uses 50 Hz, 10 history frames and 993 inputs; the inspected native runtime was fixed to eight frames and 795 inputs. Python-history/ONNX replay error of 2.38e-6 validates only that offline path. Add a native 10-frame profile; do not truncate or pad inputs to imply compatibility.
2. Training uses serial ankle joints. Real parallel-ankle torque/speed envelopes were not validated. Matching the 50 Hz policy rate does not establish equivalent 500 Hz ankle transport, 50 Hz other-motor transport or sensor latency. Torque lag was disabled in this training run.
3. The runtime hardware-admission check rejected this contract; see evidence/runtime-readiness.json. G74 hardware evidence does not qualify this policy. The original Xbox hardware entry still disallows reverse commands.

Next steps: add a separate native 10-frame profile, validate C++ replay and target-Jetson latency, simulate real actuator/sensor timing, then verify calibration before protected standing and short low-speed trials. This label grants no powered-test authorization.

## Retained artifacts

`model_026951.pt` is the training checkpoint. `deploy/` retains the original ONNX and contract with hardware_qualified=false. `training/` contains the original configuration, 169 KB training-source snapshot and locally prepared 40-second v5 reference. `source/` contains evaluation/export code, separately from the training snapshot. Later code fixes two A.6 reward issues without rewriting these weights.

`videos/` retains continuous-control and reverse demonstrations. `evidence/` contains measurement summaries and the low-speed gait review. Low foot clearance is not automatically classified as sliding failure; consult the review before relying on earlier qualitative claims.

Generic raw motion libraries and dependency caches are excluded. Full rollouts and earlier important archives remain local; LABEL.json lists their paths and SHA256 hashes. Reproduction also requires the original robot assets and dependencies. This directory is not a ready-to-deploy hardware package.

`SHA256SUMS.json` covers the frozen contents; `VERIFIED.json` records local verification. Create a new label for improvements rather than overwriting this version.
'''
    (dest / 'README.md').write_text(readme, encoding='utf-8')
    copy(Path(__file__), 'source/freeze_sim_baseline.py')
    hashes = {p.relative_to(dest).as_posix(): sha(p) for p in sorted(dest.rglob('*')) if p.is_file()}
    write(dest / 'SHA256SUMS.json', hashes)
    failures = [name for name, digest in hashes.items() if sha(dest / name) != digest]
    write(dest / 'VERIFIED.json', {'verified_at': datetime.now(timezone.utc).isoformat(),
                                  'files': len(hashes), 'failed': failures})
    assert not failures
    write(ROOT / 'CURRENT_BASELINE.json', {'label': LABEL, 'path': dest.relative_to(ROOT).as_posix(),
                                         'checkpoint_sha256': MODEL_SHA, 'hardware_qualified': False,
                                         'status': manifest['status']})
    print(json.dumps({'label': LABEL, 'path': str(dest), 'verified_files': len(hashes),
                      'bytes': sum(p.stat().st_size for p in dest.rglob('*') if p.is_file()),
                      'hardware_qualified': False}, ensure_ascii=False))


if __name__ == '__main__':
    main()
