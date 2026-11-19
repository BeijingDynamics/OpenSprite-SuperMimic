#!/usr/bin/env bash
# Bounded Xbox commissioning with a new LT permission after standing admission.
set -euo pipefail
stage=/home/tony/supermimic-pm01-cpufreq-1117
live=/home/tony/supermimic-pm01-live-1116
base=/home/tony/supermimic-run002-20261106
python=/home/tony/open_sprite_runtime/.venv/bin/python
export PYTHONPATH="$live/runtime/src"
export LD_LIBRARY_PATH="$stage/build"

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check ) ]]; then
  echo 'Usage: operator-xbox.sh [--check]' >&2
  exit 2
fi

# This preflight reads evidence and checks the scheduler without CAN/IMU access.
"$python" - "$stage" "$live" "$base" <<'PY'
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

stage, live, base = map(Path, sys.argv[1:])
disabled = stage / 'fixed-disabled-01'
fault = stage / 'fixed-fault-02'
reports = [json.loads((p / 'commissioning.json').read_text()) for p in (disabled, fault)]
for report, mode in zip(reports, ('disabled', 'deadline_fault')):
    frequency = report.get('cpu_frequency', {})
    if (frequency.get('requested_khz') != 1344000
            or frequency.get('released', {}).get('status') != 'restored'
            or frequency['released']['after'] != frequency['released']['before']):
        raise SystemExit('CPU frequency lease or restoration evidence is incomplete.')
    if not (report['passed'] and report['mode'] == mode
            and report['duration_s'] == 54.0 and report['motor_enable_attempts'] == 0
            and report['final_disabled_count'] == 31
            and report['training_full_body'] and report['direct_policy_start']
            and report['coherent_action_send'] and report['action_history'] == 'policy_output'
            and report['walk_speed_m_s'] == 0
            and report['velocity_command'] is None
            and report['command_source'] == 'xbox_control'):
        raise SystemExit('Xbox same-build evidence is incomplete. No enable allowed.')
if reports[0]['fingerprints'] != reports[1]['fingerprints']:
    raise SystemExit('Disabled and fault fingerprints differ. No enable allowed.')
inputs = np.fromfile(disabled / 'native.bin.inputs', dtype='<f8').reshape(-1, 71)
if (inputs.shape != (2701, 71) or not np.all(inputs[:1002, 68:71] == 0)
        or not np.all(inputs[2630:, 68:71] == 0)):
    raise SystemExit('Xbox bounded-window input verification failed.')
audit = json.loads((disabled / 'action-tx-audit.json').read_text())
if not audit['passed'] or audit['final_disabled_count'] != 31:
    raise SystemExit('Motor TX audit is incomplete.')
for directory in (disabled, fault):
    can = json.loads((directory / 'can.json').read_text())
    if not can.get('xbox_body_admission') or can.get('walk_joint_settling_required', True):
        raise SystemExit('Xbox body-only admission was not verified.')
fingerprints = reports[0]['fingerprints']
paths = [stage / 'tools/run_supermimic_commissioning.py',
         stage / 'tools/cpu_frequency_client.py', stage / 'tools/cpu_frequency_guard.py',
         stage / 'tools/action_tx_audit.py', live / 'bundle/policy.onnx',
         stage / 'build/sprite_can_shadow', stage / 'build/sprite_supermimic_protected_ipc',
         base / 'zero-gain-1618/build/icm/sprite_icm45686']
for path in paths:
    if hashlib.sha256(path.read_bytes()).hexdigest() != fingerprints[path.name]:
        raise SystemExit('Qualified file changed: ' + str(path))
if Path('/sys/module/kcan/srcversion').read_text().strip() != fingerprints['loaded_kcan_srcversion']:
    raise SystemExit('Qualified CAN timestamp driver is not loaded.')
print('Xbox disabled and deadline evidence verified. No motor enable sent.', flush=True)
PY
if [[ ${1:-} != --check ]]; then
  bash "$stage/ensure-can-scheduler.sh"
fi
"$stage/build/sprite_can_shadow" --check-commissioning-scheduler 2
if [[ ${1:-} == --check ]]; then
  echo 'Read-only preflight passed. No CAN/IMU session was started.'
  exit 0
fi

echo 'POWERED XBOX TEST: new PM01 policy, full training gains on all 31 joints.'
echo 'Inference CPU 4/5 automatically hold 1344 MHz for this session and restore their prior settings on exit.'
echo 'Left stick: forward up to 0.25 m/s, reverse up to 0.05 m/s. Right stick: yaw up to +/-0.20 rad/s.'
echo 'Lateral command is zero. Hold LT to command motion; release LT to decelerate to zero.'
echo 'Forward acceleration is 0.05 m/s^2; braking is 0.10 m/s^2 (up to 2.5 s from full forward speed).'
echo 'Keep the chosen travel area clear and level, with fall protection covering the full movement.'
echo 'A continuous full-forward 30-second input can travel about 7.2 m including braking; allow 8 m for that test.'
echo 'Release LT early enough to stop before any route or fall-protection boundary.'
echo 'IMU stationary bias calibration takes about 15 seconds; it does not reset the robot to level.'
echo 'After enable, policy takes over directly with fresh history and no gain/target ramp.'
echo 'Control lasts up to 54 seconds: keep sling support for the first 20 seconds.'
echo 'Only after the standing-check message, slowly slacken the sling if the robot is supporting itself.'
echo 'At XBOX_WINDOW_OPEN: center sticks, RELEASE LT, then hold LT and move the sticks.'
echo 'Pre-held LT cannot start motion. The input window is a full 30 s, followed by automatic braking and shutdown.'
echo 'Do not push the robot during this command-control trial. LT release is command braking, not motor disable.'
echo 'The message is a sensor check, not proof of unsupported balance. Retain fall protection throughout.'
echo 'Feet must be on the floor, nobody touching the robot, and you must be at the motor power switch.'
echo 'Release the sling only from outside the movement area. Retain support if posture looks wrong.'
echo 'If disable feedback is incomplete, feedback is lost, or any motor stays green after exit, CUT MOTOR POWER.'
read -r -p 'Confirm these conditions by typing READY; anything else exits: ' answer
if [[ "$answer" != READY ]]; then
  echo 'Cancelled. No motor enable sent.'
  exit 0
fi

session="$stage/xbox-powered-$(date +%Y%m%d-%H%M%S)-$$"
echo "Evidence: $session"
exec "$python" "$stage/tools/run_supermimic_commissioning.py" \
  --runtime-root "$base/live-shadow-1700" --bundle "$live/bundle" \
  --hardware "$base/live-shadow-1700/hardware.json" \
  --joint-limits "$base/strict-sm-20261109/sprite0825_urdf_limit_candidates.json" \
  --build "$stage/build" --icm "$base/zero-gain-1618/build/icm/sprite_icm45686" \
  --lock "$base/commissioning.lock" --output "$session" \
  --tracked-startup "$base/walk025-sync-en-1111/reviewed-profile" \
  --training-full-body --direct-policy-start --coherent-action-send \
  --sling-window --operator-status "$stage/operator-status.log" \
  --passed-disabled "$stage/fixed-disabled-01" \
  --passed-deadline-fault "$stage/fixed-fault-02" --gamepad-control-device /dev/input/js0 \
  --cpu-frequency-socket /run/open-sprite-cpu-frequency/lease.sock --enable-suspended
