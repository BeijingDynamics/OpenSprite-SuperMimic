#!/usr/bin/env bash
# Fixed-zero powered commissioning. The preview-only Xbox path is not used.
set -euo pipefail
stage=/home/tony/supermimic-pm01-xbox-1116
live=/home/tony/supermimic-pm01-live-1116
base=/home/tony/supermimic-run002-20261106
python=/home/tony/open_sprite_runtime/.venv/bin/python
export PYTHONPATH="$live/runtime/src"
export LD_LIBRARY_PATH="$stage/build"

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check ) ]]; then
  echo 'Usage: operator-stand-zero.sh [--check]' >&2
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
disabled = stage / 'stand-zero-disabled-01'
fault = stage / 'stand-zero-fault-01'
reports = [json.loads((p / 'commissioning.json').read_text()) for p in (disabled, fault)]
for report, mode in zip(reports, ('disabled', 'deadline_fault')):
    if not (report['passed'] and report['mode'] == mode
            and report['duration_s'] == 30.5 and report['motor_enable_attempts'] == 0
            and report['final_disabled_count'] == 31
            and report['training_full_body'] and report['direct_policy_start']
            and report['coherent_action_send'] and report['action_history'] == 'policy_output'
            and report['walk_speed_m_s'] == 0
            and report['velocity_command'] == [0.0, 0.0, 0.0]
            and report['command_source'] == 'scripted'):
        raise SystemExit('Fixed-zero same-build evidence is incomplete. No enable allowed.')
if reports[0]['fingerprints'] != reports[1]['fingerprints']:
    raise SystemExit('Disabled and fault fingerprints differ. No enable allowed.')
inputs = np.fromfile(disabled / 'native.bin.inputs', dtype='<f8').reshape(-1, 71)
if inputs.shape != (1526, 71) or not np.all(inputs[:, 68:71] == 0):
    raise SystemExit('Fixed-zero policy input verification failed.')
audit = json.loads((disabled / 'action-tx-audit.json').read_text())
if not audit['passed'] or audit['final_disabled_count'] != 31:
    raise SystemExit('Motor TX audit is incomplete.')
fingerprints = reports[0]['fingerprints']
paths = [stage / 'tools/run_supermimic_commissioning.py',
         stage / 'tools/action_tx_audit.py', live / 'bundle/policy.onnx',
         stage / 'build/sprite_can_shadow', stage / 'build/sprite_supermimic_protected_ipc',
         base / 'zero-gain-1618/build/icm/sprite_icm45686']
for path in paths:
    if hashlib.sha256(path.read_bytes()).hexdigest() != fingerprints[path.name]:
        raise SystemExit('Qualified file changed: ' + str(path))
if Path('/sys/module/kcan/srcversion').read_text().strip() != fingerprints['loaded_kcan_srcversion']:
    raise SystemExit('Qualified CAN timestamp driver is not loaded.')
print('Fixed-zero disabled and deadline evidence verified. No motor enable sent.', flush=True)
PY
"$stage/build/sprite_can_shadow" --check-commissioning-scheduler 2
if [[ ${1:-} == --check ]]; then
  echo 'Read-only preflight passed. No CAN/IMU session was started.'
  exit 0
fi

echo 'POWERED ZERO-COMMAND TEST: new PM01 policy, full training gains on all 31 joints.'
echo 'Xbox input is not connected to this test. All velocity commands stay zero.'
echo 'IMU stationary bias calibration takes about 15 seconds; it does not reset the robot to level.'
echo 'After enable, policy takes over directly with fresh history and no gain/target ramp.'
echo 'Control lasts up to 30.5 seconds: keep sling support for the first 20 seconds.'
echo 'Only after the standing-check message, slowly slacken the sling if the robot is supporting itself.'
echo 'The message is a sensor check, not proof of unsupported balance. Retain fall protection throughout.'
echo 'Feet must be on the floor, nobody touching the robot, and you must be at the motor power switch.'
echo 'Release the sling only from outside the movement area. Retain support if posture looks wrong.'
echo 'If disable feedback is incomplete, feedback is lost, or any motor stays green after exit, CUT MOTOR POWER.'
read -r -p 'Confirm these conditions by typing READY; anything else exits: ' answer
if [[ "$answer" != READY ]]; then
  echo 'Cancelled. No motor enable sent.'
  exit 0
fi

session="$stage/stand-powered-$(date +%Y%m%d-%H%M%S)-$$"
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
  --passed-disabled "$stage/stand-zero-disabled-01" \
  --passed-deadline-fault "$stage/stand-zero-fault-01" --enable-suspended
