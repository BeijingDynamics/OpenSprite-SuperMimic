#!/usr/bin/env bash
# Interactive input/IPC qualification only. No motor-enable mode is exposed.
set -euo pipefail
stage=/home/tony/supermimic-pm01-xbox-1116
live=/home/tony/supermimic-pm01-live-1116
base=/home/tony/supermimic-run002-20261106
export PYTHONPATH="$live/runtime/src"
export LD_LIBRARY_PATH="$stage/build"
session="$stage/manual-$(date +%Y%m%d-%H%M%S)"
echo 'DISABLED-MOTOR PREVIEW ONLY. Keep the robot stationary and retain sling support.'
echo 'Wait for GAMEPAD PREVIEW READY before operating the controller. No sling release is required.'
echo "Evidence: $session"
exec /home/tony/open_sprite_runtime/.venv/bin/python "$stage/tools/run_supermimic_commissioning.py" \
  --runtime-root "$base/live-shadow-1700" --bundle "$live/bundle" \
  --hardware "$base/live-shadow-1700/hardware.json" \
  --joint-limits "$base/strict-sm-20261109/sprite0825_urdf_limit_candidates.json" \
  --build "$stage/build" --icm "$base/zero-gain-1618/build/icm/sprite_icm45686" \
  --lock "$base/commissioning.lock" --output "$session" \
  --tracked-startup "$base/walk025-sync-en-1111/reviewed-profile" \
  --training-full-body --direct-policy-start --coherent-action-send \
  --sling-window --operator-status "$stage/operator-status.log" \
  --gamepad-preview-device /dev/input/js0
