#!/usr/bin/env bash
set -euo pipefail
# Install as run-stage.sh beside operator-test.sh.
stage=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
base=$(dirname -- "$stage")
mode=${1:?expected dry, fault, or powered}
session=${2:?expected session directory}
validation=${3:-$session}
for directory in "$session" "$validation"; do
  case "$directory" in "$stage"/runs/*) ;; *) echo 'Invalid session or validation directory'; exit 2;; esac
done
case "$mode" in
  dry) extra=(); result=dry ;;
  fault) extra=(--deadline-fault); result=fault ;;
  powered) extra=(--enable-suspended --passed-disabled "$validation/dry" --passed-deadline-fault "$validation/fault" --standing-body-checkpoint "$base/training-full31-1110/powered-zero-01"); result=powered ;;
  *) exit 2 ;;
esac
export PYTHONPATH="$base/live-shadow-1700/src"
export LD_LIBRARY_PATH="$stage/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
bash "$stage/ensure-can-scheduler.sh"
exec /home/tony/open_sprite_runtime/.venv/bin/python "$stage/tools/run_supermimic_commissioning.py" \
  --runtime-root "$base/live-shadow-1700" \
  --bundle "$base/jetson-run002-cpu-only/bundle" \
  --hardware "$base/live-shadow-1700/hardware.json" \
  --joint-limits "$base/strict-sm-20261109/sprite0825_urdf_limit_candidates.json" \
  --build "$stage/build" --icm "$base/zero-gain-1618/build/icm/sprite_icm45686" \
  --lock "$base/commissioning.lock" --output "$session/$result" \
  --tracked-startup "$stage/reviewed-profile" --training-full-body --direct-policy-start --coherent-action-send --all-joint-500hz --sling-window --walk-025 \
  --operator-status "$session/operator-status.log" "${extra[@]}"
