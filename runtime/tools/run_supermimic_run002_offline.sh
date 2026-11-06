#!/usr/bin/env bash
# Recorded-data replay only. No hardware setup, CAN, physical IMU or motor enable.
set -euo pipefail
if [ "$#" -ne 4 ]; then
  echo 'usage: run_supermimic_run002_offline.sh BUNDLE ROLLOUT SUMMARY NEW_OUTPUT_DIR' >&2
  exit 2
fi
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python_bin="${SPRITE_REPLAY_PYTHON:-.venv/bin/python}"
build_dir="${SPRITE_REPLAY_BUILD:-build/supermimic_offline}"
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"
"$python_bin" -m open_sprite_runtime.supermimic_run002 verify --bundle "$1"
if [ ! -x "$build_dir/sprite_supermimic_replay" ]; then
  bash tools/build_supermimic_replay.sh
fi
"$python_bin" tools/verify_supermimic_replay.py \
  --contract "$1/contract.json" --rollout "$2" --summary "$3" \
  --binary "$build_dir/sprite_supermimic_replay" --output-dir "$4" --handoff both
