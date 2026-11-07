#!/usr/bin/env bash
# Build replay/receive-only actors and zero-gain diagnostics; no device setup.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
python_bin="${SPRITE_REPLAY_PYTHON:-.venv/bin/python}"
build_dir="${SPRITE_REPLAY_BUILD:-build/supermimic_offline}"
headers="${SPRITE_ORT_INCLUDE:?Set SPRITE_ORT_INCLUDE to headers matching installed ONNX Runtime}"
libpath="$($python_bin -c 'import onnxruntime,pathlib; p=pathlib.Path(onnxruntime.__file__).parent/"capi"; v=onnxruntime.__version__; f=p/("libonnxruntime.so."+v); assert f.is_file(),f; print(f)')"
mkdir -p "$build_dir"
ln -sfn "$libpath" "$build_dir/libonnxruntime.so.1"
cmake -S native -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release \
  -DSPRITE_ORT_INCLUDE="$headers" -DSPRITE_ORT_LIBRARY="$libpath" \
  -DCMAKE_BUILD_RPATH="$(cd "$build_dir" && pwd)"
cmake --build "$build_dir" --parallel 2 \
  --target test_policy_core test_supermimic_core test_readonly_state_server sprite_zero_gain_can_probe sprite_supermimic_pose_gate sprite_policy_replay sprite_supermimic_replay sprite_supermimic_ipc_readonly
ctest --test-dir "$build_dir" --output-on-failure -R '^(policy_core|supermimic_core|readonly_state_server|zero_gain_codec|supermimic_pose_gate)$'
echo 'Built replay/read-only actors, tests and zero-gain diagnostic. No devices accessed or binaries installed.'
