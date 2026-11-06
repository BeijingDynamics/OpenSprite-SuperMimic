#!/usr/bin/env bash
# Build file-only actors and math tests. Reuses the installed ORT; no device setup.
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
  --target test_policy_core test_supermimic_core sprite_policy_replay sprite_supermimic_replay sprite_supermimic_ipc_readonly
ctest --test-dir "$build_dir" --output-on-failure -R '^(policy_core|supermimic_core)$'
echo 'Built offline replay/test targets only; no CAN/IMU or powered binaries installed.'
