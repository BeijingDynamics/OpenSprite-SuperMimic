"""File-only 10-frame fixtures and native/Python recurrence verification."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import struct
import subprocess

import numpy as np

from .contracts import PolicyContract
from .handoff import MeasuredPoseActionHandoff
from .policy_shadow import PolicyObservationHistory

PROFILE = "supermimic_v5_10f"
MAGIC = b"SPMIM001"
NATIVE_RECORD = np.dtype([
    ("obs", "<f4", (993,)), ("raw", "<f4", (31,)),
    ("action", "<f8", (31,)), ("target", "<f8", (31,)),
])


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def checked_model(contract: PolicyContract) -> Path:
    contract.validate_for_replay(PROFILE)
    model = contract.path.parent / contract.data["policy_onnx"]
    if file_sha256(model) != contract.data["policy_onnx_sha256"]:
        raise ValueError("ONNX SHA256 differs from policy contract")
    return model


def write_fixture(path: Path, contract: PolicyContract, inputs: np.ndarray,
                  handoff_steps: int) -> None:
    contract.validate_for_replay(PROFILE)
    inputs = np.asarray(inputs, dtype="<f8")
    if inputs.ndim != 2 or inputs.shape[1] != 71 or not 0 < len(inputs) <= 10000:
        raise ValueError("fixture must contain 1..10000 recorded 71-value states")
    if not np.isfinite(inputs).all() or not 0 <= handoff_steps <= 100000:
        raise ValueError("invalid fixture values or handoff length")
    with path.open("xb") as stream:
        stream.write(struct.pack("<8s6I", MAGIC, 10, 993, 31, 50,
                                 len(inputs), handoff_steps))
        for name in ("default_joint_pos", "action_offset", "action_scale"):
            stream.write(np.asarray(contract.data[name], dtype="<f8").tobytes())
        stream.write(inputs.tobytes())


def inputs_from_rollout(rollout, contract: PolicyContract, env: int,
                        max_frames: int | None = None) -> np.ndarray:
    p = rollout["proprioception"][:max_frames, env]
    commands = rollout["commands"][:max_frames, env]
    if p.ndim != 2 or p.shape[1] != 99 or commands.shape != (len(p), 3):
        raise ValueError("unexpected joint-run proprioception/command layout")
    default = np.asarray(contract.data["default_joint_pos"], dtype=np.float32)
    # Reconstruct measured q from the recorded relative-q field, in float32.
    return np.concatenate((p[:, :31] + default, p[:, 31:62], p[:, 62:65],
                           p[:, 65:68], commands), axis=1).astype("<f8")


def verify_case(contract: PolicyContract, inputs: np.ndarray, binary: Path,
                output: Path, *, handoff: bool, benchmark: bool = False,
                threads: int = 1, warmup: int = 50, recorded_proprio=None,
                recorded_actions=None, tolerance: float = 1e-4) -> dict:
    import onnxruntime as ort

    model = checked_model(contract)
    output.mkdir(parents=True, exist_ok=False)
    replay_data = copy.deepcopy(contract.data)
    if not handoff:
        replay_data["deployment_handoff_seconds"] = 0.0
    replay_contract = PolicyContract(contract.path, replay_data)
    blend = MeasuredPoseActionHandoff(replay_contract)
    fixture, native_path = output / "input.bin", output / "native.bin"
    write_fixture(fixture, contract, inputs, blend.total_steps)
    command = [str(binary.resolve()), str(model.resolve()), str(fixture.resolve()),
               str(native_path.resolve()), "--threads", str(threads), "--warmup", str(warmup)]
    if benchmark:
        command.append("--benchmark")
    result = subprocess.run(command, text=True, capture_output=True, check=False,
                            timeout=max(120, len(inputs) * .025 + 60))
    (output / "native.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    if result.returncode:
        raise RuntimeError(f"native replay failed ({result.returncode}): {result.stderr}")
    if native_path.stat().st_size != len(inputs) * NATIVE_RECORD.itemsize:
        raise ValueError("native output has unexpected record count/size")
    native = np.fromfile(native_path, dtype=NATIVE_RECORD)
    options = ort.SessionOptions()
    options.intra_op_num_threads = threads
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    session = ort.InferenceSession(str(model), sess_options=options,
                                   providers=["CPUExecutionProvider"])
    if session.get_providers() != ["CPUExecutionProvider"]:
        raise ValueError("replay requires CPU-only inference")
    history = PolicyObservationHistory(contract)
    previous = np.zeros(31, dtype=np.float32)
    offset = np.asarray(contract.data["action_offset"], dtype=np.float64)
    scale = np.asarray(contract.data["action_scale"], dtype=np.float64)
    maximum = {name: 0.0 for name in NATIVE_RECORD.names}
    recorded_max = {"observation": 0.0, "raw_action": 0.0}
    for i, row in enumerate(inputs):
        history.append(row[:31], row[31:62], previous, row[62:65], row[65:68])
        obs = history.observation(row[68:71])
        raw = session.run(None, {session.get_inputs()[0].name: obs[None]})[0][0]
        previous = blend.blend(raw, row[:31])
        target = offset + scale * previous
        for name, reference in zip(NATIVE_RECORD.names, (obs, raw, previous, target)):
            delta = float(np.max(np.abs(native[name][i] - reference)))
            if not np.isfinite(delta):
                raise ValueError("nonfinite native/Python comparison")
            maximum[name] = max(maximum[name], delta)
        if not handoff and recorded_proprio is not None and recorded_actions is not None:
            window = recorded_proprio[np.maximum(np.arange(i - 9, i + 1), 0)]
            expected = np.concatenate([window[:, s].reshape(-1) for s in
                (slice(0, 31), slice(31, 62), slice(68, 99), slice(62, 65), slice(65, 68))] + [row[68:71]])
            recorded_max["observation"] = max(recorded_max["observation"],
                float(np.max(np.abs(native["obs"][i] - expected))))
            recorded_max["raw_action"] = max(recorded_max["raw_action"],
                float(np.max(np.abs(native["raw"][i] - recorded_actions[i]))))
    native_timing = json.loads(Path(str(native_path) + ".timing.json").read_text())
    compare_recorded = not handoff and recorded_proprio is not None and recorded_actions is not None
    passed = all(v <= tolerance for v in maximum.values())
    if compare_recorded:
        passed = passed and all(v <= tolerance for v in recorded_max.values())
    report = {
        "profile": PROFILE, "mode": "recorded_states_native_python_recurrence_no_hardware",
        "frames": len(inputs), "input_dim": 993, "action_dim": 31, "history_frames": 10,
        "handoff_steps": blend.total_steps,
        "native_vs_python_max_abs_error": maximum,
        "native_vs_recorded_simulation_max_abs_error": recorded_max if compare_recorded else None,
        "recorded_comparison_note": "Handoff changes recurrence; original simulator comparison is only applicable with handoff disabled.",
        "tolerance": tolerance, "passed": passed,
        "model_sha256": file_sha256(model), "contract_sha256": file_sha256(contract.path),
        "binary_sha256": file_sha256(binary), "fixture_sha256": file_sha256(fixture),
        "timing": native_timing, "hardware_tx": 0, "live_devices_opened": 0,
        "hardware_qualified": False,
        "scope": "Offline inference on fixed recorded physical states, not closed-loop dynamics or hardware admission.",
    }
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report
