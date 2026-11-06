"""Native CPU inference at 50 Hz on recorded states; never opens CAN or IMU."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import platform
import subprocess

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.supermimic_replay import (
    checked_model, file_sha256, inputs_from_rollout, verify_case,
)


def environment():
    def read(path):
        try:
            return Path(path).read_text().strip().strip("\0")
        except (OSError, UnicodeError):
            return None

    try:
        power = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True,
                               timeout=5, check=False)
        power_mode = power.stdout.strip() if power.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        power_mode = None
    return {
        "host": platform.node(), "architecture": platform.machine(), "platform": platform.platform(),
        "board": read("/proc/device-tree/model"), "jetpack_l4t": read("/etc/nv_tegra_release"),
        "cpu_affinity": sorted(os.sched_getaffinity(0)), "loadavg": read("/proc/loadavg"),
        "cpu_governors": {p.parent.parent.name: read(p) for p in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_governor")},
        "cpu_frequency_khz": {p.parent.parent.name: read(p) for p in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq")},
        "temperatures_millicelsius": {read(p / "type"): read(p / "temp") for p in Path("/sys/class/thermal").glob("thermal_zone*")},
        "power_mode_query": power_mode,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--rollout", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--env", type=int, default=0)
    parser.add_argument("--seconds", type=int, choices=range(10, 121), default=60)
    parser.add_argument("--threads", type=int, choices=range(1, 9), default=1)
    parser.add_argument("--cpus", type=int, nargs="+", required=True)
    args = parser.parse_args()
    if platform.machine() != "aarch64":
        raise ValueError("Jetson timing must be measured on the aarch64 target")
    before = environment()
    if not before["board"] or "Jetson" not in before["board"]:
        raise ValueError("target device tree does not identify a Jetson")
    if not set(args.cpus).issubset(os.sched_getaffinity(0)) or args.threads > len(set(args.cpus)):
        raise ValueError("CPU affinity cannot service the requested inference threads")
    os.sched_setaffinity(0, args.cpus)  # This process/children only; no system policy changes.
    contract = PolicyContract.load(args.contract)
    checked_model(contract)
    summary = json.loads(args.summary.read_text())
    if summary["checkpoint_sha256"] != contract.data["checkpoint_sha256"]:
        raise ValueError("recording and actor checkpoints differ")
    with np.load(args.rollout, allow_pickle=False) as source:
        rollout = {key: source[key] for key in ("proprioception", "commands")}
    inputs = inputs_from_rollout(rollout, contract, args.env)
    count = args.seconds * 50
    stimulus = np.tile(inputs, (math.ceil(count / len(inputs)), 1))[:count]
    report = verify_case(contract, stimulus, args.binary, args.output_dir, handoff=True,
                         benchmark=True, threads=args.threads, warmup=100)
    report.update({
        "created_at": datetime.now(timezone.utc).isoformat(),
        "environment_before": before, "environment_after": environment(),
        "cpu_affinity": sorted(os.sched_getaffinity(0)), "seconds": args.seconds,
        "source_rollout_sha256": file_sha256(args.rollout), "source_env": args.env,
        "source_frames": len(inputs), "recorded_states_repeat": count > len(inputs),
        "timing_meets_20ms_under_test": report["timing"]["deadline_misses"] == 0,
        "system_power_mode_changed": False,
        "scope": "Native observation+CPU inference+handoff on memory-resident recorded states at 50Hz. Excludes live sensor IPC, CAN, physical dynamics and motor control. Python parity runs after native timing finishes.",
    })
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "timing": report["timing"],
        "timing_meets_20ms_under_test": report["timing_meets_20ms_under_test"],
        "cpu_affinity": report["cpu_affinity"], "output": str(args.output_dir)}), flush=True)
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
