"""Verify native SuperMimic recurrence on physical simulation recordings."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import platform

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.supermimic_replay import (
    checked_model, file_sha256, inputs_from_rollout, verify_case,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--rollout", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--envs", type=int, nargs="+")
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--handoff", choices=("off", "contract", "both"), default="both")
    parser.add_argument("--benchmark", action="store_true")
    parser.add_argument("--threads", type=int, choices=range(1, 9), default=1)
    parser.add_argument("--warmup", type=int, choices=range(0, 1001), default=50)
    args = parser.parse_args()
    contract = PolicyContract.load(args.contract)
    checked_model(contract)
    summary = json.loads(args.summary.read_text())
    if summary["checkpoint_sha256"] != contract.data["checkpoint_sha256"]:
        raise ValueError("rollout checkpoint identity differs from contract")
    if args.max_frames is not None and args.max_frames < 1:
        raise ValueError("max-frames must be positive")
    args.output_dir.mkdir(parents=True, exist_ok=False)
    reports = []
    with np.load(args.rollout, allow_pickle=False) as source:
        # NPZ decompression happens once, outside all native timing loops.
        rollout = {key: source[key] for key in ("proprioception", "commands", "policy_action", "case_names")}
    envs = args.envs if args.envs is not None else list(range(rollout["proprioception"].shape[1]))
    for env in envs:
        inputs = inputs_from_rollout(rollout, contract, env, args.max_frames)
        for handoff in ([False, True] if args.handoff == "both" else [args.handoff == "contract"]):
            name = f"env-{env:02d}-handoff-{'contract' if handoff else 'off'}"
            report = verify_case(contract, inputs, args.binary, args.output_dir / name,
                handoff=handoff, benchmark=args.benchmark, threads=args.threads, warmup=args.warmup,
                recorded_proprio=rollout["proprioception"][:len(inputs), env],
                recorded_actions=rollout["policy_action"][:len(inputs), env])
            report["case_name"] = str(rollout["case_names"][env])
            reports.append(report)
            print(json.dumps({"case": name, "passed": report["passed"],
                              "errors": report["native_vs_python_max_abs_error"],
                              "recorded": report["native_vs_recorded_simulation_max_abs_error"]}), flush=True)
    aggregate = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "host": platform.node(), "architecture": platform.machine(), "platform": platform.platform(),
        "passed": bool(reports) and all(r["passed"] for r in reports),
        "rollout_sha256": file_sha256(args.rollout), "summary_sha256": file_sha256(args.summary),
        "cases": reports, "hardware_tx": 0, "live_devices_opened": 0, "hardware_qualified": False,
    }
    (args.output_dir / "report.json").write_text(json.dumps(aggregate, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": aggregate["passed"], "cases": len(reports),
                      "architecture": aggregate["architecture"], "output": str(args.output_dir)}), flush=True)
    raise SystemExit(0 if aggregate["passed"] else 1)


if __name__ == "__main__":
    main()
