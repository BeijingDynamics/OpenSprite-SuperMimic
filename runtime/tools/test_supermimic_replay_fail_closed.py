"""Malformed/profile-mismatch checks for the file-only native executable."""
import argparse
import json
from pathlib import Path
import struct
import subprocess

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.supermimic_replay import checked_model, write_fixture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--legacy-binary", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--legacy-model", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    contract = PolicyContract.load(args.contract)
    model = checked_model(contract)
    args.output_dir.mkdir(parents=True, exist_ok=False)
    valid = args.output_dir / "valid.bin"
    write_fixture(valid, contract, np.zeros((2, 71)), 0)
    original = valid.read_bytes()
    cases = []

    def check(name, content, expected, *, actor=model, binary=args.binary, extra=(),
              allow_empty_output=False):
        fixture = args.output_dir / f"{name}.bin"
        fixture.write_bytes(content)
        output = args.output_dir / f"{name}.output.bin"
        run = subprocess.run([str(binary.resolve()), str(actor.resolve()),
            str(fixture.resolve()), str(output.resolve()), *extra], text=True,
            capture_output=True, timeout=30)
        output_bytes = output.stat().st_size if output.exists() else None
        output_ok = not output.exists() or (allow_empty_output and output_bytes == 0)
        passed = run.returncode != 0 and expected in run.stderr and output_ok
        cases.append({"name": name, "passed": passed, "returncode": run.returncode,
                      "stderr": run.stderr.strip(), "output_created": output.exists(),
                      "output_bytes": output_bytes})

    check("legacy-magic", b"SPPOL001" + original[8:], "profile magic mismatch")
    for offset, value, name in ((8, 8, "history"), (12, 795, "width"),
                                 (16, 30, "actions"), (20, 100, "rate")):
        data = bytearray(original)
        struct.pack_into("<I", data, offset, value)
        check(f"wrong-{name}", data, "profile dimensions/rate mismatch")
    data = bytearray(original)
    struct.pack_into("<I", data, 24, 0)
    check("zero-count", data, "count/handoff out of bounds")
    check("truncated", original[:-1], "truncated replay fixture")
    check("trailing", original + b"extra", "trailing fixture bytes")
    data = bytearray(original)
    struct.pack_into("<d", data, 32 + 3 * 31 * 8, float("nan"))
    check("nan-state", data, "nonfinite recorded state")
    data = bytearray(original)
    struct.pack_into("<d", data, 32 + 2 * 31 * 8, 0)
    check("zero-scale", data, "invalid affine configuration")
    check("legacy-model", original, "model shape/type mismatch", actor=args.legacy_model)
    # The unchanged legacy parser opens an empty output before checking its magic.
    check("legacy-binary", original, "bad input magic", binary=args.legacy_binary,
          allow_empty_output=True)
    check("unknown-mode", original, "unknown or incomplete option", extra=("--powered",))
    check("invalid-threads", original, "option out of bounds", extra=("--threads", "0"))
    report = {"passed": all(c["passed"] for c in cases), "cases": cases,
              "hardware_tx": 0, "live_devices_opened": 0}
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
