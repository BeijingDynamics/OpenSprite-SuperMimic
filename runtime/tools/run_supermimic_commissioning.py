"""Commission the protected SuperMimic IPC path in an isolated stage on Jetson."""
import argparse
import csv
from datetime import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import threading
import time

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.native_ipc import ordered_name_hash
from open_sprite_runtime.socketcan import audit_socketcan_active_fd_snapshot
from open_sprite_runtime.supermimic_replay import checked_model, file_sha256, write_fixture
from open_sprite_runtime.supermimic_run002 import verify_bundle

GAIN = .02
CAN_CPU = 2
CAN_REALTIME_PRIORITY = 50
ACTOR_CPUS = (4, 5)
SLING_DURATION = 30.5
WALK_DURATION = 49.5
SLING_COUNTDOWN = [f"COUNTDOWN_{seconds}" for seconds in range(9, 0, -1)]
INTERNAL_OPERATOR_EVENTS = {"ENABLED_31", "DRY_ENABLE_NOTICE"}
OPERATOR_MESSAGES = {
    "POLICY_DIRECT": "SuperMimic zero-command takeover started with full training gains on all joints, without a gain ramp. Keep the sling supporting the robot; wait for the standing confirmation.",
    "DRY_POLICY_DIRECT": "Disabled validation: policy history reset; full training-gain targets generated directly. Motors remain disabled.",
    "WAITING_FOR_ENABLE": "Preparing zero-command control. Keep the sling supporting the robot.",
    "POLICY_BLEND": "Starting the SuperMimic zero-command policy ramp. Keep sling support; wait for the standing confirmation.",
    "DRY_POLICY_BLEND": "Disabled validation: previewing the zero-command policy ramp.",
    "NOT_STABLE": "Standing check failed. Keep sling support; shutting down.",
    "SLING_WINDOW_OPEN": "Zero-command standing check passed. Slowly slacken the sling while retaining fall protection. 10 seconds remaining.",
    "DRY_WINDOW_OPEN": "Disabled validation: simulating a 10-second sling-release window. Keep sling support.",
    "WINDOW_END": "The 10-second window has ended; zero-command control is finishing.",
    "WALK_SLING_WINDOW_OPEN": "Body stability check passed. Slowly slacken the sling while retaining fall protection. Recheck in 10 seconds; forward motion follows only if it passes.",
    "DRY_WALK_SLING_WINDOW_OPEN": "Disabled validation: simulating the 10-second sling-release and pre-walk window. Keep sling support.",
    "WALK_RAMP": "Body stability recheck passed. Ramping forward command to {speed:.2f} m/s over 3 seconds.",
    "WALK_FULL": "Holding forward command {speed:.2f} m/s for 10 seconds.",
    "WALK_STOP": "Decelerating the forward command to zero over 3 seconds.",
    "WALK_ZERO": "Velocity command is zero. Retain fall protection; control is finishing.",
    "DISABLED_31": "Individual feedback confirms all 31 motors are disabled.",
    "DISABLE_UNCONFIRMED": "Could not confirm all 31 motors disabled. CUT MOTOR POWER.",
}
OPERATOR_MESSAGES.update({
    f"COUNTDOWN_{seconds}": f"Window remaining: {seconds} seconds." for seconds in range(9, 0, -1)
})
OPERATOR_MESSAGES.update({
    f"DRY_{event}": "Disabled validation. " + OPERATOR_MESSAGES[event].removeprefix("Body stability recheck passed. ")
    for event in ("WALK_RAMP", "WALK_FULL", "WALK_STOP", "WALK_ZERO")
})
HARDWARE_SHA = "9ddf06034f24bbca49cb06f80236caa4cea7aee0040516d11209f848da1f2c10"
# This reviewed soft-limit profile has the same 31 hard joint bounds as run002.
LIMITS_SHA = "a939239f4481aea1ee970a23040077bc29142642d81f327415dfdd4b9393275f"
MATCHED_PROFILE_SHA = {
    "motors.tsv": "7b44abe8c0267fd52d1a57511a4436b977378c29b17227e97d127bfe23f915b1",
    "joints.tsv": "604c8bd60cb3cc5292b4a8b0a6943fb47643f22036968ed7cd9dc9a72a89f4de",
    "target.bin": "a67a8a48166fea2a6c95c08e8c753207734487ec4c1502778a1d43e025a58d81",
}
MATCHED_MAPPING_SHA = {
    "kinematics.tsv": "9b47255137997ae1f0da2845be8b8548dae537c1658741469ffe9d3c6ab38cc9",
    "feedback.bin": "d03e9f0c34f8c7f6182474178927b9749699a54c7885a859fdbeffc82111191c",
}
XBOX_JOINT_GAINS_SHA = "f12823189ebca4b4bf0670e3f7999a5e97d03dccf4c3b8d6a1451cca978e1cb9"


def set_peak_feedback_caps(path):
    header, *lines = path.read_text().splitlines()
    columns = header.split("\t")
    feedback = columns.index("feedback_torque_cap_nm")
    peak = columns.index("mechanical_peak_torque_nm")
    lower = columns.index("torque_min_nm")
    upper = columns.index("torque_max_nm")
    updated = [header]
    for line in lines:
        row = line.split("\t")
        lo, hi, mechanical = (float(row[i]) for i in (lower, upper, peak))
        # Keep the extreme feedback codes detectable when the wire range is
        # smaller than the motor's mechanical peak.
        cap = min(.90 * mechanical, min(-lo, hi) - (hi - lo) / (2 * 4095))
        if not math.isfinite(cap) or not 0. < cap < min(-lo, hi):
            raise ValueError("invalid peak feedback torque envelope")
        row[feedback] = repr(cap)
        updated.append("\t".join(row))
    path.write_text("\n".join(updated) + "\n", encoding="utf-8")


def startup_joint_roles(joint_names):
    # Binary SPMTGT02 role codes match native StartupJoint.
    roles = []
    for name in joint_names:
        if name.startswith("head_") or "_wrist_" in name:
            roles.append(4)
        elif name.startswith("left_ankle_"):
            roles.append(2)
        elif name.startswith("right_ankle_"):
            roles.append(3)
        elif name.startswith("waist_") or "_hip_" in name or name.endswith("_knee_joint"):
            roles.append(1)
        else:
            roles.append(0)
    if [roles.count(code) for code in range(5)] != [8, 10, 2, 2, 9]:
        raise ValueError("unexpected SuperMimic startup joint groups")
    return bytes(roles)


def apply_xbox_leg_waist_gains(source, joint_table, target, joint_names, roles):
    if file_sha256(source) != XBOX_JOINT_GAINS_SHA:
        raise ValueError("sealed Xbox joint gain table changed")
    with source.open() as stream:
        xbox = {row["joint_name"]: row for row in csv.DictReader(stream, delimiter="\t")}
    header, *lines = joint_table.read_text().splitlines()
    columns = header.split("\t")
    kp_col, kd_col = (columns.index(key) for key in ("kp_max", "kd_max"))
    target = bytearray(target)
    gains = np.frombuffer(target, dtype="<f8", offset=16).reshape(4, 31)
    rows = [line.split("\t") for line in lines]
    if [row[0] for row in rows] != joint_names:
        raise ValueError("reviewed Xbox gain overlay joint order mismatch")
    for j, (row, role) in enumerate(zip(rows, roles)):
        if role in (1, 2, 3):
            gain = xbox[row[0]]
            row[kp_col], row[kd_col] = gain["kp_max"], gain["kd_max"]
            gains[2, j], gains[3, j] = float(row[kp_col]), float(row[kd_col])
    joint_table.write_text("\n".join([header] + ["\t".join(row) for row in rows]) + "\n",
                           encoding="utf-8", newline="\n")
    return bytes(target)


def timing_breakdown(out, frames):
    def read_rows(path):
        with (out/path).open() as stream:
            return list(csv.DictReader(stream))

    can = read_rows("can.json.timing.csv")
    actor = read_rows("native.bin.ipc.csv")
    if len(can) != frames or len(actor) != frames:
        raise RuntimeError("incomplete per-frame CAN or actor timing trace")
    maxima = {name: 0 for name in ("can_feedback", "state_ipc", "actor_preprocess",
                                    "inference", "actor_postprocess", "target_ipc", "total")}
    for seq, (c, a) in enumerate(zip(can, actor), 1):
        stamps = [int(row[key]) for row, key in (
            (c, "feedback_ns"), (c, "publish_ns"), (a, "receive_ns"), (a, "infer_start_ns"),
            (a, "infer_end_ns"), (a, "target_ns"), (c, "accept_ns"))]
        if (int(c["sequence"]) != seq or int(a["sequence"]) != seq or stamps[0] <= 0 or
            int(a["feedback_ns"]) != stamps[0] or int(c["actor_target_ns"]) != stamps[-2] or
            not stamps[-2] <= int(c["receive_ns"]) <= stamps[-1] or
            not stamps[-2] <= int(a["sent_ns"]) or
            stamps != sorted(stamps) or stamps[-1] - stamps[0] > 20_000_000):
            raise RuntimeError(f"invalid joined CAN/actor timing at sequence {seq}")
        durations = [b-a for a,b in zip(stamps, stamps[1:])] + [stamps[-1]-stamps[0]]
        for name, duration in zip(maxima, durations):
            maxima[name] = max(maxima[name], duration)
    return {name: value/1e6 for name, value in maxima.items()}


def apply_training_gains(joint_table, target, contract):
    with joint_table.open() as stream:
        reader = csv.DictReader(stream, delimiter="\t")
        columns, rows = reader.fieldnames, list(reader)
    if [row["joint_name"] for row in rows] != contract.data["joint_names"]:
        raise ValueError("training gain overlay joint order mismatch")
    target = bytearray(target)
    gains = np.frombuffer(target, dtype="<f8", offset=16).reshape(4, 31)
    for j, row in enumerate(rows):
        kp, kd = contract.data["stiffness"][j], contract.data["damping"][j]
        if not math.isfinite(kp + kd) or not 0 < kp <= 500 or not 0 < kd <= 5:
            raise ValueError("training gains outside existing MIT range")
        row["kp_max"], row["kd_max"] = repr(kp), repr(kd)
        gains[2, j], gains[3, j] = kp, kd
    with joint_table.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, delimiter="\t", lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    return bytes(target)


def operator_message(path, message):
    line = f"{datetime.now().astimezone().isoformat(timespec='milliseconds')} {message}"
    with path.open("a", encoding="utf-8") as stream:
        print(line, file=stream, flush=True)
    print(line, flush=True)


def relay_operator_status(fd, path, events, errors, walk_speed):
    try:
        with os.fdopen(fd, encoding="ascii") as source:
            for line in source:
                event = line.strip()
                if event not in INTERNAL_OPERATOR_EVENTS:
                    operator_message(path, OPERATOR_MESSAGES[event].format(speed=walk_speed))
                events.append(event)
    except (OSError, ValueError, KeyError) as exc:
        errors.append(str(exc))


def verify_walk_commands(commands, speed):
    tick = np.maximum(0, np.arange(len(commands)) - 1)
    rise = np.clip((tick - 1500) / 150, 0., 1.)
    fall = np.clip((tick - 2150 + 1) / 150, 0., 1.)
    vx = speed * (rise * rise * (3. - 2. * rise)) * (1. - fall * fall * (3. - 2. * fall))
    if (np.any(commands[:, 1:] != 0) or not np.allclose(commands[:, 0], vx, rtol=0, atol=1e-12) or
        np.count_nonzero(commands[:, 0] == speed) != 500):
        raise RuntimeError("recorded actor commands differ from the bounded walking profile")


def verify_direct_start(records, feedback, trace, contract):
    """One non-policy handshake, then fresh ten-frame history and unblended actions."""
    if len(records) < 2 or len(trace) != len(records) or len(feedback) != len(records):
        raise ValueError("direct-start trace incomplete")
    if (np.any(records['raw'][0]) or np.any(records['action'][0]) or
        float(trace[0]['gain_alpha']) != 0 or
        any(float(row['gain_alpha']) != 1 for row in trace[1:])):
        raise ValueError("direct start retained a gain ramp or nonzero handshake action")
    parts = [feedback[1, :31].astype(np.float32)-np.asarray(contract['default_joint_pos'], dtype=np.float32),
             feedback[1, 31:62].astype(np.float32), np.zeros(31, dtype=np.float32),
             feedback[1, 62:65].astype(np.float32), feedback[1, 65:68].astype(np.float32)]
    expected = np.concatenate([np.tile(part, 10) for part in parts])
    if not np.array_equal(records['obs'][1, :990], expected):
        raise ValueError("direct start inherited pre-control history")
    goal = np.asarray(contract['action_offset']) + np.asarray(contract['action_scale']) * records['raw'][1:].astype(np.float64)
    if not np.allclose(records['target'][1:], goal, rtol=0, atol=1e-12):
        raise ValueError("direct start retained policy target interpolation")


def verify_standing_evidence(directory, fingerprints, training_full_body, body_checkpoint=False):
    standing = json.loads((directory/"commissioning.json").read_text())
    names = ("policy.onnx", "sprite_icm45686", "motors.tsv", "kinematics.tsv",
             "joints.tsv", "feedback.bin", "target.bin")
    if (standing["mode"] != "powered" or standing.get("walk_speed_m_s", 0.) != 0. or
        standing.get("training_full_body", False) != training_full_body or
        standing.get("xbox_leg_waist_gain_scale") != 1. or standing["final_disabled_count"] != 31 or
        any(standing["fingerprints"][name] != fingerprints[name] for name in names)):
        raise ValueError("standing evidence must match the walking model, full gains and transmission")
    if not body_checkpoint:
        if not standing["passed"] or not standing.get("sling_checkpoint", {}).get("sling_window_opened"):
            raise ValueError("a passed zero-speed standing run is required")
        return "passed_standing"
    # Explicit, narrow admission of the full-body zero-speed run that stopped
    # only at the old all-joint settling check. Walking already uses the body
    # criterion and repeats it live at 20 s AND immediately before the ramp.
    report = json.loads((directory/"can.json").read_text())
    checkpoint = standing.get("sling_checkpoint", {})
    expected_error = "zero-command standing not settled at sling checkpoint"
    if (not training_full_body or standing["passed"] or
        standing.get("failure") != f"CAN 2: ['{expected_error}']; actor 1: state receive timeout/error" or
        report.get("errors") != [expected_error] or
        checkpoint.get("sling_checkpoint_sequence") != 1000 or
        checkpoint.get("sling_checkpoint_body_frames") != 50 or
        report.get("sling_checkpoint_body_frames") != 50 or
        report.get("sling_checkpoint_sequence") != 1000 or
        report.get("deadline_misses") != 0 or report.get("missing_feedback_motor") != "" or
        not 0 < report.get("oldest_feedback_to_target_accept_max_ms", 0) <= 20 or
        report.get("motor_enable_attempts") != 31 or len(report.get("motors", {})) != 31 or
        not all(m["final_disabled"] for m in report["motors"].values())):
        raise ValueError("body checkpoint evidence includes an unqualified stop or incomplete disable")
    return "recorded_full_body_stable_checkpoint"


def prepare(args, out, contract):
    tools = args.runtime_root / "tools"
    def export(script, *options):
        subprocess.run([sys.executable, str(tools / script), *map(str, options)], check=True)

    motors = out / "motors.tsv"
    kinematics = out / "kinematics.tsv"
    joint_limits = out / "joints.tsv"
    export("export_native_motor_config.py", "--hardware", args.hardware, "--output", motors)
    export("export_native_kinematics_config.py", "--hardware", args.hardware, "--contract",
           contract.path, "--output", kinematics)
    export("export_native_joint_safety_config.py", "--contract", contract.path,
           "--joint-limit-candidates", args.joint_limits, "--gain-scale", GAIN, "--output", joint_limits)
    with motors.open() as stream:
        motor_names = [row["motor_name"] for row in csv.DictReader(stream, delimiter="\t")]
    with kinematics.open() as stream:
        rows = list(csv.DictReader(stream, delimiter="\t"))
    with joint_limits.open() as stream:
        joints = list(csv.DictReader(stream, delimiter="\t"))
    joint_names = contract.data["joint_names"]
    if (len(motor_names) != 31 or len(rows) != 62 or len(joints) != 31 or
        [row["name"] for row in rows[:31]] != motor_names or
        [row["name"] for row in rows[31:]] != joint_names or
        [row["joint_name"] for row in joints] != joint_names):
        raise ValueError("commissioning motor or joint ABI order mismatch")
    offsets = np.array([float(row["offset_or_effort"]) for row in rows[:31]], dtype="<f8")
    inverse = np.array([[float(row[f"c{i}"]) for i in range(31)] for row in rows[31:]], dtype="<f8")
    (out/"feedback.bin").write_bytes(struct.pack("<8sQ", b"SPMSEN01", ordered_name_hash(motor_names))
                                    + offsets.tobytes() + inverse.tobytes())
    target_fields = ("position_min_rad", "position_max_rad", "kp_max", "kd_max")
    limits = np.array([[float(row[key]) for row in joints] for key in target_fields], dtype="<f8")
    joint_hash = ordered_name_hash(joint_names)
    (out/"target.bin").write_bytes(struct.pack("<8sQ", b"SPMTGT01", joint_hash) + limits.tobytes())
    inputs = np.zeros((int(args.duration * 50) + 1, 71))
    inputs[:, 67] = -1
    write_fixture(out/"input.bin", contract, inputs, 75)
    return joint_hash


def run(args, out, mode):
    frames = int(args.duration * 50) + 1
    build = args.build.resolve()
    actor_binary = build / "sprite_supermimic_protected_ipc"
    can_binary = build / "sprite_can_shadow"
    subprocess.run([can_binary, "--check-commissioning-scheduler", str(CAN_CPU)], check=True, timeout=5)
    can_schedule = dict(cpu=CAN_CPU, policy="SCHED_FIFO", priority=CAN_REALTIME_PRIORITY)
    verify_bundle(args.bundle)
    contract = PolicyContract.load(args.bundle/"contract.json")
    model = checked_model(contract)
    if file_sha256(args.hardware) != HARDWARE_SHA or file_sha256(args.joint_limits) != LIMITS_SHA:
        raise ValueError("commissioning hardware or joint limits identity changed")
    joint_hash = prepare(args, out, contract)
    roles = None
    if args.tracked_startup:
        for name, digest in MATCHED_PROFILE_SHA.items():
            candidate = args.tracked_startup / name
            if file_sha256(candidate) != digest:
                raise ValueError("reviewed Xbox gain and torque profile changed: " + name)
            (out/name).write_bytes(candidate.read_bytes())
        set_peak_feedback_caps(out/"motors.tsv")
        for name, digest in MATCHED_MAPPING_SHA.items():
            if file_sha256(out/name) != digest:
                raise ValueError("reviewed transmission or calibration changed: " + name)
        target_path = out/"target.bin"
        reviewed = target_path.read_bytes()
        if reviewed[:16] != struct.pack("<8sQ", b"SPMTGT01", joint_hash):
            raise ValueError("reviewed target joint order changed")
        roles = startup_joint_roles(contract.data["joint_names"])
        if args.xbox_leg_waist_gains:
            reviewed = apply_xbox_leg_waist_gains(args.xbox_leg_waist_gains, out/"joints.tsv",
                                                  reviewed, contract.data["joint_names"], roles)
        if args.training_full_body:
            reviewed = apply_training_gains(out/"joints.tsv", reviewed, contract)
        target_path.write_bytes(b"SPMTGT02" + reviewed[8:] + roles)
    fingerprints = {p.name: file_sha256(p) for p in (
        Path(__file__), model, args.icm, can_binary, actor_binary, out/"motors.tsv",
        out/"kinematics.tsv", out/"joints.tsv", out/"feedback.bin", out/"target.bin", out/"input.bin")}
    if args.coherent_action_send:
        tx_audit_path = Path(__file__).with_name('action_tx_audit.py')
        fingerprints[tx_audit_path.name] = file_sha256(tx_audit_path)
        fingerprints['loaded_kcan_srcversion'] = Path('/sys/module/kcan/srcversion').read_text().strip()
    standing_admission = None
    if mode == "powered":
        if args.walk_speed:
            standing_admission = verify_standing_evidence(
                args.standing_body_checkpoint or args.passed_standing, fingerprints,
                args.training_full_body, bool(args.standing_body_checkpoint))
        for directory, expected_mode in ((args.passed_disabled, "disabled"), (args.passed_deadline_fault, "deadline_fault")):
            prior = json.loads((directory/"commissioning.json").read_text())
            if (not prior["passed"] or prior["mode"] != expected_mode or prior["duration_s"] != args.duration or
                prior["fingerprints"] != fingerprints or prior["motor_enable_attempts"] != 0 or
                prior.get("tracked_startup", False) != bool(args.tracked_startup) or
                prior.get("sling_window", False) != args.sling_window or
                prior.get("walk_speed_m_s", 0.) != args.walk_speed or
                prior.get("training_full_body", False) != args.training_full_body or
                prior.get("direct_policy_start", False) != args.direct_policy_start or
                prior.get("coherent_action_send", False) != args.coherent_action_send or
                prior.get("can_schedule") != can_schedule or
                prior["final_disabled_count"] != 31):
                raise ValueError(f"same-build {expected_mode} gate required before enable")

    interfaces = json.loads(args.hardware.read_text())["can_adapter"]["interfaces"]
    def snapshot(name):
        raw = subprocess.check_output(["ip", "-j", "-d", "-s", "link", "show", "type", "can"])
        (out/name).write_bytes(raw)
        data = json.loads(raw)
        for interface in interfaces:
            result = audit_socketcan_active_fd_snapshot(data, interface)
            if not result.passed:
                raise RuntimeError(str(result.errors))
        return {item["ifname"]: tuple(item["stats64"][direction][k] for direction in ("rx","tx")
                                       for k in ("errors","dropped"))
                for item in data if item["ifname"] in interfaces}

    before = snapshot("can-before.json")
    processes = []
    paused = False
    can = actor = None
    failure = None
    report = {}
    startup = None
    timing_max_ms = None
    operator_events, relay_errors = [], []
    pipe_fds = set()
    relay = None
    imu_socket, state_socket = out/"imu.sock", out/"state.sock"
    def launch(name, command, **kwargs):
        with (out/(name+".log")).open("wb") as log:
            process = subprocess.Popen([str(x) for x in command], stdout=log, stderr=subprocess.STDOUT, **kwargs)
        processes.append(process)
        return process

    def close_pipe(fd):
        os.close(fd)
        pipe_fds.remove(fd)

    try:
        can_status = actor_status = []
        can_fds = actor_fds = ()
        if args.sling_window:
            mode_label = "Powered test" if mode == "powered" else "Disabled validation"
            operator_message(args.operator_status, f"{mode_label}. Calibrating IMU; keep the robot still.")
            ready_read, ready_write = os.pipe()
            pipe_fds.update((ready_read, ready_write))
            status_read, status_write = os.pipe()
            pipe_fds.update((status_read, status_write))
            for fd in (ready_read, ready_write, status_write):
                os.set_blocking(fd, False)
            relay = threading.Thread(target=relay_operator_status, args=(
                status_read, args.operator_status, operator_events, relay_errors, args.walk_speed), daemon=True)
            relay.start()
            pipe_fds.remove(status_read)
            can_status = ["--sling-ready-fd", ready_read, "--operator-status-fd", status_write]
            actor_status = ["--sling-ready-fd", ready_write]
            can_fds, actor_fds = (ready_read, status_write), (ready_write,)
        imu = launch("imu", ["taskset", "-c", "3", args.icm, "capture", "/dev/spidev0.0", "/dev/gpiochip0",
            "123", "800", "85" if args.walk_speed else "60" if args.sling_window else "35",
            out/"imu.csv", "X_DOWN_Z_BACK", "CONFIGURE_ICM45686_RAW_NO_CAN"],
            env=dict(os.environ, ICM_IPC_SOCKET=str(imu_socket)))
        wait = time.monotonic()+25
        while "BIAS_READY" not in (out/"imu.log").read_text(errors="replace"):
            if imu.poll() is not None or time.monotonic()>wait:
                raise RuntimeError("ICM did not pass its stationary calibration")
            time.sleep(.05)
        print("ICM_READY", flush=True)
        options = ["--policy-actuation"] if mode == "powered" else []
        tracking = ["--tracked-startup"] if args.tracked_startup else []
        walk_code = {.20: "020", .25: "025", .30: "030"}.get(args.walk_speed)
        walking = [f"--walk-{walk_code}"] if args.walk_speed else []
        xbox_full = ["--xbox-full-leg-waist-gains"] if args.xbox_leg_waist_gains else []
        full_body = ["--training-full-body"] if args.training_full_body else []
        direct = ["--direct-policy-start"] if args.direct_policy_start else []
        coherent = ["--coherent-action-send"] if args.coherent_action_send else []
        ack = "ZERO_GAIN_SUPERMIMIC_WRITEBACK"
        if mode == "powered":
            ack = "ENABLE_SUSPENDED_SUPERMIMIC_LOW_GAIN"
            if args.duration != 2:
                ack += f"_{args.duration}S"
        if args.sling_window:
            ack = ("ENABLE_SUPERMIMIC_ZERO_SPD_SLING_WINDOW" if mode == "powered"
                   else "ZERO_GAIN_SUPERMIMIC_SLING_WINDOW")
        if args.walk_speed:
            action = "ENABLE" if mode == "powered" else "PREVIEW"
            ack = f"{action}_SUPERMIMIC_WALK_{walk_code}_10S"
        can = launch("can", [can_binary, "--supermimic-commissioning", *options, *tracking, *direct, *coherent, *walking, *can_status,
            "--acknowledge-hardware-tx", ack, "--all-motors-disabled-confirmed", "--config", out/"motors.tsv",
            "--kinematics-config", out/"kinematics.tsv", "--joint-safety-config", out/"joints.tsv",
            "--duration", args.duration, "--cpu", CAN_CPU, "--realtime-priority", CAN_REALTIME_PRIORITY,
            "--output", out/"can.json",
            "--ipc-socket", state_socket, "--policy-joint-hash", joint_hash], pass_fds=can_fds)
        for fd in can_fds:
            close_pipe(fd)
        wait = time.monotonic()+3
        while not state_socket.exists():
            if can.poll() is not None or time.monotonic()>wait:
                raise RuntimeError("protected CAN process did not open IPC")
            time.sleep(.01)
        # Check the running process, not just the requested command-line options.
        # The actor has not connected: no policy target or enable can occur yet.
        if (os.sched_getaffinity(can.pid) != {CAN_CPU} or
            os.sched_getscheduler(can.pid) != os.SCHED_FIFO or
            os.sched_getparam(can.pid).sched_priority != CAN_REALTIME_PRIORITY or
            CAN_CPU in ACTOR_CPUS):
            raise RuntimeError("live CAN scheduling/affinity verification failed")
        (out/"scheduler-actual.json").write_text(json.dumps(dict(
            pid=can.pid, cpu=CAN_CPU, affinity=sorted(os.sched_getaffinity(can.pid)),
            policy="SCHED_FIFO", priority=os.sched_getparam(can.pid).sched_priority,
            actor_cpus=list(ACTOR_CPUS), verified_before_actor_connection=True),indent=2))
        actor = launch("actor", ["taskset", "-c", ",".join(map(str,ACTOR_CPUS)), actor_binary, model, out/"input.bin", out/"native.bin",
            "--threads", "2", "--warmup", "100", "--state-socket", state_socket, "--imu-socket", imu_socket,
            "--feedback-config", out/"feedback.bin", "--target-config", out/"target.bin",
            "--imu-kind", "icm", *tracking, *xbox_full, *full_body, *direct, *walking, *actor_status], pass_fds=actor_fds)
        for fd in actor_fds:
            close_pipe(fd)
        if mode == "deadline_fault":
            time.sleep(36.5 if args.walk_speed else 6.5 if args.duration >= 10 else 1.25)
            if actor.poll() is not None or can.poll() is not None:
                raise RuntimeError("process exited before deadline fault injection")
            actor.send_signal(signal.SIGSTOP)
            paused = True
        can_code = can.wait(timeout=max(15, args.duration+5))
        report = json.loads((out/"can.json").read_text())
        print(f"CAN_EXIT code={can_code} disabled={sum(m['final_disabled'] for m in report['motors'].values())}", flush=True)
        if report["can_cpu"] != CAN_CPU or report["can_realtime_priority"] != CAN_REALTIME_PRIORITY:
            raise RuntimeError("CAN scheduling profile differs from admission")
        if report.get("walk_speed_m_s", 0.) != args.walk_speed:
            raise RuntimeError("CAN walking profile differs from admission")
        if report.get("direct_policy_start", False) != args.direct_policy_start:
            raise RuntimeError("CAN startup mode differs from admission")
        if report.get("coherent_action_send", False) != args.coherent_action_send:
            raise RuntimeError("CAN coherent-action mode differs from admission")
        if paused:
            actor.send_signal(signal.SIGCONT)
            paused = False
        actor_code = actor.wait(timeout=5)
        if args.coherent_action_send:
            tx_audit = subprocess.run([sys.executable, str(Path(__file__).with_name('action_tx_audit.py')), str(out)],
                                      capture_output=True, text=True, timeout=30)
            (out/'action-tx-audit.log').write_text(tx_audit.stdout + tx_audit.stderr)
            if tx_audit.returncode:
                raise RuntimeError('per-action motor TX audit failed; see action-tx-audit.log')
        if mode == "deadline_fault":
            minimum_targets = 1652 if args.walk_speed else 251 if args.duration >= 10 else 2
            if (can_code != 2 or report["passed"] or report["policy_target_count"] < minimum_targets or not actor_code or
                report.get("deadline_stage") != "target_ipc_pending" or
                not any("oldest-feedback 20 ms deadline exceeded" in error for error in report["errors"])):
                raise RuntimeError("did not verify the expected real-feedback deadline rejection")
        else:
            if can_code or actor_code or not report["passed"]:
                raise RuntimeError(f"CAN {can_code}: {report['errors']}; actor {actor_code}: "
                                   + (out/"actor.log").read_text(errors="replace").strip().split("\n")[-1])
            timing = json.loads((out/"native.bin.timing.json").read_text())
            if (report["policy_state_count"] != frames or
                report["policy_target_count"] != frames or report["deadline_misses"] or
                not 0 < report["oldest_feedback_to_target_accept_max_ms"] <= 20 or
                timing["frames"] != frames or timing["deadline_misses"] or timing["maximum_target_kp"] <= 0):
                raise RuntimeError("strict writeback did not pass")
            timing_max_ms = timing_breakdown(out, frames)
            if args.tracked_startup and (not report["tracked_startup"] or not timing["tracked_startup"]):
                raise RuntimeError("tracked startup mode was not active on both sides")
            if bool(args.xbox_leg_waist_gains) != timing.get("xbox_full_leg_waist_gains", False):
                raise RuntimeError("actor Xbox gain mode differs from the reviewed profile")
            if args.training_full_body != timing.get("training_full_body", False):
                raise RuntimeError("actor full-body mode differs from admission")
            if timing.get("direct_policy_start", False) != args.direct_policy_start:
                raise RuntimeError("actor startup mode differs from admission")
            if timing.get("walk_speed_m_s", 0.) != args.walk_speed:
                raise RuntimeError("actor walking profile differs from admission")
            if args.tracked_startup:
                if not report.get("pd_position_saturation"):
                    raise RuntimeError("pure PD position saturation was not active")
                feedback = np.fromfile(out/"native.bin.inputs", dtype="<f8").reshape(frames, 71)
                if args.walk_speed:
                    verify_walk_commands(feedback[:, 68:], args.walk_speed)
                held = [] if args.training_full_body else [j for j, role in enumerate(roles) if role == 4]
                with (out/"native.bin.startup.csv").open() as stream:
                    trace = list(csv.DictReader(stream))
                if (len(trace) != frames or
                    any(abs(float(row[f"waypoint_{j}_rad"]) - feedback[0, j]) > 1e-10
                        for row in trace for j in held) or
                    float(trace[0]["gain_alpha"]) != 0 or float(trace[-1]["gain_alpha"]) != 1):
                    raise RuntimeError("head and wrists did not hold the measured pose at reviewed gains")
                if args.training_full_body:
                    if timing.get("action_history") != "policy_output":
                        raise RuntimeError("full-body action history must match training policy output")
                    dtype = np.dtype([("obs", "<f4", (993,)), ("raw", "<f4", (31,)),
                                      ("action", "<f8", (31,)), ("target", "<f8", (31,))])
                    records = np.fromfile(out/"native.bin", dtype=dtype)
                    limits = np.frombuffer((out/"target.bin").read_bytes(), dtype="<f8", count=124, offset=16).reshape(4,31)
                    if len(records) != frames:
                        raise RuntimeError("incomplete full-body actor trace")
                    if args.direct_policy_start:
                        verify_direct_start(records, feedback, trace, contract.data)
                    if (not np.array_equal(records["action"], records["raw"].astype(np.float64)) or
                        not np.array_equal(records["obs"][1:, 899:930], records["raw"][:-1])):
                        raise RuntimeError("previous-action observations differ from preceding policy outputs")
                    for i, row in enumerate(trace):
                        if float(row["gain_alpha"]) == 1:
                            waypoint = np.array([float(row[f"waypoint_{j}_rad"]) for j in range(31)])
                            if not np.allclose(waypoint, np.clip(records["target"][i], limits[0], limits[1]), rtol=0, atol=1e-10):
                                raise RuntimeError("full-body waypoint differs from projected policy target")
                final_second = feedback[-50:]
                gyro_peak = float(np.max(np.linalg.norm(final_second[:, 62:65], axis=1)))
                tilt_peak = float(np.max(np.degrees(np.arccos(np.clip(-final_second[:, 67], -1, 1)))))
                startup = dict(gain_ramp_completed=timing["gain_ramp_completed"],
                               final_goal_error_rad=timing["startup_final_goal_error_rad"],
                               joint_settled=timing["startup_settled"],
                               held_joint_count=len(held),
                               last_second_gyro_peak_rad_s=gyro_peak,
                               last_second_tilt_peak_deg=tilt_peak,
                               body_settled=gyro_peak <= .15 and tilt_peak <= 10.)
                if mode == "powered" and (not startup["body_settled"] or
                                          (not args.walk_speed and not startup["joint_settled"])):
                    raise RuntimeError("tracked startup did not meet the zero-command settling gate")
            if not args.tracked_startup and args.duration >= 6 and (not timing["gain_ramp_completed"] or timing["final_gain_ramp_alpha"] != 1):
                raise RuntimeError("full startup gain ramp was not verified")
        if snapshot("can-after.json") != before:
            raise RuntimeError("CAN error or drop counters changed")
    except Exception as exc:
        failure = str(exc)
    finally:
        if paused:
            actor.send_signal(signal.SIGCONT)
        for fd in pipe_fds:
            os.close(fd)
        # A controller SIGTERM takes its native disable path. Never SIGKILL it.
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    failure = failure or "commissioning process did not exit on SIGTERM"
        if relay:
            relay.join(timeout=2)

    if not report and can is not None and can.poll() is not None and (out/"can.json").is_file():
        report = json.loads((out/"can.json").read_text())
    disabled = sum(motor["final_disabled"] for motor in report.get("motors", {}).values())
    enables = report.get("motor_enable_attempts")
    if args.sling_window:
        expected = ["WAITING_FOR_ENABLE"]
        prefix = "" if mode == "powered" else "DRY_"
        if args.direct_policy_start:
            expected += (["POLICY_DIRECT", "ENABLED_31"] if mode == "powered"
                         else ["DRY_POLICY_DIRECT", "DRY_ENABLE_NOTICE"])
        else:
            expected += (["ENABLED_31", "POLICY_BLEND"] if mode == "powered"
                         else ["DRY_ENABLE_NOTICE", "DRY_POLICY_BLEND"])
        if args.walk_speed:
            expected += [prefix+"WALK_SLING_WINDOW_OPEN", *SLING_COUNTDOWN,
                         prefix+"WALK_RAMP", prefix+"WALK_FULL"]
            if mode != "deadline_fault":
                expected += [prefix+"WALK_STOP", prefix+"WALK_ZERO"]
        else:
            if mode != "deadline_fault":
                expected += ["SLING_WINDOW_OPEN" if mode == "powered" else "DRY_WINDOW_OPEN",
                             *SLING_COUNTDOWN, "WINDOW_END"]
        expected.append("DISABLED_31")
        if relay_errors or operator_events != expected:
            failure = failure or "operator status sequence incomplete or out of order"
        if not args.direct_policy_start and report.get("sling_notice_to_blend_ms", 0) < 2000:
            failure = failure or "operator notice was shorter than two seconds"
        if "DISABLED_31" not in operator_events and "DISABLE_UNCONFIRMED" not in operator_events:
            if can is None:
                operator_message(args.operator_status, "Startup preparation failed. No motor enable was sent.")
            elif disabled == 31:
                operator_message(args.operator_status, OPERATOR_MESSAGES["DISABLED_31"])
            else:
                operator_message(args.operator_status, OPERATOR_MESSAGES["DISABLE_UNCONFIRMED"])
    passed = not failure and disabled == 31 and enables == (31 if mode == "powered" else 0)
    leg_waist_gain_scale = .20 if args.tracked_startup else GAIN
    if args.xbox_leg_waist_gains or args.training_full_body:
        leg_waist_gain_scale = 1.
    evidence = dict(mode=mode, passed=bool(passed), failure=failure, duration_s=args.duration,
                    gain_scale=1. if args.training_full_body else GAIN, xbox_leg_waist_gain_scale=leg_waist_gain_scale,
                    training_full_body=args.training_full_body, standing_admission=standing_admission,
                    direct_policy_start=args.direct_policy_start,
                    coherent_action_send=args.coherent_action_send,
                    action_history="policy_output" if args.training_full_body else "admitted_waypoint",
                    tracked_startup=bool(args.tracked_startup), startup=startup,
                    sling_window=args.sling_window, walk_020=args.walk_speed == .20,
                    walk_speed_m_s=args.walk_speed, operator_events=operator_events,
                    sling_checkpoint={name: value for name, value in report.items() if name.startswith("sling_")},
                    walk_checkpoint={name: value for name, value in report.items() if name.startswith("walk_")},
                    can_schedule=can_schedule,
                    timing_max_ms=timing_max_ms, deadline_stage=report.get("deadline_stage"),
                    velocity_command=[args.walk_speed,0,0], motor_enable_attempts=enables,
                    final_disabled_count=disabled, fingerprints=fingerprints,
                    hardware_qualified=False, powered_walking_ready=False)
    (out/"commissioning.json").write_text(json.dumps(evidence, indent=2)+"\n")
    print(json.dumps(evidence), flush=True)
    return 0 if passed else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--duration", type=int, choices=(2, 6, 10), default=2)
    for name in ("bundle", "hardware", "joint-limits", "build", "icm", "lock", "output"):
        parser.add_argument("--"+name, type=Path, required=True)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--deadline-fault", action="store_true")
    modes.add_argument("--enable-suspended", action="store_true")
    parser.add_argument("--passed-disabled", type=Path)
    parser.add_argument("--passed-deadline-fault", type=Path)
    parser.add_argument("--tracked-startup", type=Path, metavar="REVIEWED_PROFILE_DIR")
    parser.add_argument("--xbox-leg-waist-gains", type=Path, metavar="SEALED_XBOX_JOINT_TSV",
                        help="use the sealed Xbox Kp/Kd on legs and both waist joints")
    parser.add_argument("--sling-window", action="store_true")
    parser.add_argument("--training-full-body", action="store_true")
    parser.add_argument("--direct-policy-start", action="store_true")
    parser.add_argument("--coherent-action-send", action="store_true")
    walking = parser.add_mutually_exclusive_group()
    walking.add_argument("--walk-020", dest="walk_speed", action="store_const", const=.20, default=0.)
    walking.add_argument("--walk-025", dest="walk_speed", action="store_const", const=.25)
    walking.add_argument("--walk-030", dest="walk_speed", action="store_const", const=.30)
    standing = parser.add_mutually_exclusive_group()
    standing.add_argument("--passed-standing", type=Path)
    standing.add_argument("--standing-body-checkpoint", type=Path,
                          help="full-body zero-speed evidence stopped only by all-joint settling")
    parser.add_argument("--operator-status", type=Path)
    args = parser.parse_args()
    if args.coherent_action_send and not args.direct_policy_start:
        parser.error("coherent action sending requires direct policy start")
    if args.direct_policy_start and not args.training_full_body:
        parser.error("direct policy start requires full training gains")
    if args.training_full_body and (not args.tracked_startup or args.xbox_leg_waist_gains):
        parser.error("training full-body requires tracked startup without Xbox overlay")
    if args.xbox_leg_waist_gains and not args.tracked_startup:
        parser.error("sealed Xbox leg and waist gains require tracked startup")
    if args.walk_speed and (not args.sling_window or not (args.xbox_leg_waist_gains or args.training_full_body)):
        parser.error("walking requires the sling window and full gains")
    if args.standing_body_checkpoint and (not args.walk_speed or not args.training_full_body):
        parser.error("body-checkpoint evidence requires full-body walking")
    if args.walk_speed and args.enable_suspended and not (args.passed_standing or args.standing_body_checkpoint):
        parser.error("powered walking requires standing evidence")
    if args.sling_window:
        if not args.tracked_startup or not args.operator_status:
            parser.error("sling window requires a reviewed tracked startup profile and --operator-status")
        args.duration = WALK_DURATION if args.walk_speed else SLING_DURATION
    elif args.tracked_startup and args.duration != 10:
        parser.error("tracked startup requires the 10-second commissioning tier")
    if args.enable_suspended and not (args.passed_disabled and args.passed_deadline_fault):
        parser.error("powered mode requires the same-build disabled and deadline fault results")
    mode = "powered" if args.enable_suspended else "deadline_fault" if args.deadline_fault else "disabled"
    with args.lock.open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        out = args.output.resolve()
        out.mkdir(parents=True, exist_ok=False)
        return run(args, out, mode)


if __name__ == "__main__":
    raise SystemExit(main())
