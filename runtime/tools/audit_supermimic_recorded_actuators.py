"""File-only counterfactual actuator audit of a real receive-only policy capture.

The disabled physical plant did NOT follow these targets. Torque predictions are
diagnostics, never measured actuator torque or closed-loop admission evidence.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.simulation_drives import SimulationDriveBank
from open_sprite_runtime.supermimic_replay import NATIVE_RECORD


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit(contract_path, hardware_path, capture, output):
    contract = PolicyContract.load(contract_path)
    contract.validate_for_replay('supermimic_v5_10f')
    hardware = json.loads(hardware_path.read_text())
    source = json.loads((capture/'summary.json').read_text())
    if not source['passed'] or source['policy_sha256'] != contract.data['policy_onnx_sha256']:
        raise ValueError('capture/model identity mismatch')
    if sha(hardware_path) != '9ddf06034f24bbca49cb06f80236caa4cea7aee0040516d11209f848da1f2c10':
        raise ValueError('frozen hardware changed')
    bank = SimulationDriveBank(hardware, contract.data, allow_nameplate_assumptions=True,
                               compensate_damping_cap=True)
    states = np.fromfile(capture/'native.bin.inputs', dtype='<f8')
    if len(states) % 71: raise ValueError('partial input record')
    states = states.reshape(-1, 71)
    records = np.fromfile(capture/'native.bin', dtype=NATIVE_RECORD)
    if len(records) != len(states) or len(states) != source['actor']['frames']:
        raise ValueError('input/output frame count mismatch')
    if not all(np.isfinite(v).all() for v in (states, records['target'], records['action'])):
        raise ValueError('nonfinite recording')
    if np.any(states[:, 68:] != 0): raise ValueError('expected the actual zero-command capture')
    q, v, target = states[:, :31], states[:, 31:62], records['target']
    qm, vm, tm = q@bank.matrix.T+bank.zero, v@bank.matrix.T, target@bank.matrix.T+bank.zero
    error = tm-qm
    ff = error@bank.cross_kp.T-vm@bank.cross_kd.T
    request = error*bank.motor_kp-vm*bank.motor_kd+ff
    # Ankle commands in the actual intended runtime are host joint-PD torque,
    # joint-effort limited first, then mapped with the inverse transpose.
    joint_pd = (target-q)*bank.kp-v*bank.kd
    ankle = np.clip(joint_pd, -bank.joint_limit, bank.joint_limit)@bank.inverse
    request[:, bank.ankle] = ankle[:, bank.ankle]
    ff[:, bank.ankle] = ankle[:, bank.ankle]
    cap = bank.limits(vm)
    hard = (qm < bank.hard_lower) | (qm > bank.hard_upper)
    soft = (tm < bank.soft_lower) | (tm > bank.soft_upper)
    torque = np.abs(request) > cap+1e-6
    ff_bad = np.abs(ff) > bank.protocol_torque+1e-6
    speed = np.abs(vm) > bank.maximum_speed
    rows = []
    for i, name in enumerate(bank.names):
        ratio = np.abs(request[:, i])/np.maximum(cap[:, i], 1e-12)
        worst = int(np.argmax(ratio))
        rows.append(dict(motor=name, measured_min_rad=float(qm[:, i].min()),
            measured_max_rad=float(qm[:, i].max()), hard_limits_rad=[bank.hard_lower[i], bank.hard_upper[i]],
            measured_hard_limit_frames=int(hard[:, i].sum()),
            target_soft_limit_frames=int(soft[:, i].sum()),
            predicted_torque_limit_frames=int(torque[:, i].sum()),
            predicted_feedforward_field_limit_frames=int(ff_bad[:, i].sum()),
            measured_speed_limit_frames=int(speed[:, i].sum()),
            worst_torque_frame=worst, predicted_torque_nm=float(request[worst, i]),
            predicted_cap_nm=float(cap[worst, i]), peak_abs_feedforward_nm=float(np.abs(ff[:, i]).max())))
    # Decode the original integer payload independently; don't trust only the
    # actor's already-converted floating-point state for an admission decision.
    inventory = [line.split() for line in (capture/'fixtures/inventory.tsv').read_text().splitlines()]
    seen = {row[1]: set() for row in inventory}; counts = dict.fromkeys(seen, 0)
    with (capture/'can.csv').open() as stream:
        for row in csv.DictReader(stream):
            item = inventory[int(row['motor_index'])]; payload = bytes.fromhex(row['payload_hex'])
            if payload[0] >> 4: raise ValueError('capture contained non-disabled motor status')
            code = int.from_bytes(payload[1:3], 'big')
            seen[item[1]].add(code); counts[item[1]] += 1
    raw_checks = []
    for i, item in enumerate(inventory):
        name=item[1]; j=bank.names.index(name)
        decoded = np.array(sorted(seen[name]))*(float(item[5])-float(item[4]))/65535+float(item[4])
        if not len(decoded): raise ValueError('raw CAN coverage missing: '+name)
        # 50Hz actor samples must lie in the range of independent 500Hz decoding.
        if qm[:, j].min() < decoded.min()-1e-12 or qm[:, j].max() > decoded.max()+1e-12:
            raise ValueError('raw CAN and consumed state disagree: '+name)
        if hard[:, j].any():
            raw_checks.append(dict(motor=name, raw_samples=counts[name], position_codes=sorted(seen[name]),
                independently_decoded_range_rad=[float(decoded.min()),float(decoded.max())],
                hard_limit_source=hardware['motor_map'][name]['limit_provenance']))
    initial = np.flatnonzero(hard[0])
    report = dict(schema='sprite_recorded_actuator_counterfactual_v1',
        policy_sha256=contract.data['policy_onnx_sha256'], hardware_sha256=sha(hardware_path),
        input_sha256=sha(capture/'native.bin.inputs'), output_sha256=sha(capture/'native.bin'),
        frames=len(states), raw_can_samples=sum(counts.values()), raw_can_decode_consistent=True,
        initial_hard_position_admission_passed=len(initial)==0,
        initial_rejections=[rows[i] for i in initial], independently_checked_hard_violations=raw_checks,
        predicted_torque_exceedances=int(torque.sum()), predicted_feedforward_exceedances=int(ff_bad.sum()),
        target_soft_exceedances=int(soft.sum()), measured_speed_exceedances=int(speed.sum()),
        motors=rows, hardware_tx=0, motor_enable_attempts=0, hardware_qualified=False,
        limitations=['Frozen disabled physical states, not a following plant or closed-loop simulation.',
            'Per-frame instantaneous hypothetical requests, not a bus-scheduled command or measured motor torque.',
            'Original limits unchanged. Nameplate torque/speed proxy, not measured actuator dynamics.',
            'Position-code mismatch with a software guard does not identify physical hard-stop or zero calibration error.'])
    output.mkdir(parents=True, exist_ok=False)
    (output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    np.savez_compressed(output/'counterfactual.npz', measured_motor_position=qm,
        measured_motor_velocity=vm, raw_motor_target=tm, requested_torque=request,
        torque_cap=cap, feedforward=ff, motor_names=np.array(bank.names))
    print(json.dumps({k:v for k,v in report.items() if k not in ('motors','initial_rejections','limitations')}))
    return report


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('contract','hardware','capture','output'):p.add_argument('--'+name, type=Path, required=True)
    a=p.parse_args();audit(a.contract,a.hardware,a.capture,a.output)
