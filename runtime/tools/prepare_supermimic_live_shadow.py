"""Offline prepare exact-model, zero-command fixtures and calibrated feedback mapping."""
from pathlib import Path
import argparse
import hashlib
import json
import struct

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.damiao import endpoints_from_hardware_config
from open_sprite_runtime.motor_mapping import motor_map_from_hardware_config
from open_sprite_runtime.native_ipc import ordered_name_hash
from open_sprite_runtime.supermimic_replay import write_fixture
from open_sprite_runtime.supermimic_run002 import verify_bundle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--hardware', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=1500)
    args = parser.parse_args()
    if not 50 <= args.frames <= 3000:
        parser.error('live shadow is bounded to 1..60 seconds at 50Hz')
    verify_bundle(args.bundle)
    contract = PolicyContract.load(args.bundle/'contract.json')
    hardware = json.loads(args.hardware.read_text())
    if hashlib.sha256(args.hardware.read_bytes()).hexdigest() != '9ddf06034f24bbca49cb06f80236caa4cea7aee0040516d11209f848da1f2c10':
        raise ValueError('frozen run002 hardware/feedback identity changed')
    endpoints = endpoints_from_hardware_config(hardware)
    names = tuple(e.motor_name for e in endpoints)
    mapping = motor_map_from_hardware_config(hardware, contract.data['joint_names'], require_armable=False)
    zero = np.zeros(31)
    offsets = mapping.joint_to_motor_positions(zero)
    offset = np.array([offsets[n] for n in names])
    matrix = np.empty((31,31))
    for j in range(31):
        basis = zero.copy();basis[j] = 1
        converted = mapping.joint_to_motor_positions(basis)
        matrix[:,j] = np.array([converted[n] for n in names]) - offset
    inverse = np.linalg.inv(matrix)
    np.testing.assert_allclose(inverse@matrix, np.eye(31), atol=1e-12)
    args.output.mkdir(parents=True, exist_ok=False)
    inventory = ''.join(' '.join(map(str,(e.interface,e.motor_name,e.can_id,e.master_id,
                          *e.ranges.position_rad,*e.ranges.velocity_rad_s)))+'\n' for e in endpoints)
    (args.output/'inventory.tsv').write_text(inventory)
    (args.output/'feedback.bin').write_bytes(b'SPMSEN01'+struct.pack('<Q',ordered_name_hash(names))+
        offset.astype('<f8').tobytes()+inverse.astype('<f8').tobytes())
    inputs = np.zeros((args.frames,71));inputs[:,67] = -1
    # Live native IPC replaces all 68 sensor values. Only the zero command remains.
    write_fixture(args.output/'input.bin', contract, inputs, 75)
    metadata = {'frames':args.frames,'command':[0,0,0],'handoff_seconds':1.5,
                'policy_sha256':contract.data['policy_onnx_sha256'],'motor_order':names,
                'sensor_values_replaced_from_live_ipc':True,'imu_kind':'icm','hardware_tx':0,
                'motor_enable_supported':False,'hardware_qualified':False}
    (args.output/'preparation.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata))


if __name__ == '__main__':
    main()
