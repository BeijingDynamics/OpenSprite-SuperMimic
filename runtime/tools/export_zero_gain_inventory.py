"""Offline configuration export for the C++ zero-gain probe; never opens CAN/IMU."""
from pathlib import Path
import argparse
import hashlib
import json

from open_sprite_runtime.damiao import endpoints_from_hardware_config


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hardware', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    endpoints = endpoints_from_hardware_config(json.loads(args.hardware.read_text()))
    if args.output.exists():
        raise FileExistsError(args.output)
    for endpoint in endpoints:
        if endpoint.can_id not in range(1, 9) or endpoint.master_id != endpoint.can_id + 16:
            raise ValueError('this bounded Sprite probe requires qualified command/master IDs')
    content = ''.join(' '.join(map(str, (e.interface, e.motor_name, e.can_id, e.master_id))) + '\n'
                      for e in endpoints)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(content, encoding='utf-8', newline='\n')
    print(json.dumps({'motors':len(endpoints), 'hardware_sha256':hashlib.sha256(args.hardware.read_bytes()).hexdigest(),
                      'inventory_sha256':hashlib.sha256(content.encode()).hexdigest(), 'hardware_tx':0}))


if __name__ == '__main__':
    main()
