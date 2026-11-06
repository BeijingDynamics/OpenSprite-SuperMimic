"""Explicit local-only acquisition of the pinned Geno retarget inputs; see external terms."""
from pathlib import Path
import argparse
import hashlib
import json
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-use', action='store_true', required=True,
                        help='Download external assets for local processing, not redistribution')
    parser.parse_args()
    manifest = json.loads((ROOT / 'configs/retarget_inputs.json').read_text())
    directory = ROOT / 'assets/geno'
    directory.mkdir(parents=True, exist_ok=True)
    for name, record in manifest.items():
        path = directory / name
        if path.exists():
            payload = path.read_bytes()
        else:
            with urllib.request.urlopen(record['url'], timeout=90) as response:
                payload = response.read()
        if hashlib.sha256(payload).hexdigest() != record['sha256']:
            raise ValueError('external asset differs from pinned input: ' + name)
        if not path.exists():
            path.write_bytes(payload)
    print('Verified Geno inputs locally. External license remains in assets/geno/license.txt.')


if __name__ == '__main__':
    main()
