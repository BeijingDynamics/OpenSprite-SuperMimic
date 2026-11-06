"""Install a locally authorized reference bank; never download or publish motion data."""
from pathlib import Path
import argparse
import hashlib
import shutil

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = '6f9cefe15646b532ccf57a40f20292c69c18f77ae0c33254abdedfcd1c66a8d0'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    args = parser.parse_args()
    with args.source.open('rb') as stream:
        if hashlib.file_digest(stream, 'sha256').hexdigest() != EXPECTED:
            raise ValueError('reference bank differs from the reviewed run002 bank')
    target = ROOT / 'data/motions/contact_v5_v11/v5_locomotion.npz'
    if args.source.resolve() != target.resolve():
        shutil.copyfile(args.source, target)
    print('Installed exact reviewed bank locally; data remains excluded from Git.')


if __name__ == '__main__':
    main()
