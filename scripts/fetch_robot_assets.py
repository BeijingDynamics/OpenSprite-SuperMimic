"""Fetch only missing, hash-pinned public Sprite meshes; no data or environment caches."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
REVISION = 'sprite0825_v5_4340_shoulders_waist20261105'
COMMIT = 'f42a266c860db70d73ce92f34e26d253a2659636'
BASE = ROOT / 'upstream_asset/models' / REVISION


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--local-cache', type=Path, help='Optional original model directory, verified identically')
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    manifest = json.loads((BASE / 'manifest.json').read_text())
    fetched = 0
    for relative, expected in manifest['files_sha256'].items():
        target = BASE / relative
        if not target.resolve().is_relative_to(BASE.resolve()):
            raise ValueError('manifest path escapes model directory')
        if target.is_file():
            if sha(target) != expected:
                raise ValueError('existing asset hash mismatch: ' + relative)
            continue
        if args.verify_only:
            raise FileNotFoundError(target)
        if not relative.startswith('meshes/'):
            raise FileNotFoundError('pinned source metadata missing: ' + relative)
        target.parent.mkdir(parents=True, exist_ok=True)
        temporary = target.with_suffix(target.suffix + '.download')
        if args.local_cache is not None:
            shutil.copyfile(args.local_cache / relative, temporary)
        else:
            url = f'https://raw.githubusercontent.com/BeijingDynamics/open_sprite/{COMMIT}/models/{REVISION}/{relative}'
            with urllib.request.urlopen(url, timeout=90) as response, temporary.open('wb') as stream:
                shutil.copyfileobj(response, stream)
        if sha(temporary) != expected:
            temporary.unlink()
            raise ValueError('downloaded asset hash mismatch: ' + relative)
        temporary.replace(target)
        fetched += 1
    print(json.dumps({'verified_files': len(manifest['files_sha256']), 'fetched': fetched,
                      'asset_revision': REVISION, 'commit': COMMIT}))


if __name__ == '__main__':
    main()
