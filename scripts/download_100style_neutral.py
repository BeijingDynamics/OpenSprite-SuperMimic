"""Selectively fetch author-linked Geno Neutral BVHs with archive CRC checks."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import download_lafan_walks as ranges

SOURCE = 'https://theorangeduck.com/media/uploads/Geno/100style-retarget/bvh.zip'
MOVEMENTS = ('FW', 'BW', 'SW', 'ID', 'TR1', 'FR', 'BR', 'SR')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--movements', nargs='+', choices=MOVEMENTS, default=MOVEMENTS)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    ranges.URL = SOURCE
    manifest = dict(
        original_dataset='https://www.ianxmason.com/100style/',
        conversion='https://github.com/orangeduck/100style-retarget',
        source_archive=SOURCE, license='CC-BY-4.0',
        attribution='The 100STYLE Dataset - Ian Mason; Geno conversion by Daniel Holden',
        mesh_license_note='Geno character mesh is separately restricted to non-commercial research.',
        accepted_for_training=False, files=[])
    with zipfile.ZipFile(ranges.RemoteZip()) as archive:
        for kind in args.movements:
            name = f'Neutral_{kind}.bvh'
            entry = archive.getinfo('bvh/' + name)
            target = args.output / name
            if not target.exists():
                content = archive.read(entry)
                if not content.lstrip().startswith(b'HIERARCHY'):
                    raise ValueError(f'Not a BVH: {name}')
                temporary = target.with_suffix('.part')
                temporary.write_bytes(content)
                temporary.replace(target)
            content = target.read_bytes()
            import zlib
            if len(content) != entry.file_size or zlib.crc32(content) != entry.CRC:
                raise ValueError(f'Archive CRC or size mismatch: {name}')
            row = dict(file=name, archive_entry=entry.filename, bytes=len(content),
                       sha256=hashlib.sha256(content).hexdigest(), archive_crc32=entry.CRC)
            manifest['files'].append(row)
            (args.output / 'source-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
            print(json.dumps(row), flush=True)


if __name__ == '__main__':
    main()
