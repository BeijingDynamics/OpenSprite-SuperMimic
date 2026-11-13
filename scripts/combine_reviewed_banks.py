"""Combine reviewed banks without changing any trajectory samples or provenance."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

FIELDS = ('joint_pos', 'joint_vel', 'body_pos_w', 'body_quat_w', 'body_lin_vel_w', 'body_ang_vel_w')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def combine(selection, output):
    sources = json.loads(Path(selection).read_text())
    chunks = {key: [] for key in FIELDS}
    names, lengths, provenance, evidence = [], [], [], []
    first = None
    for source in sources:
        path = Path(source['bank'])
        qa_path = path.with_suffix('.qa.json')
        qa = json.loads(qa_path.read_text())
        if qa.get('accepted_for_training') is not True or qa.get('visual_review_passed') is not True:
            raise ValueError('Every source bank requires numeric and visual training qualification')
        if qa.get('split') == 'validation' or qa['motion_bank_sha256'] != sha(path):
            raise ValueError('Held-out or changed bank cannot enter training')
        with np.load(path, allow_pickle=False) as loaded:
            data = {key: loaded[key] for key in loaded.files}
        if float(data['fps']) != 50 or str(data['asset_revision']) != qa['asset_revision']:
            raise ValueError('Source asset or timing mismatch')
        if first is None:
            first = data
        for key in ('joint_names', 'body_names', 'asset_revision'):
            if not np.array_equal(first[key], data[key]):
                raise ValueError('Incompatible source ' + key)
        prefix = 'clip' if 'clip_starts' in data else 'motion'
        starts = data.get(prefix + '_starts', np.array([0]))
        sizes = data.get(prefix + '_lengths', np.array([len(data['joint_pos'])]))
        labels = data.get(prefix + '_names', np.array([source['single_clip_name']]) if 'single_clip_name' in source else None)
        rows = source['source_rows']
        if labels is None or len(labels) != len(starts) or len(rows) != len(starts):
            raise ValueError('Every clip requires a named provenance row')
        for start, size, label, row in zip(starts, sizes, labels, rows):
            start, size, label = int(start), int(size), str(label)
            if size < 2 or start < 0 or start + size > len(data['joint_pos']):
                raise ValueError('Invalid source clip boundary')
            if row['split'] != 'candidate_training' or row['name'] != label:
                raise ValueError('Source clip identity mismatch')
            names.append(label); lengths.append(size); provenance.append(row)
            for key in FIELDS:
                chunk = data[key][start:start + size]
                if len(chunk) != size or not np.isfinite(chunk).all():
                    raise ValueError('Invalid trajectory field ' + key)
                chunks[key].append(chunk)
        evidence.append(dict(bank=str(path),bank_sha256=sha(path),qa_sha256=sha(qa_path),qualification=qa))
    if not names or len(names) != len(set(names)):
        raise ValueError('Require distinct source clips')
    from pack_reviewed_neutral import validate_source_intervals
    validate_source_intervals(provenance, [], 'candidate_training')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=False)
    payload = {key: np.concatenate(value, axis=0) for key, value in chunks.items()}
    payload.update(fps=np.array(50.),joint_names=first['joint_names'],body_names=first['body_names'],
        asset_revision=first['asset_revision'],retarget_method=np.array('composition_of_independently_reviewed_banks_v1'),
        motion_names=np.array(names),motion_starts=np.r_[0,np.cumsum(lengths)[:-1]],motion_lengths=np.array(lengths))
    np.savez_compressed(output, **payload)
    with np.load(output, allow_pickle=False) as check:
        for key in FIELDS:
            offset = 0
            for chunk in chunks[key]:
                if not np.array_equal(check[key][offset:offset + len(chunk)], chunk):
                    raise ValueError('Composition changed source samples')
                offset += len(chunk)
    manifest = output.parent/'source-manifest.json'
    manifest.write_text(json.dumps(provenance,indent=2)+'\n')
    report = dict(accepted_for_training=True,accepted_for_evaluation=True,visual_review_passed=False,
        split='candidate_training',asset_revision=str(first['asset_revision']),motion_bank_sha256=sha(output),
        source_manifest_sha256=sha(manifest),source_evidence=evidence,source_samples_exactly_preserved=True,
        clips=names,frames=lengths,total_duration_s=sum(n-1 for n in lengths)/50,
        qualification='Source numeric/visual reviews inherited without interpolation; composition visual review still required')
    output.with_suffix('.qa.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'source_evidence'},indent=2))


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('--selection',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();combine(a.selection,a.output)
