"""Authorize research evaluation on a reviewed, source-disjoint motion bank."""
import hashlib
import json
from pathlib import Path
from pack_reviewed_neutral import validate_source_intervals


def verify_heldout_reference(training_bank, heldout_bank, training_sha256, asset_revision):
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    banks = [Path(training_bank), Path(heldout_bank)]
    if sha(banks[0]) != training_sha256:
        raise ValueError('Training bank does not match the checkpoint')
    if sha(banks[1]) == training_sha256:
        raise ValueError('Held-out evaluation requires a distinct reference bank')
    rows = []
    for bank, split in zip(banks, ['candidate_training', 'validation']):
        qa = json.loads(bank.with_suffix('.qa.json').read_text())
        manifest = bank.parent / 'source-manifest.json'
        qualified=qa.get('accepted_for_evaluation',qa.get('accepted_for_training'))
        if qualified is not True or qa.get('visual_review_passed') is not True:
            raise ValueError('Reference bank lacks numeric or rendered qualification')
        if qa['motion_bank_sha256'] != sha(bank) or qa['asset_revision'] != asset_revision:
            raise ValueError('Reference hash or robot asset differs from qualification')
        if qa.get('split') != split or qa['source_manifest_sha256'] != sha(manifest):
            raise ValueError('Source manifest or split differs from qualification')
        source = json.loads(manifest.read_text())
        if not source or any(row.get('split') != split for row in source):
            raise ValueError('Source rows do not belong to the requested split')
        rows.append(source)
    validate_source_intervals(rows[0], rows[1], 'candidate_training')
    validate_source_intervals(rows[1], rows[0], 'validation')
    return dict(mode='reviewed_source_disjoint_motion', training_sha256=training_sha256,
                evaluation_sha256=sha(banks[1]), source_margin_seconds=1,
                training_clips=len(rows[0]), evaluation_clips=len(rows[1]),
                limitation='Held-out motion tracking only; no training-data replacement or hardware admission')
