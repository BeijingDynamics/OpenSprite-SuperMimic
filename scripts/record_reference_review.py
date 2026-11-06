"""Bind the already performed visual review to exact bank/preview hashes."""
import argparse,hashlib,json
from pathlib import Path

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--quality',type=Path,required=True);p.add_argument('--review',type=Path,required=True);a=p.parse_args()
    quality=json.loads(a.quality.read_text());review=json.loads(a.review.read_text(encoding='utf-8-sig'))
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    bank=a.quality.with_name(a.quality.name.replace('.qa.json','.npz'))
    assert quality['accepted_for_training'] is True and review['passed'] is True
    assert quality['motion_bank_sha256']==review['motion_bank_sha256']==sha(bank)
    assert {r['name'] for r in quality['references']}=={r['motion'] for r in review['reviewed_candidates']}
    for row in review['reviewed_candidates']:
        assert row['passed'] is True and sha(bank.parent/'previews'/row['preview_file'])==row['sha256']
    quality.update(visual_review_passed=True,visual_review_sha256=sha(a.review),visual_reviewed_at=review['reviewed_at'])
    a.quality.write_text(json.dumps(quality,indent=2));print('REFERENCE_NUMERIC_AND_VISUAL_GATES_PASSED',sha(bank),flush=True)
