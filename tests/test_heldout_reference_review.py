"""Held-out labels cannot bypass checkpoint identity or source-time separation."""
from pathlib import Path
import hashlib,json,sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from verify_heldout_reference import verify_heldout_reference


def bank(tmp_path,label,split,begin,end):
    folder=tmp_path/label;folder.mkdir();path=folder/'bank.npz';path.write_bytes(label.encode())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest=folder/'source-manifest.json'
    manifest.write_text(json.dumps([dict(split=split,begin=begin,end=end,fps=60,source_sha256='source')]))
    qa=dict(accepted_for_training=split=='candidate_training',accepted_for_evaluation=True,visual_review_passed=True,motion_bank_sha256=sha(path),asset_revision='v5',split=split,source_manifest_sha256=sha(manifest))
    path.with_suffix('.qa.json').write_text(json.dumps(qa));return path,sha(path)


def test_disjoint_reviewed_reference_is_explicitly_labeled(tmp_path):
    train,digest=bank(tmp_path,'train','candidate_training',0,300)
    heldout,_=bank(tmp_path,'test','validation',400,700)
    assert verify_heldout_reference(train,heldout,digest,'v5')['mode']=='reviewed_source_disjoint_motion'


def test_overlapping_source_cannot_be_called_heldout(tmp_path):
    train,digest=bank(tmp_path,'train','candidate_training',0,300)
    heldout,_=bank(tmp_path,'test','validation',250,550)
    with pytest.raises(ValueError,match='opposite split'):verify_heldout_reference(train,heldout,digest,'v5')


def test_wrong_training_checkpoint_and_stale_manifest_are_rejected(tmp_path):
    train,digest=bank(tmp_path,'train','candidate_training',0,300)
    heldout,_=bank(tmp_path,'test','validation',400,700)
    with pytest.raises(ValueError,match='checkpoint'):verify_heldout_reference(train,heldout,'wrong','v5')
    (heldout.parent/'source-manifest.json').write_text('[]')
    with pytest.raises(ValueError,match='manifest'):verify_heldout_reference(train,heldout,digest,'v5')
