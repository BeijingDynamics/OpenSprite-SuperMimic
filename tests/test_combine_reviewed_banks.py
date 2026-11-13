"""Composition must preserve source samples and reject stale or held-out banks."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from combine_reviewed_banks import combine, sha, FIELDS


class ReviewedCompositionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.bank=self.root/'source.npz'
        arrays={key:np.arange(12,dtype=np.float32).reshape(4,3) for key in FIELDS}
        np.savez(self.bank,**arrays,fps=np.array(50.),asset_revision=np.array('fixture'),joint_names=np.array(['a']),body_names=np.array(['pelvis_link']))
        self.qa=dict(accepted_for_training=True,visual_review_passed=True,motion_bank_sha256=sha(self.bank),asset_revision='fixture')
        self.qa_path=self.bank.with_suffix('.qa.json');self.qa_path.write_text(json.dumps(self.qa))
        self.selection=self.root/'selection.json'
        self.selection.write_text(json.dumps([dict(bank=str(self.bank),single_clip_name='fixture',source_rows=[dict(name='fixture',split='candidate_training',source_sha256='source',begin=0,end=4,fps=50)])]))

    def tearDown(self):
        self.temp.cleanup()

    def test_preserves_samples_and_requires_composition_review(self):
        out=self.root/'new'/'bank.npz';combine(self.selection,out)
        with np.load(out) as merged,np.load(self.bank) as original:
            for key in FIELDS:np.testing.assert_array_equal(merged[key],original[key])
        self.assertFalse(json.loads(out.with_suffix('.qa.json').read_text())['visual_review_passed'])

    def test_stale_source_is_rejected(self):
        self.qa['motion_bank_sha256']='stale';self.qa_path.write_text(json.dumps(self.qa))
        with self.assertRaisesRegex(ValueError,'changed'):combine(self.selection,self.root/'new'/'bank.npz')

    def test_validation_source_is_rejected(self):
        self.qa['split']='validation';self.qa_path.write_text(json.dumps(self.qa))
        with self.assertRaisesRegex(ValueError,'Held-out'):combine(self.selection,self.root/'new'/'bank.npz')


if __name__=='__main__':unittest.main()
