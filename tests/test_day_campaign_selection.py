"""Check that research ranking cannot silently qualify a failed nominal run."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

path=Path(__file__).resolve().parents[1]/'scripts/run_day_campaign.py'
spec=importlib.util.spec_from_file_location('day_campaign',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class SelectionTests(unittest.TestCase):
    def test_survival_ranking_and_acceptance_remain_separate(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for n,failures,survived,pitch in [(500,3,True,2),(1000,2,True,3),(2000,0,False,0)]:
                folder=root/f'eval-{n}';folder.mkdir()
                (folder/'gait-quality-selection.json').write_text(json.dumps({
                    'failed_checks':['failure']*failures,'accepted_for_multiseed_validation':failures==0}))
                (folder/'posture-width.json').write_text(json.dumps({'results':[
                    {'survived':survived,'pitch_median_deg':pitch}]}))
                model=root/f'train-{n}';model.mkdir();(model/f'model_{n:06d}.pt').touch()
            selected=module.select_candidates(root)
            self.assertEqual([x[1] for x in selected],[1000,500])
            self.assertTrue(all(x[3] is False for x in selected))
            (root/'train-1000/model_001000.pt').unlink()
            self.assertEqual([x[1] for x in module.select_candidates(root)],[500])

if __name__=='__main__':unittest.main()
