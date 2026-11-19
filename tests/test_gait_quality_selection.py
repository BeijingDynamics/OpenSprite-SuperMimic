"""Regression cases for multi-objective model selection."""
import copy
import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('quality',Path(__file__).resolve().parents[1]/'scripts/assess_gait_quality.py')
quality=importlib.util.module_from_spec(spec);spec.loader.exec_module(quality)

class QualityTests(unittest.TestCase):
    def inputs(self):
        rows=[];geoms=[]
        for name in quality.CASES:
            rows.append(dict(case_name=name,survived=True,pitch_median_deg=1.,pitch_abs_p95_deg=2.,
                mean_vx_m_s=.25 if name=='forward025' else .15,
                mean_yaw_rad_s=-.2 if name=='right_curve' else .2,yaw_error_rad_s=.02,
                center_p05_m=.12,edge_gap_p05_m=.04,crossed_fraction=0))
            geoms.append(dict(case_name=name,geometry=dict(event_peak_median_m=[.02,.02],clearance_event_counts=[20,20])))
        pose=dict(checkpoint_sha256='test',results=rows);geometry=dict(checkpoint_sha256='test',results=geoms)
        return pose,geometry,copy.deepcopy(pose),copy.deepcopy(geometry)
    def test_balanced_candidate_passes(self):
        self.assertTrue(quality.assess(*self.inputs())['accepted_for_multiseed_validation'])
    def test_low_right_foot_rejected_even_with_wider_steps(self):
        a=self.inputs();a[3]['results'][1]['geometry']['event_peak_median_m'][1]=.007
        r=quality.assess(*a);self.assertFalse(r['accepted_for_multiseed_validation'])
        self.assertIn('forward025/right_clearance_retained',r['failed_checks'])
    def test_wrong_turn_sign_rejected(self):
        a=self.inputs();a[2]['results'][4]['mean_yaw_rad_s']=.2
        self.assertIn('right_curve/turn_direction_and_response',quality.assess(*a)['failed_checks'])
    def test_low_speed_overshoot_rejected(self):
        a=self.inputs();a[2]['results'][2]['mean_vx_m_s']=.23
        self.assertIn('forward015/speed_tracking_retained',quality.assess(*a)['failed_checks'])
    def test_backward_stand_rejected(self):
        a=self.inputs();a[2]['results'][0]['pitch_median_deg']=-5
        self.assertIn('zero/upright_median',quality.assess(*a)['failed_checks'])

if __name__=='__main__':unittest.main()
