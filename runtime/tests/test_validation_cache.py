"""Validation reuse rejects stale identity and damaged evidence without hardware."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('validation_cache',Path(__file__).parents[1]/'tools/select_walk025_validation.py')
cache = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cache)


class ValidationCacheTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.session = Path(self.temp.name)
        self.expected = {'sprite_can_shadow':'can-build','sprite_supermimic_protected_ipc':'actor-build',
                         'run_supermimic_commissioning.py':'launcher','policy.onnx':'policy','sprite_icm45686':'imu'}
        for folder in ('dry','fault'):
            out = self.session/folder
            out.mkdir()
            for name in ('motors.tsv','kinematics.tsv','joints.tsv','feedback.bin','target.bin','input.bin'):
                (out/name).write_text('reviewed '+name)
                self.expected[name] = cache.sha(out/name)
        for folder,mode in (('dry','disabled'),('fault','deadline_fault')):
            report = dict(passed=True,mode=mode,duration_s=49.5,walk_speed_m_s=.25,
                          training_full_body=True,tracked_startup=True,sling_window=True,
                          action_history='policy_output',motor_enable_attempts=0,final_disabled_count=31,
                          can_schedule=dict(cpu=2,policy='SCHED_FIFO',priority=50),fingerprints=self.expected)
            (self.session/folder/'commissioning.json').write_text(json.dumps(report))

    def test_same_build_pair_reusable(self):
        cache.check_reports(self.session,self.expected)

    def test_full_rate_cannot_reuse_mixed_rate_records(self):
        with self.assertRaises(ValueError):
            cache.check_reports(self.session,self.expected,all_joint_500hz=True)
        for mode in ('dry','fault'):
            path=self.session/mode/'commissioning.json'
            report=json.loads(path.read_text());report['all_joint_500hz']=True
            path.write_text(json.dumps(report))
        cache.check_reports(self.session,self.expected,all_joint_500hz=True)
        with self.assertRaises(ValueError):
            cache.check_reports(self.session,self.expected)

    def test_direct_start_cannot_reuse_ramped_start_evidence(self):
        with self.assertRaises(ValueError):
            cache.check_reports(self.session,self.expected,direct_policy_start=True)
        for mode in ('dry','fault'):
            path=self.session/mode/'commissioning.json'
            report=json.loads(path.read_text())
            report['direct_policy_start']=True
            path.write_text(json.dumps(report))
        cache.check_reports(self.session,self.expected,direct_policy_start=True)
        with self.assertRaises(ValueError):
            cache.check_reports(self.session,self.expected,direct_policy_start=False)

    def test_binary_model_launcher_or_configuration_change_rejected(self):
        for name in self.expected:
            with self.subTest(name=name):
                changed = dict(self.expected,**{name:'different'})
                with self.assertRaises(ValueError):
                    cache.check_reports(self.session,changed)

    def test_coherent_send_cannot_reuse_previous_schedule(self):
        with self.assertRaises(ValueError):
            cache.check_reports(self.session,self.expected,coherent_action_send=True)
        for mode in ('dry','fault'):
            path=self.session/mode/'commissioning.json'
            report=json.loads(path.read_text());report['coherent_action_send']=True
            path.write_text(json.dumps(report))
        cache.check_reports(self.session,self.expected,coherent_action_send=True)
        with self.assertRaises(ValueError):
            cache.check_reports(self.session,self.expected)

    def test_failed_wrong_speed_partial_disable_or_powered_report_rejected(self):
        path = self.session/'fault/commissioning.json'
        valid = json.loads(path.read_text())
        for key,value in [('passed',False),('mode','powered'),('walk_speed_m_s',.3),
                          ('final_disabled_count',30),('motor_enable_attempts',31),
                          ('action_history','admitted_waypoint'),('duration_s',30.5)]:
            with self.subTest(key=key):
                changed = copy.deepcopy(valid)
                changed[key] = value
                path.write_text(json.dumps(changed))
                with self.assertRaises(ValueError):
                    cache.check_reports(self.session,self.expected)

    def test_modified_archived_gain_table_rejected(self):
        (self.session/'dry/joints.tsv').write_text('unreviewed gains')
        with self.assertRaises(ValueError):
            cache.check_reports(self.session,self.expected)

    def test_missing_second_stage_rejected(self):
        (self.session/'fault/commissioning.json').unlink()
        with self.assertRaises(FileNotFoundError):
            cache.check_reports(self.session,self.expected)


if __name__ == '__main__':
    unittest.main()
