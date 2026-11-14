"""Protect recovery reporting against stride oscillation, drift and resets."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('matched',Path(__file__).resolve().parents[1]/'scripts/summarize_command_matched_recovery.py')
matched=importlib.util.module_from_spec(spec);spec.loader.exec_module(matched)


class RecoveryReportTests(unittest.TestCase):
    def make_report(self,bias=0.,failure=None,heading_change=0.):
        count=1000;fps=50.;time=np.arange(count)/fps
        velocity=np.zeros((count,2,3));velocity[:,:,0]=(.20+.08*np.sin(2*np.pi*time))[:,None]
        velocity[300:,1,0]+=bias
        qpos=np.zeros((count,2,38));qpos[:,:,3]=1
        yaw_rate=np.zeros((count,2))
        angle=heading_change*np.clip((time-6.)/.2,0,1)
        qpos[:,1,3]=np.cos(angle/2);qpos[:,1,6]=np.sin(angle/2)
        yaw_rate[:,1]=np.gradient(angle)*fps
        failed=np.zeros((count,2),dtype=bool)
        if failure is not None:failed[failure,1]=True
        cases=[dict(env=index,label=[.25,0,0],case_name=name,survived=failure is None or index==0,
                    first_failure_seconds=failure/fps if index==1 and failure is not None else None,
                    forces=[dict(at_seconds=6.,duration_seconds=.2,force_world_n=[5,0,0])] if index else [])
               for index,name in enumerate(['nominal','pushed'])]
        with tempfile.TemporaryDirectory() as temporary:
            directory=Path(temporary)
            (directory/'summary.json').write_text(json.dumps(dict(iteration=1,seed=101,results=cases)))
            np.savez(directory/'rollout.npz',fps=fps,velocity=velocity,qpos=qpos,failed=failed,
                     commands=np.broadcast_to([.25,0,0],(count,2,3)),yaw_rate=yaw_rate)
            return matched.summarize(directory)['results'][0]

    def test_stride_oscillation_is_not_permanent_failure(self):
        report=self.make_report()
        self.assertAlmostEqual(report['events'][0]['matched_recovery_confirmed_seconds'],1.96)
        self.assertAlmostEqual(report['last5s_signed_command_error_m_s'][0],-.05)

    def test_sustained_bias_is_not_hidden_by_averaging(self):
        report=self.make_report(bias=.15)
        self.assertIsNone(report['events'][0]['matched_recovery_confirmed_seconds'])
        self.assertAlmostEqual(report['last5s_nominal_velocity_difference_m_s'][0],.15)

    def test_failure_before_recovery_never_uses_later_reset_frames(self):
        report=self.make_report(failure=340)
        self.assertFalse(report['survived'])
        self.assertIsNone(report['events'][0]['matched_recovery_confirmed_seconds'])
        self.assertEqual(report['first_failure_seconds'],6.8)

    def test_new_world_heading_does_not_block_velocity_recovery(self):
        report=self.make_report(heading_change=np.deg2rad(60))
        self.assertIsNotNone(report['events'][0]['matched_recovery_confirmed_seconds'])
        self.assertAlmostEqual(report['accumulated_heading_difference_deg'],60.)


if __name__=='__main__':unittest.main()
