import unittest
import numpy as np
from open_sprite_runtime.command_slew import CommandSlewLimiter

class CommandSlewTests(unittest.TestCase):
    def test_vector_acceleration_and_yaw_bounds_during_reversals(self):
        limiter=CommandSlewLimiter(.25,.3);last=np.zeros(3)
        for goal in ([.3,.3,.15],[-.3,-.3,-.15],[0,0,0]):
            for _ in range(200):
                actual=limiter.step(goal,.02)
                self.assertLessEqual(np.linalg.norm((actual-last)[:2]),.005+1e-12)
                self.assertLessEqual(abs(actual[2]-last[2]),.006+1e-12)
                last=actual
            np.testing.assert_allclose(actual,goal,atol=1e-12)

    def test_rejects_nonfinite_and_invalid_timing(self):
        with self.assertRaises(ValueError):CommandSlewLimiter(0,.3)
        limiter=CommandSlewLimiter(.25,.3)
        for goal,dt in [([np.nan,0,0],.02),([0,0],.02),([0,0,0],0)]:
            with self.assertRaises(ValueError):limiter.step(goal,dt)

    def test_caller_cannot_change_internal_command(self):
        limiter=CommandSlewLimiter(.25,.3);value=limiter.step([.3,0,0],.02);value[:]=9
        self.assertAlmostEqual(limiter.step([.3,0,0],.02)[0],.01)
