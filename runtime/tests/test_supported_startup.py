import importlib.util
import unittest
import numpy as np
from open_sprite_runtime.supported_startup import SupportedStartup


class SupportedStartupTests(unittest.TestCase):
    def fixture(self):return SupportedStartup(np.ones(31)*.1,[1,0,0,0],[0,0],.46)

    def test_preposition_reaches_zero_and_exact_published_gain(self):
        s=self.fixture()
        q,g=s.targets(0);np.testing.assert_array_equal(q,np.ones(31)*.1);self.assertEqual(g,.1)
        for t in [5.,7.,8.,20.]:
            q,g=s.targets(t);np.testing.assert_array_equal(q,np.zeros(31));self.assertEqual(g,1.)

    def test_support_is_exactly_zero_after_release_even_for_displaced_body(self):
        s=self.fixture();self.assertEqual(s.actor_start,7);self.assertEqual(s.release,8)
        self.assertEqual(s.fraction(7),1);self.assertGreater(s.fraction(7.5),0)
        for t in [8.,8.0001,100.]:
            f,tau=s.wrench(t,np.ones(3),np.eye(3),np.ones(3),np.ones(3),np.zeros(3),6.,[0,0,-9.81])
            np.testing.assert_array_equal(f,np.zeros(3));np.testing.assert_array_equal(tau,np.zeros(3))

    @unittest.skipUnless(importlib.util.find_spec('mujoco'),'optional MuJoCo fixture dependency')
    def test_fixture_support_has_explicit_force_and_torque_bounds(self):
        s=self.fixture()
        f,tau=s.wrench(0,np.array([0.,0.,.51]),np.eye(3),np.zeros(3),np.zeros(3),np.array([.1,0,.51]),6.,[0,0,-9.81])
        np.testing.assert_allclose(f,[0,0,58.86],atol=1e-10)
        np.testing.assert_allclose(tau,[0,-5.886,0],atol=1e-10)
        f,tau=s.wrench(0,np.ones(3)*-100,np.eye(3),np.zeros(3),np.zeros(3),np.ones(3)*100,6.,[0,0,-9.81])
        self.assertLessEqual(np.linalg.norm(f),2*58.86+1e-10);self.assertLessEqual(np.linalg.norm(tau),20+1e-10)

    def test_invalid_timing_and_gain_cannot_extend_a_virtual_fixture(self):
        for value in [-1.,0.,np.nan]:
            with self.assertRaises(ValueError):SupportedStartup(np.zeros(31),[1,0,0,0],[0,0],.46,seconds=value)
        with self.assertRaises(ValueError):SupportedStartup(np.zeros(31),[1,0,0,0],[0,0],.46,initial_gain=1.1)
