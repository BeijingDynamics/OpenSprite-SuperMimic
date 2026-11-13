"""Check diagnostic profile isolation and motoring/braking sign semantics."""
import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('sensitivity', Path(__file__).resolve().parents[1] / 'scripts/evaluate_actuator_sensitivity.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SensitivityTest(unittest.TestCase):
    def test_single_factor_profiles(self):
        self.assertEqual(module.PROFILES['baseline'], {})
        for name, fields in module.PROFILES.items():
            self.assertLessEqual(len(fields), 1, name)

    def test_braking_and_constant_cap(self):
        bounds = module.torque_bounds_scalar
        self.assertEqual(bounds(0, 40, 10, 28), (-28, 28))
        self.assertEqual(bounds(8, 40, 10, 28)[0], -28)
        self.assertAlmostEqual(bounds(8, 40, 10, 28)[1], 8)
        self.assertEqual(bounds(12, 40, 10, 28), (-28, 0))
        self.assertEqual(bounds(-12, 40, 10, 28), (0, 28))
        for speed in [-15, -3, 0, 3, 15]:
            lo, hi = bounds(speed, 40, 10, 28)
            rev_lo, rev_hi = bounds(-speed, 40, 10, 28)
            self.assertAlmostEqual(lo, -rev_hi)
            self.assertAlmostEqual(hi, -rev_lo)

    def test_invalid_speed(self):
        with self.assertRaises(ValueError):
            module.torque_bounds_scalar(0, 40, 0, 28)


if __name__ == '__main__':
    unittest.main()
