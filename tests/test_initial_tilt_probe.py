"""Verify bounded reproducible rotations without requiring a GPU."""
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from initial_tilt_probe import sample_tilt

class TiltTests(unittest.TestCase):
    def test_zero_is_identity(self):
        r,a=sample_tilt(5,0,101)
        np.testing.assert_array_equal(r,np.tile(np.eye(3),(5,1,1)))
        np.testing.assert_array_equal(a,0)
    def test_paired_seeds_and_rotations(self):
        r,a=sample_tilt(5,2,202);r2,a2=sample_tilt(5,2,202)
        np.testing.assert_array_equal(r,r2);np.testing.assert_array_equal(a,a2)
        self.assertFalse(np.array_equal(a,sample_tilt(5,2,303)[1]))
        self.assertLessEqual(np.abs(a).max(),np.deg2rad(2))
        np.testing.assert_allclose(r@r.transpose(0,2,1),np.tile(np.eye(3),(5,1,1)),atol=1e-14)
        np.testing.assert_allclose(np.linalg.det(r),1,atol=1e-14)
    def test_invalid_bound_rejected(self):
        for value in [-1,6,float('nan')]:
            with self.assertRaises(ValueError):sample_tilt(5,value,101)

if __name__=='__main__':unittest.main()
