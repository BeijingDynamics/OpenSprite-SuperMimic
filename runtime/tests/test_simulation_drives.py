import copy
import json
from pathlib import Path
import unittest
import importlib.util

import numpy as np
from open_sprite_runtime.simulation_drives import SimulationDriveBank,link_frame_twist

ROOT=Path(__file__).resolve().parents[1]


class SimulationDriveTests(unittest.TestCase):
    def setUp(self):
        self.hardware=json.loads((ROOT/'config/hardware.sprite0825.measurement.json').read_text())
        self.contract=json.loads((ROOT/'tests/fixtures/legacy_serial_contract.json').read_text())

    def bank(self):
        return SimulationDriveBank(self.hardware,self.contract,allow_nameplate_assumptions=True)

    def test_missing_speed_never_becomes_nan(self):
        with self.assertRaisesRegex(ValueError,'unidentified'):
            SimulationDriveBank(self.hardware,self.contract)
        b=self.bank()
        self.assertTrue(b.assumptions)
        self.assertTrue(np.isfinite(b.limits(np.zeros(31))).all())
        self.assertTrue(np.all(b.limits(np.zeros(31))>0))

    def test_calibrated_transmission_preserves_power(self):
        b=self.bank();r=np.random.default_rng(9)
        for _ in range(20):
            velocity=r.normal(size=31);torque=r.normal(size=31)
            self.assertAlmostEqual(torque@velocity,b.map_joint_torque(torque)@(b.matrix@velocity),places=12)
            np.testing.assert_allclose(b.map_motor_torque(b.map_joint_torque(torque)),torque,atol=1e-12)

    def test_motor_envelope_breakpoints_and_protocol_limit(self):
        b=self.bank()
        np.testing.assert_allclose(b.limits(np.zeros(31)),np.minimum(b.peak,b.protocol_torque))
        np.testing.assert_allclose(b.limits(b.rated_speed),np.minimum(b.rated,b.protocol_torque))
        np.testing.assert_allclose(b.limits(b.no_load_speed),0,atol=1e-12)
        np.testing.assert_allclose(b.limits(b.no_load_speed+1),0,atol=1e-12)

    def test_direct_targets_remain_held_until_50hz_send(self):
        b=self.bank();q=np.zeros(31);v=np.zeros(31)
        a=np.zeros(31);a[0]=.1;b.latch(a,q,v)
        b.step(q,v,q,v,0)
        first=b.motor_target.copy();a[0]=.2;b.latch(a,q,v)
        b.step(q,v,q,v,2)
        np.testing.assert_array_equal(b.motor_target,first)
        b.step(q,v,q,v,40)
        self.assertNotEqual(b.motor_target[0],first[0])

    def test_applied_output_is_finite_and_within_envelope(self):
        b=self.bank();q=np.zeros(31);v=np.zeros(31)
        b.latch(np.full(31,10.),q,v)
        tau=b.step(q,v,q,v,0,lag_seconds=.005)
        self.assertTrue(np.isfinite(tau).all())
        self.assertTrue(np.all(abs(b.output)<=b.limits(b.matrix@v)+1e-12))

    def test_residual_damping_restores_requested_impedance_without_raising_embedded_limit(self):
        contract=copy.deepcopy(self.contract)
        hip=contract['joint_names'].index('left_hip_pitch_joint');contract['damping'][hip]=5.
        q=np.zeros(31);v=np.zeros(31);v[hip]=.4
        baseline=SimulationDriveBank(self.hardware,contract,allow_nameplate_assumptions=True)
        candidate=SimulationDriveBank(self.hardware,contract,allow_nameplate_assumptions=True,
                                      compensate_damping_cap=True)
        for bank in (baseline,candidate):bank.latch(q,q,v)
        old=baseline.step(q,v,q,v,0,envelope=False)
        new=candidate.step(q,v,q,v,0,envelope=False)
        self.assertAlmostEqual(old[hip],-1.2,places=12)
        self.assertAlmostEqual(new[hip],-2.,places=12)
        np.testing.assert_array_equal(candidate.motor_kd,baseline.motor_kd)
        np.testing.assert_array_equal(candidate.limits(np.zeros(31)),baseline.limits(np.zeros(31)))
        np.testing.assert_array_equal(candidate.maximum_speed,baseline.maximum_speed)
        # Host feedforward stays held between50Hz sends; it cannot observe
        # a fresher hidden simulator velocity to manufacture ideal damping.
        newer=v.copy();newer[hip]=.5
        held=candidate.step(q,newer,q,v,2,envelope=False)
        self.assertAlmostEqual(held[hip],-2.3,places=12)
        refreshed=candidate.step(q,newer,q,newer,40,envelope=False)
        self.assertAlmostEqual(refreshed[hip],-2.5,places=12)

    @unittest.skipUnless(importlib.util.find_spec('mujoco'),'optional MuJoCo dependency')
    def test_policy_gyro_uses_link_axes_not_principal_inertia_axes(self):
        import mujoco
        m=mujoco.MjModel.from_xml_string('''<mujoco><worldbody><body name="pelvis_link"><freejoint/>
          <inertial pos=".1 .2 .3" quat=".7071067811865476 .7071067811865476 0 0" mass="1" diaginertia=".02 .025 .03"/>
          <geom type="sphere" size=".1"/></body></worldbody></mujoco>''')
        d=mujoco.MjData(m);d.qvel[:]=[.3,.4,.5,1,2,3];mujoco.mj_forward(m,d)
        actual=link_frame_twist(m,d,1)
        np.testing.assert_allclose(actual[:3],[1,2,3],atol=1e-12)
        np.testing.assert_allclose(actual[3:],[.3,.4,.5],atol=1e-12)
        wrong=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,1,wrong,1)
        self.assertGreater(np.max(abs(wrong-actual)),1.)

    def test_residual_feedforward_cannot_hide_protocol_overflow_by_pd_cancellation(self):
        contract=copy.deepcopy(self.contract)
        hip=contract['joint_names'].index('left_hip_pitch_joint')
        contract['stiffness'][hip]=130.;contract['damping'][hip]=15.
        bank=SimulationDriveBank(self.hardware,contract,allow_nameplate_assumptions=True,
                                 compensate_damping_cap=True)
        q=np.zeros(31);v=np.zeros(31);v[hip]=3.
        target=q.copy();target[hip]=45./130.
        bank.latch(target,q,v);bank.step(q,v,q,v,0)
        motor=bank.names.index('left_hip_pitch_motor')
        self.assertAlmostEqual(bank.held_request[motor],0.,places=10)
        self.assertFalse(bank.command_rejection[motor])
        self.assertTrue(bank.feedforward_rejection[motor])
        self.assertEqual(bank.feedforward_exceedances[motor],1)


if __name__=='__main__':unittest.main()
