import json
from pathlib import Path
import unittest

import numpy as np
from open_sprite_runtime.scheduled_drives import SlotScheduledDriveBank

ROOT=Path(__file__).resolve().parents[1]


class ScheduledDriveTests(unittest.TestCase):
    def bank(self, repeat=False, wire=False):
        hardware=json.loads((ROOT/'config/hardware.sprite0825.measurement.json').read_text())
        contract=json.loads((ROOT/'tests/fixtures/legacy_serial_contract.json').read_text())
        return SlotScheduledDriveBank(hardware,contract,repeat_held_commands=repeat,wire_quantization=wire)

    def test_sparse_slots_never_collide_on_a_bus(self):
        b=self.bank()
        for tick in range(40):
            buses=[r['can_channel'] for r,due in zip(b.records,b.due(tick)) if due]
            self.assertEqual(len(buses),len(set(buses)))
        np.testing.assert_array_equal(sum(b.due(tick) for tick in range(40)),np.where(b.ankle,10,1))

    def test_repeat_schedule_is_bounded_two_frames_per_bus_slot(self):
        b=self.bank(True)
        for tick in range(4):
            for bus in range(4):
                self.assertLessEqual(sum(r['can_channel']==bus and due for r,due in zip(b.records,b.due(tick))),2)
        np.testing.assert_array_equal(sum(b.due(tick) for tick in range(40)),np.full(31,10))

    def test_direct_drive_waits_for_send_and_internal_pd_clock(self):
        b=self.bank();q=np.zeros(31);target=q.copy();target[0]=.1
        i=b.names.index('left_hip_pitch_motor');self.assertEqual(b.send_phase[i],1)
        b.latch(target,q,q)
        b.step(q,q,q,q,0,envelope=False);self.assertEqual(b.held_request[i],0)
        b.step(q,q,q,q,1,envelope=False);self.assertEqual(b.held_request[i],0)
        b.step(q,q,q,q,2,envelope=False);self.assertGreater(abs(b.held_request[i]),0)

    def test_b_ankle_sends_the_held_500hz_preview(self):
        b=self.bank();q=np.zeros(31);b.latch(q,q,q)
        b.step(q,q,q,q,0,envelope=False)
        i=b.names.index('left_ankle_motor_b');self.assertEqual(b.send_phase[i],2)
        new_v=np.zeros(31);new_v[b.mapping.policy_joint_names.index('left_ankle_pitch_joint')]=.4
        b.step(q,new_v,q,new_v,2,envelope=False)
        self.assertAlmostEqual(b.held_request[i],0,places=12)
        b.step(q,new_v,q,new_v,4,envelope=False)
        b.step(q,new_v,q,new_v,6,envelope=False)
        self.assertGreater(abs(b.held_request[i]),0)

    def test_at_send_ankle_candidate_uses_fresh_feedback_without_more_sends(self):
        b=self.bank();b.ankle_at_send=True;q=np.zeros(31);b.latch(q,q,q)
        b.step(q,q,q,q,0,envelope=False)
        i=b.names.index('left_ankle_motor_b')
        new_v=np.zeros(31);new_v[b.mapping.policy_joint_names.index('left_ankle_pitch_joint')]=.4
        b.step(q,new_v,q,new_v,2,envelope=False)
        self.assertGreater(abs(b.held_request[i]),0)
        self.assertEqual(b.command_sends[i],1)

    def test_repeat_does_not_increase_target_update_frequency(self):
        b=self.bank(True);q=np.zeros(31);a=q.copy();a[0]=.1
        b.latch(a,q,q)
        for tick in range(4):b.step(q,q,q,q,tick,envelope=False)
        i=b.names.index('left_hip_pitch_motor');first=b.motor_target[i]
        a[0]=.2;b.latch(a,q,q)
        for tick in range(4,40):
            b.step(q,q,q,q,tick,envelope=False);self.assertEqual(b.motor_target[i],first)
        b.step(q,q,q,q,40,envelope=False);b.step(q,q,q,q,41,envelope=False)
        self.assertNotEqual(b.motor_target[i],first)

    def test_guard_uses_received_feedback_and_retains_raw_violation(self):
        b=self.bank(True);q=np.zeros(31);target=q.copy();target[0]=1
        b.latch(target,q,q,soft_limits=False)
        b.step(q,q,q,q,0);b.step(q,q,q,q,1);b.step(q,q,q,q,2)
        i=b.names.index('left_hip_pitch_motor')
        self.assertTrue(b.instantaneous_rejection[i]);self.assertGreater(b.saturations[i],0)
        self.assertLessEqual(abs(b.output[i]),b.limits(q)[i])
        # Clipped physical output cannot turn an unsafe raw request into a pass.
        b.step(q,q,q,q,4);self.assertTrue(b.command_rejection[i])

    def test_residual_compensation_keeps_the_same_embedded_limits(self):
        a=self.bank();b=self.bank(True)
        np.testing.assert_array_equal(a.motor_kd,b.motor_kd)
        np.testing.assert_array_equal(a.maximum_speed,b.maximum_speed)
        np.testing.assert_array_equal(a.limits(np.zeros(31)),b.limits(np.zeros(31)))
        self.assertLessEqual(b.motor_kd.max(),3)

    def test_arrival_preview_avoids_second_free_running_50hz_wait(self):
        b=self.bank(True);b.arrival_driven_preview=True
        q=np.zeros(31);b.latch(q,q,q)
        for tick in range(24):b.step(q,q,q,q,tick,envelope=False)
        a=q.copy();a[0]=.1;b.latch(a,q,q)
        for tick in range(24,28):b.step(q,q,q,q,tick,envelope=False)
        i=b.names.index('left_hip_pitch_motor')
        self.assertAlmostEqual(b.motor_target[i]-b.zero[i],(b.matrix@a)[i])
        held=b.preview_ff.copy()
        for tick in range(28,40):b.step(q,q,q,np.ones(31),tick,envelope=False)
        np.testing.assert_array_equal(b.preview_ff,held)

    def test_wire_zero_has_the_official_signed_midpoint_bias(self):
        b=self.bank(True)
        decoded=b.quantize(np.zeros(31),b.torque_min,b.torque_max,12)
        np.testing.assert_allclose(decoded,b.torque_min+(b.torque_max-b.torque_min)*2047/4095,atol=1e-14)
        self.assertTrue(np.all(decoded<0))

    def test_representable_position_stays_inside_unchanged_soft_limits(self):
        b=self.bank(True)
        for value in (b.soft_lower,b.soft_upper):
            decoded=b.quantize_position(value)
            self.assertTrue(np.all(decoded>=b.soft_lower))
            self.assertTrue(np.all(decoded<=b.soft_upper))
            self.assertTrue(np.all(abs(decoded-value)<(b.position_max-b.position_min)/65535+1e-12))

    def test_wire_model_uses_quantized_fields_for_the_actual_request(self):
        b=self.bank(True,True)
        q=np.zeros(31);b.latch(q,q,q)
        for tick in range(4):b.step(q,q,q,q,tick,envelope=False)
        i=b.names.index('left_hip_pitch_motor')
        expected=b.wire_kp[i]*(b.motor_target[i]-b.zero[i])+b.wire_kd[i]*b.wire_velocity[i]+b.held_ff[i]
        self.assertAlmostEqual(b.held_request[i],expected,places=12)
        self.assertNotEqual(expected,0)

    def test_startup_gain_scale_never_changes_limits_or_exceeds_published_gains(self):
        b=self.bank(True,True);kp=b.kp.copy();kd=b.kd.copy();cap=b.limits(np.zeros(31));speed=b.maximum_speed.copy()
        for scale in (0.,.1,.4,1.):
            b.set_gain_scale(scale)
            np.testing.assert_array_equal(b.kp,kp*scale);np.testing.assert_array_equal(b.kd,kd*scale)
            self.assertLessEqual(b.motor_kd.max(),3)
            np.testing.assert_array_equal(b.limits(np.zeros(31)),cap)
            np.testing.assert_array_equal(b.maximum_speed,speed)
        for scale in (-.01,1.01,np.nan):
            with self.assertRaises(ValueError):b.set_gain_scale(scale)

    def test_gain_change_waits_for_that_motors_can_slot(self):
        b=self.bank(True,True);q=np.zeros(31);b.latch(q,q,q)
        for tick in range(4):b.step(q,q,q,q,tick)
        i=b.names.index('left_hip_pitch_motor');old=b.wire_kp[i]
        b.set_gain_scale(.1)
        self.assertEqual(b.wire_kp[i],old)
        b.step(q,q,q,q,4)
        self.assertLess(b.wire_kp[i],old)


if __name__=='__main__':unittest.main()
