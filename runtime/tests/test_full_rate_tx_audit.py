"""Verify measured 500 Hz delivery, including failures hidden by average rates."""
import csv
import copy
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).parents[1]/'tools'))
from full_rate_tx_audit import audit_full_rate


class FullRateAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)
        self.names={(f'kcan{b}',i):f'm{b}_{i}' for b in range(1,5) for i in range(1,8 if b==4 else 9)}
        self.can=dict(full_rate_ticks=12,duration_s=.024,passed=True)
        self.rows=[];self.rx=[]
        for (bus,motor) in self.names:
            for tick in range(1,13):
                scheduled=1_000_000_000+(tick-1)*2_000_000
                self.rows.append(dict(interface=bus,motor_id=motor,control_tick=tick,
                    sequence=2 if tick<=10 else 3,group=int(tick in (1,11)),scheduled_ns=scheduled,
                    call_begin_ns=scheduled+10_000,call_end_ns=scheduled+20_000,
                    motor_feedback_ns=scheduled-1_400_000,
                    software_realtime_ns=scheduled+30_000,
                    hardware_raw_ns=scheduled+int(bus[-1])*10_000_000_000+motor*10_000,
                    hardware_received_ns=scheduled+700_000))
                self.rx.append(dict(interface=bus,motor_id=motor,feedback_ns=scheduled+600_000,
                                    dequeue_ns=scheduled+800_000))

    def run_audit(self,rows=None,rx=None):
        for name,values in [('can.json.tx.csv',rows if rows is not None else self.rows),
                            ('can.json.rx.csv',rx if rx is not None else self.rx)]:
            with (self.path/name).open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=list(values[0]));writer.writeheader();writer.writerows(values)
        return audit_full_rate(self.path,self.can,self.names)

    def test_full_delivery_and_independent_adapter_clock_origins(self):
        result=self.run_audit()
        self.assertEqual(result['motor_dispatches'],372)
        self.assertTrue(all(r['rate_hz']==500 for r in result['actual_rx'].values()))

    def test_missing_motor_rejected(self):
        with self.assertRaises(AssertionError):self.run_audit(self.rows[1:])

    def test_mixed_action_rejected(self):
        rows=copy.deepcopy(self.rows);rows[2]['sequence']=9
        with self.assertRaises(AssertionError):self.run_audit(rows)

    def test_completion_receipt_is_not_a_wire_deadline(self):
        rows=copy.deepcopy(self.rows);rows[4]['hardware_received_ns']=rows[4]['scheduled_ns']+2_000_001
        self.assertEqual(self.run_audit(rows)['completion_notifications_after_period'],1)

    def test_missing_hardware_stamp_rejected(self):
        rows=copy.deepcopy(self.rows);rows[3]['hardware_raw_ns']=0
        with self.assertRaises(AssertionError):self.run_audit(rows)

    def test_wrong_tick_period_rejected(self):
        rows=copy.deepcopy(self.rows);rows[5]['scheduled_ns']-=10_000
        with self.assertRaises(AssertionError):self.run_audit(rows)

    def test_feedback_lag_is_measured_separately_from_output_period(self):
        rows=copy.deepcopy(self.rows);rows[2]['motor_feedback_ns']-=10_000_000
        self.assertGreater(self.run_audit(rows)['maximum_feedback_age_at_send_ms'],10.)

    def test_low_receive_rate_rejected(self):
        with self.assertRaises(AssertionError):self.run_audit(rx=self.rx[::2])
