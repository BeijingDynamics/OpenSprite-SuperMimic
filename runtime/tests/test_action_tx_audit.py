"""Synthetic transport evidence tests; never opens a hardware device."""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec=importlib.util.spec_from_file_location('action_tx_audit',Path(__file__).parents[1]/'tools/action_tx_audit.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def write_csv(path, rows, delimiter=','):
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]),delimiter=delimiter)
        writer.writeheader();writer.writerows(rows)


class AuditTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.folder=Path(self.temp.name)
        motors=[dict(motor_name=f'm{i}',interface=f'kcan{i%4}',can_id=i//4+1) for i in range(31)]
        write_csv(self.folder/'motors.tsv',motors,'\t')
        can=dict(coherent_action_send=True,coherent_action_groups=1,motor_enable_attempts=0,
                 motors={m['motor_name']:dict(final_disabled=True,nominal_sends=100,cadenced_monitor_samples=100) for m in motors})
        (self.folder/'can.json').write_text(json.dumps(can))
        write_csv(self.folder/'can.json.timing.csv',[dict(sequence=2,feedback_ns=1_000_000_000,
            actor_target_ns=1_004_000_000,accept_ns=1_005_000_000)])
        self.rows=[]
        for i,m in enumerate(motors):
            mono=1_006_000_000+i*10_000
            real=1_700_000_000_000_000_000+i*10_000
            self.rows.append(dict(interface=m['interface'],packet_id=i//4,motor_id=m['can_id'],
                sequence=2,feedback_ns=1_000_000_000,accept_ns=1_005_000_000,group=1,powered=0,
                call_begin_ns=mono,call_end_ns=mono+2000,realtime_before_ns=real,
                software_realtime_ns=real+10000,hardware_raw_ns=100_000_000+i*10000,hardware_received_ns=mono+500000))

    def run_audit(self):
        write_csv(self.folder/'can.json.tx.csv',self.rows)
        return module.audit(self.folder)

    def test_complete_group(self):
        r=self.run_audit()
        self.assertEqual(r['motor_dispatches'],31)
        self.assertLess(r['whole_group_driver_span_max_ms'],.4)

    def test_missing_motor(self):
        self.rows.pop()
        with self.assertRaises(AssertionError): self.run_audit()

    def test_missing_hardware_stamp(self):
        self.rows[8]['hardware_raw_ns']=0
        with self.assertRaises(AssertionError): self.run_audit()

    def test_wrong_action(self):
        self.rows[8]['sequence']=1
        with self.assertRaises(AssertionError): self.run_audit()

    def test_late_driver_send(self):
        self.rows[8]['software_realtime_ns']+=20_000_000
        with self.assertRaises(AssertionError): self.run_audit()

    def test_late_hardware_completion(self):
        self.rows[8]['hardware_received_ns']+=20_000_000
        with self.assertRaisesRegex(AssertionError,'completion'): self.run_audit()

    def test_independent_hardware_clock_origin(self):
        # Per-adapter clock offsets must not alter host-time latency bounds.
        first=self.run_audit()
        for r in self.rows:
            r['hardware_raw_ns']+=(1+int(r['interface'][-1]))*10**17
        second=self.run_audit()
        self.assertEqual(first['physical_tx_delay_upper_max_ms'],second['physical_tx_delay_upper_max_ms'])
        self.assertEqual(first['per_bus_hardware_span_max_ms'],second['per_bus_hardware_span_max_ms'])

    def test_monitor_cadence_lost(self):
        p=self.folder/'can.json';can=json.loads(p.read_text());can['motors']['m0']['cadenced_monitor_samples']=50
        p.write_text(json.dumps(can))
        with self.assertRaisesRegex(AssertionError,'cadence'): self.run_audit()

    def test_incomplete_disable(self):
        p=self.folder/'can.json';can=json.loads(p.read_text());can['motors']['m0']['final_disabled']=False
        p.write_text(json.dumps(can))
        with self.assertRaises(AssertionError): self.run_audit()


if __name__=='__main__': unittest.main()
