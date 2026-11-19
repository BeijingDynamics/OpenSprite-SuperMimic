"""Linux lease lifecycle tests using temporary files, never real CPU controls."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

TOOLS=Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0,str(TOOLS))
from cpu_frequency_client import CpuFrequencyLease


@unittest.skipUnless(sys.platform=='linux','Linux Unix peer credentials required')
class LeaseLifecycle(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.policy=self.root/'policy';self.policy.mkdir();self.run=self.root/'run'
        for name,value in dict(scaling_min_freq='729600',scaling_max_freq='1344000',scaling_governor='schedutil',related_cpus='4 5').items():
            (self.policy/name).write_text(value+'\n')
        self.server=None;self.start()

    def start(self):
        self.server=subprocess.Popen([sys.executable,str(TOOLS/'cpu_frequency_guard.py'),'--policy',str(self.policy),'--directory',str(self.run),'--uid',str(os.getuid())],stdout=subprocess.DEVNULL)
        self.wait(lambda:(self.run/'lease.sock').exists())

    def wait(self,predicate):
        until=time.monotonic()+4
        while not predicate():
            if time.monotonic()>until:self.fail('Lifecycle condition timed out')
            time.sleep(.02)

    def floor(self):return (self.policy/'scaling_min_freq').read_text().strip()

    def lease(self):return CpuFrequencyLease(self.run/'lease.sock')

    def tearDown(self):
        if self.server is not None and self.server.poll() is None:self.server.terminate();self.server.wait(timeout=5)
        self.temp.cleanup()

    def test_normal_release(self):
        lease=self.lease();self.assertEqual(self.floor(),'1344000')
        result=lease.close();self.assertEqual(result['after']['scaling_min_freq'],'729600')
        self.assertFalse((self.run/'state.json').exists())

    def test_client_sigkill_restores(self):
        code='import sys,time;sys.path.insert(0,sys.argv[1]);from cpu_frequency_client import CpuFrequencyLease;l=CpuFrequencyLease(sys.argv[2]);print("ready",flush=True);time.sleep(60)'
        child=subprocess.Popen([sys.executable,'-c',code,str(TOOLS),str(self.run/'lease.sock')],stdout=subprocess.PIPE,text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(),'ready');child.kill();child.wait(timeout=3)
            self.wait(lambda:self.floor()=='729600')
        finally:
            if child.poll() is None:child.kill();child.wait()
            child.stdout.close()

    def test_inherited_actor_keeps_lease(self):
        lease=self.lease()
        child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(.4)'],pass_fds=(lease.fileno(),))
        lease.close(release=False);self.assertEqual(self.floor(),'1344000')
        child.wait(timeout=3);self.wait(lambda:self.floor()=='729600')

    def test_daemon_sigkill_restart_recovers(self):
        lease=self.lease();self.server.kill();self.server.wait(timeout=3)
        self.assertEqual(self.floor(),'1344000')
        with self.assertRaises(RuntimeError):lease.check()
        lease.close(release=False)
        (self.run/'lease.sock').unlink(missing_ok=True)
        self.start();self.wait(lambda:self.floor()=='729600')
        self.lease().close()

    def test_wrong_cap_rejected_without_mutation(self):
        (self.policy/'scaling_max_freq').write_text('1190400\n')
        with self.assertRaises(RuntimeError):self.lease()
        self.assertEqual(self.floor(),'729600')

    def test_competing_client_rejected(self):
        first=self.lease()
        with self.assertRaises((OSError,RuntimeError)):self.lease()
        self.assertEqual(self.floor(),'1344000');first.close()

    def test_external_change_notifies_and_restores(self):
        lease=self.lease();(self.policy/'scaling_min_freq').write_text('960000\n')
        self.wait(lambda:self.floor()=='729600')
        with self.assertRaises(RuntimeError):lease.check()
        lease.close(release=False)


if __name__=='__main__':unittest.main(verbosity=2)
