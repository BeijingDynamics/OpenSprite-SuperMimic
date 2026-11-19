"""Deterministically exercise the documented echo ownership interleavings."""
import unittest

class Ring:
    def __init__(self):
        self.slot=[None]*256;self.reserved=[False]*256;self.done=[];self.drops=[]
    def reserve(self,n):
        i=n%256
        assert not self.reserved[i]
        self.reserved[i]=True
    def register(self,n):
        i=n%256
        if self.slot[i] is not None:
            self.drops.append(n);return False
        self.slot[i]=n;return True
    def complete(self,n):
        i=n%256
        if self.reserved[i]:
            if self.slot[i] is not None:self.done.append((self.slot[i],n))
            self.slot[i]=None;self.reserved[i]=False

class EchoOrderTests(unittest.TestCase):
    def test_old_order_reproduces_wraparound_misassociation(self):
        r=Ring();r.reserve(0);r.complete(0);r.register(0)
        for n in range(1,256):r.reserve(n);r.register(n);r.complete(n)
        r.reserve(256)
        self.assertFalse(r.register(256))
        r.complete(256)
        self.assertEqual(r.done[-1],(0,256))
        self.assertEqual(r.drops,[256])
    def test_registered_before_publish_handles_early_and_late_completions(self):
        for early in (True,False):
            r=Ring();pending=[]
            for n in range(4096):
                r.reserve(n);self.assertTrue(r.register(n))
                if early:r.complete(n)
                else:
                    pending.append(n)
                    if len(pending)==4:
                        for queued in pending:r.complete(queued)
                        pending=[]
            self.assertEqual(r.done,[(n,n) for n in range(4096)])
            self.assertFalse(any(r.reserved))
    def test_queue_failure_releases_owned_echo_and_marker(self):
        r=Ring();r.reserve(0);self.assertTrue(r.register(0))
        r.slot[0]=None;r.reserved[0]=False
        r.reserve(256);self.assertTrue(r.register(256));r.complete(256)
        self.assertEqual(r.done,[(256,256)])
    def test_registration_failure_never_publishes_a_new_frame(self):
        r=Ring();r.slot[0]=99;r.reserve(256)
        self.assertFalse(r.register(256));r.reserved[0]=False
        self.assertEqual(r.done,[])
        self.assertEqual(r.slot[0],99)

if __name__=='__main__':unittest.main()
