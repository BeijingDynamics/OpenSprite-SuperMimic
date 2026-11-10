"""Check/enable adapter TX timestamps via ioctl only. Never sends CAN frames."""
import argparse
import ctypes
import fcntl
import json
import os
from pathlib import Path
import socket


class Config(ctypes.Structure):
    _fields_=[('flags',ctypes.c_int),('tx_type',ctypes.c_int),('rx_filter',ctypes.c_int)]


class Ifreq(ctypes.Structure):
    _fields_=[('name',ctypes.c_char*16),('data',ctypes.c_void_p),('padding',ctypes.c_char*16)]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply',action='store_true')
    parser.add_argument('--lock',type=Path,required=True)
    parser.add_argument('--inherited-lock-fd',type=int)
    args=parser.parse_args()
    assert ctypes.sizeof(Ifreq)==40 and ctypes.sizeof(Config)==12
    libc=ctypes.CDLL(None,use_errno=True)
    libc.ioctl.argtypes=(ctypes.c_int,ctypes.c_ulong,ctypes.c_void_p)
    libc.ioctl.restype=ctypes.c_int
    results=[]
    lock_file=(args.lock.open('a') if args.inherited_lock_fd is None else
               os.fdopen(os.dup(args.inherited_lock_fd),'a'))
    assert os.fstat(lock_file.fileno()).st_ino==args.lock.stat().st_ino
    with lock_file as lock, socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for name in ('kcan1','kcan2','kcan3','kcan4'):
            cfg=Config();request=Ifreq(name.encode(),ctypes.addressof(cfg))
            def ioctl(command):
                if libc.ioctl(sock.fileno(),command,ctypes.byref(request)):
                    code=ctypes.get_errno();raise OSError(code,os.strerror(code),name)
            ioctl(0x89b1)  # SIOCGHWTSTAMP, read-only.
            before=dict(tx_type=cfg.tx_type,rx_filter=cfg.rx_filter)
            if args.apply and cfg.tx_type!=1:
                if cfg.rx_filter!=1: raise RuntimeError('RX hardware timestamping is not enabled: '+name)
                cfg.flags=0;cfg.tx_type=1
                ioctl(0x89b0)  # SIOCSHWTSTAMP; preserve RX, bitrate, queues and controller state.
                ioctl(0x89b1)
            results.append(dict(interface=name,before=before,tx_type=cfg.tx_type,rx_filter=cfg.rx_filter))
    print(json.dumps(dict(no_can_frames=True,interfaces=results),indent=2))
    return 0 if all(r['tx_type']==1 and r['rx_filter']==1 for r in results) else 2


if __name__=='__main__': raise SystemExit(main())
