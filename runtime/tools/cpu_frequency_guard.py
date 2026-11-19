"""Narrow privileged CPU 4/5 frequency lease; never opens robot devices."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import time

TARGET_KHZ = 1344000


class Frequency:
    def __init__(self, policy, state):
        self.policy, self.state = Path(policy), Path(state)

    def snapshot(self):
        return {name:(self.policy/name).read_text().strip() for name in
                ('scaling_min_freq','scaling_max_freq','scaling_governor','related_cpus')}

    def set_minimum(self, value):
        (self.policy/'scaling_min_freq').write_text(str(value)+'\n')
        deadline=time.monotonic()+3
        while (self.policy/'scaling_min_freq').read_text().strip()!=str(value):
            if time.monotonic()>deadline:
                raise RuntimeError('CPU frequency readback did not settle')
            time.sleep(.02)

    def acquire(self):
        if self.state.exists():raise RuntimeError('Unrestored frequency state exists')
        before=self.snapshot()
        if (before['related_cpus']!='4 5' or int(before['scaling_max_freq'])!=TARGET_KHZ
                or not 0<int(before['scaling_min_freq'])<=TARGET_KHZ):
            raise RuntimeError('Expected CPU 4/5 cluster and existing 1344000 kHz cap')
        temporary=self.state.with_suffix('.tmp')
        temporary.write_text(json.dumps(before)+'\n');os.chmod(temporary,0o600)
        temporary.replace(self.state)
        try:
            self.set_minimum(TARGET_KHZ)
            self.verify()
        except BaseException:
            self.restore();raise
        return dict(status='acquired',target_khz=TARGET_KHZ,before=before,active=self.snapshot(),
                    guard_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

    def verify(self):
        current=self.snapshot()
        if (int(current['scaling_min_freq'])!=TARGET_KHZ or int(current['scaling_max_freq'])!=TARGET_KHZ):
            raise RuntimeError('CPU frequency lease changed externally')

    def restore(self):
        if not self.state.exists():return dict(status='restored',changed=False)
        before=json.loads(self.state.read_text())
        self.set_minimum(before['scaling_min_freq'])
        after=self.snapshot()
        if after!=before:raise RuntimeError('CPU settings differ after restoration')
        self.state.unlink()
        return dict(status='restored',changed=True,before=before,after=after)


def send(conn, message):
    conn.sendall(json.dumps(message).encode()+b'\n')


def serve(path, frequency, uid):
    frequency.restore()
    path=Path(path);path.unlink(missing_ok=True)
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
    server.bind(str(path));os.chmod(path,0o660);server.listen(4)
    try:
        while True:
            conn,_=server.accept()
            with conn:
                try:
                    _,peer_uid,_=struct.unpack('3i',conn.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))
                    if peer_uid not in (0,uid):raise RuntimeError('Unauthorized lease peer')
                    conn.settimeout(3)
                    request=conn.recv(256)
                    if request!=b'ACQUIRE 1344000\n':raise RuntimeError('Invalid lease request')
                    acquired=frequency.acquire();send(conn,acquired)
                    print(json.dumps(acquired),flush=True)
                    conn.settimeout(None)
                    while True:
                        ready,_,_=select.select([conn,server],[],[],.1)
                        if server in ready:
                            other,_=server.accept()
                            with other:send(other,dict(status='error',error='Frequency lease already active'))
                        if conn in ready:
                            message=conn.recv(256)
                            if not message:break
                            if message!=b'RELEASE\n':raise RuntimeError('Invalid lease command')
                            restored=frequency.restore();send(conn,restored)
                            print(json.dumps(restored),flush=True);break
                        frequency.verify()
                except (OSError,RuntimeError,ValueError) as exc:
                    try:send(conn,dict(status='error',error=str(exc)))
                    except OSError:pass
                    print(json.dumps(dict(status='error',error=str(exc))),flush=True)
                finally:
                    restored=frequency.restore()
                    if restored.get('changed'):print(json.dumps(restored),flush=True)
    finally:
        frequency.restore();server.close();path.unlink(missing_ok=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',type=Path,default=Path('/sys/devices/system/cpu/cpufreq/policy4'))
    parser.add_argument('--directory',type=Path,default=Path('/run/open-sprite-cpu-frequency'))
    parser.add_argument('--uid',type=int,required=True)
    parser.add_argument('--restore',action='store_true')
    args=parser.parse_args();args.directory.mkdir(parents=True,exist_ok=True)
    frequency=Frequency(args.policy,args.directory/'state.json')
    if args.restore:print(json.dumps(frequency.restore()));return
    def stop(signum,frame):raise SystemExit(0)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    serve(args.directory/'lease.sock',frequency,args.uid)


if __name__=='__main__':main()
