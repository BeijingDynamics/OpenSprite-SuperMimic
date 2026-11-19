"""Unprivileged inference lifetime lease for the fixed CPU 4/5 service."""
import json
import select
import socket


class CpuFrequencyLease:
    def __init__(self,path):
        self.socket=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM)
        try:
            self.socket.settimeout(5);self.socket.connect(str(path))
            self.socket.sendall(b'ACQUIRE 1344000\n')
            self.acquired=self.receive('acquired')
        except BaseException:
            self.socket.close();raise

    def receive(self,status):
        raw=b''
        while not raw.endswith(b'\n'):
            chunk=self.socket.recv(4096)
            if not chunk:raise RuntimeError('CPU frequency guard disconnected')
            raw+=chunk
            if len(raw)>8192:raise RuntimeError('Oversize CPU frequency reply')
        result=json.loads(raw)
        if result.get('status')!=status:raise RuntimeError('CPU frequency lease rejected: '+str(result))
        return result

    def check(self):
        if select.select([self.socket],[],[],0)[0]:
            raise RuntimeError('CPU frequency guard lost its active lease')

    def fileno(self):return self.socket.fileno()

    def close(self,release=True):
        try:
            if release:
                self.socket.sendall(b'RELEASE\n')
                return self.receive('restored')
            return dict(status='retained_by_actor_until_exit')
        finally:self.socket.close()
