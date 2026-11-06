"""50Hz recorded motor-state IPC + synthetic IMU; actor never transmits targets."""
import argparse
import json
from pathlib import Path
import socket
import struct
import subprocess
import tempfile
import threading
import time

import numpy as np

from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.native_ipc import NativeStatePacket, ordered_name_hash
from open_sprite_runtime.simulation_drives import SimulationDriveBank
from open_sprite_runtime.supermimic_replay import checked_model, inputs_from_rollout, write_fixture, NATIVE_RECORD


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('contract','hardware','rollout','binary','offline-binary','output-dir'):
        p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--seconds',type=int,default=30)
    p.add_argument('--fault',choices=('none','state_stale','state_order','state_disconnect','imu_freeze'),default='none')
    a=p.parse_args();a.output_dir.mkdir(parents=True,exist_ok=False)
    c=PolicyContract.load(a.contract);model=checked_model(c)
    bank=SimulationDriveBank(json.loads(a.hardware.read_text()),c.data,allow_nameplate_assumptions=True)
    with np.load(a.rollout,allow_pickle=False) as data:
        inputs=inputs_from_rollout(data,c,3)[:a.seconds*50]
    count=len(inputs);fixture=a.output_dir/'input.bin';write_fixture(fixture,c,inputs,100)
    config=a.output_dir/'feedback.bin'
    mh=ordered_name_hash(bank.names)
    config.write_bytes(b'SPMSEN01'+struct.pack('<Q',mh)+bank.zero.astype('<f8').tobytes()+bank.inverse.astype('<f8').tobytes())
    # Body->sensor rotation of the existing rear-pelvis mounting.
    mount=np.array([[0,0,-1],[0,-1,0],[-1,0,0.]])
    errors=[];shared={'index':0};stop=threading.Event();sent=[0];reverse_bytes=[0];fault_at=[None]
    with tempfile.TemporaryDirectory(prefix='smipc_') as folder:
        state_path=str(Path(folder)/'state.sock');imu_path=str(Path(folder)/'imu.sock')
        server=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET);server.bind(state_path);server.listen(1);server.settimeout(.1)
        imu=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM);imu.bind(imu_path);imu.settimeout(.05)

        def states():
            try:
                while not stop.is_set():
                    try:conn,_=server.accept();break
                    except socket.timeout:continue
                else:return
                with conn:
                    conn.settimeout(.2);start=time.monotonic()+.03
                    for i,row in enumerate(inputs):
                        if stop.wait(max(0,start+i*.02-time.monotonic())):break
                        if i==100 and a.fault!='none':fault_at[0]=time.monotonic()
                        if i==100 and a.fault=='state_disconnect':return
                        now=time.monotonic_ns();shared['index']=i
                        stamp=now-100_000_000 if i>=100 and a.fault=='state_stale' else now
                        seq=i+1 if not(i>=100 and a.fault=='state_order') else i
                        q=bank.matrix@row[:31]+bank.zero;v=bank.matrix@row[31:62]
                        conn.sendall(NativeStatePacket(seq,stamp,mh,tuple(q),tuple(v)).pack());sent[0]+=1
                    # SHUT_WR guarantees EOF here: no target or motor packet can emerge.
                    reverse_bytes[0]=len(conn.recv(4096))
            except (BrokenPipeError,ConnectionResetError):
                if a.fault=='none':errors.append('unexpected state disconnect')
            except Exception as ex:errors.append('state '+repr(ex))

        def imu_states():
            seq=0;frozen=None
            try:
                while not stop.is_set():
                    try:request,peer=imu.recvfrom(64)
                    except socket.timeout:continue
                    if request!=b'GETIMU01':raise ValueError('unexpected IMU request')
                    row=inputs[shared['index']];seq+=1;now=time.monotonic_ns()
                    gyro=mount.T@row[62:65];counts=np.rint(gyro*900).astype(int)
                    sensor_up=-(mount.T@row[65:68]);sensor_up/=np.linalg.norm(sensor_up)
                    w=np.sqrt(max(0,(1+sensor_up[2])/2))
                    xyz=np.cross(sensor_up,[0,0,1])/(2*w) if w>1e-8 else np.array([1,0,0.])
                    quat=np.r_[w,xyz];qw,qx,qy,qz=quat
                    up=np.array([2*(qx*qz-qw*qy),2*(qy*qz+qw*qx),1-2*(qx*qx+qy*qy)])
                    values=np.r_[np.zeros(3),counts/900.,quat,9.80665*up]
                    raw=struct.pack('<9h',0,0,0,0,0,0,*counts)
                    packet=b'SPIMU002'+struct.pack('<QQQ',seq,now-500_000,now)+raw+bytes(6)+struct.pack('<3d',0,0,0)+values.astype('<f8').tobytes()+bytes(8)
                    if a.fault=='imu_freeze' and shared['index']>=100:
                        if frozen is None:frozen=packet
                        packet=frozen
                    try:imu.sendto(packet,peer)
                    except (FileNotFoundError,ConnectionRefusedError):return
            except Exception as ex:errors.append('imu '+repr(ex))

        workers=[threading.Thread(target=f) for f in (states,imu_states)]
        for t in workers:t.start()
        begin=time.monotonic()
        run=subprocess.run([str(a.binary.resolve()),str(model.resolve()),str(fixture.resolve()),str(a.output_dir/'native.bin'),
            '--state-socket',state_path,'--imu-socket',imu_path,'--feedback-config',str(config.resolve())],
            capture_output=True,text=True,timeout=a.seconds+15)
        end=time.monotonic();stop.set()
        for t in workers:t.join(2)
        server.close();imu.close()
    (a.output_dir/'native.log').write_text(run.stdout+run.stderr)
    parity=None;timing=None
    if run.returncode==0:
        consumed=np.fromfile(a.output_dir/'native.bin.inputs',dtype='<f8').reshape(-1,71)
        f=a.output_dir/'consumed.bin';write_fixture(f,c,consumed,100)
        check=subprocess.run([str(a.offline_binary.resolve()),str(model.resolve()),str(f.resolve()),str(a.output_dir/'offline.bin')],capture_output=True,text=True)
        parity=check.returncode==0 and (a.output_dir/'offline.bin').read_bytes()==(a.output_dir/'native.bin').read_bytes()
        timing=json.loads((a.output_dir/'native.bin.timing.json').read_text())
    expected={'state_stale':'stale','state_order':'sequence','state_disconnect':'disconnected','imu_freeze':'stale'}
    passed=not errors and reverse_bytes[0]==0 and not any(t.is_alive() for t in workers)
    if a.fault=='none':passed=passed and run.returncode==0 and parity and sent[0]==count
    else:passed=passed and run.returncode!=0 and expected[a.fault] in run.stderr and fault_at[0] is not None and end-fault_at[0]<1
    report={'passed':bool(passed),'mode':'recorded_realtime_state_ipc_receive_only','fault':a.fault,'frames_sent':sent[0],
        'state_socket_reverse_bytes':reverse_bytes[0],'hardware_tx':0,'motor_enable_attempts':0,'physical_sensor_access':False,
        'offline_byte_exact':parity,'timing':timing,'returncode':run.returncode,'stderr':run.stderr,'errors':errors,
        'fault_exit_seconds':end-fault_at[0] if fault_at[0] else None,'model_sha256':c.data['policy_onnx_sha256']}
    (a.output_dir/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
    raise SystemExit(0 if passed else 1)


if __name__=='__main__':main()
