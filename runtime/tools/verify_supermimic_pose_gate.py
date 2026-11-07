"""Offline native pose-gate acceptance/rejection cases; no hardware transport."""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess

import numpy as np
from open_sprite_runtime.contracts import PolicyContract
from open_sprite_runtime.native_ipc import ordered_name_hash
from open_sprite_runtime.simulation_drives import SimulationDriveBank


def verify(binary,contract_path,hardware_path,capture,output):
    if hashlib.sha256(hardware_path.read_bytes()).hexdigest()!='9ddf06034f24bbca49cb06f80236caa4cea7aee0040516d11209f848da1f2c10':
        raise ValueError('frozen hardware identity changed')
    c=PolicyContract.load(contract_path)
    if c.data['policy_onnx_sha256']!='f3529c927a44f970908d48e5694e63302d0fc838121aba2fa87c0a0863968bba':
        raise ValueError('frozen actor identity changed')
    b=SimulationDriveBank(json.loads(hardware_path.read_text()),c.data,allow_nameplate_assumptions=True)
    x=np.fromfile(capture/'native.bin.inputs',dtype='<f8').reshape(-1,71)[0]
    q=b.matrix@x[:31]+b.zero;v=b.matrix@x[31:62]
    def expected_gate(pos,vel):
        good=np.isfinite(np.r_[pos,vel]).all() and np.all((pos>=b.hard_lower)&(pos<=b.hard_upper)) and np.all(abs(vel)<=b.maximum_speed)
        return 0 if good else 3
    def record(pos,vel):return b'SPPOSE01'+struct.pack('<Q',ordered_name_hash(b.names))+np.asarray(pos,dtype='<f8').tobytes()+np.asarray(vel,dtype='<f8').tobytes()
    valid=record(b.zero,np.zeros(31))
    cases=[('real_current',record(q,v),expected_gate(q,v)),('urdf_zero',valid,0),
        ('virtual_soft_projection',record(np.clip(q,b.soft_lower,b.soft_upper),v),expected_gate(np.clip(q,b.soft_lower,b.soft_upper),v)),
        ('bad_magic',b'X'+valid[1:],2),('bad_motor_order',valid[:8]+bytes(8)+valid[16:],2),
        ('truncated',valid[:-1],2),('trailing',valid+b'X',2)]
    for name,value in [('nan',np.nan),('inf',np.inf)]:
        modified=b.zero.copy();modified[0]=value;cases.append((name,record(modified,np.zeros(31)),3))
    modified=b.zero.copy();modified[0]=np.nextafter(b.hard_upper[0],np.inf)
    cases.append(('one_ulp_past_hard_limit',record(modified,np.zeros(31)),3))
    speed=np.zeros(31);speed[0]=np.nextafter(b.maximum_speed[0],np.inf)
    cases.append(('one_ulp_past_speed_limit',record(b.zero,speed),3))
    # Independently cross-check every compiled C++ bound against Python's frozen
    # hardware mapping, not only C++'s own embedded self-test constants.
    for i in range(31):
        for edge,label,outward in [(b.hard_lower[i],'lower',-np.inf),(b.hard_upper[i],'upper',np.inf)]:
            pos=b.zero.copy();pos[i]=edge
            cases.append((f'm{i}-{label}-exact',record(pos,np.zeros(31)),0))
            pos[i]=np.nextafter(edge,outward)
            cases.append((f'm{i}-{label}-outside',record(pos,np.zeros(31)),3))
        speed=np.zeros(31);speed[i]=b.maximum_speed[i]
        cases.append((f'm{i}-speed-exact',record(b.zero,speed),0))
        speed[i]=np.nextafter(speed[i],np.inf)
        cases.append((f'm{i}-speed-outside',record(b.zero,speed),3))
    output.mkdir(parents=True,exist_ok=False)
    results=[]
    test=subprocess.run([str(binary),'--self-test'],capture_output=True,text=True,check=True)
    for name,data,expected in cases:
        path=output/(name+'.bin');path.write_bytes(data)
        r=subprocess.run([str(binary),str(path)],capture_output=True,text=True)
        report=json.loads(r.stdout) if r.stdout.strip() else None
        results.append(dict(name=name,expected_returncode=expected,returncode=r.returncode,
            passed=r.returncode==expected,report=report,stderr=r.stderr))
    report=dict(all_passed=all(r['passed'] for r in results),cases=results,self_test=test.stdout.strip(),
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),hardware_tx=0,motor_enable_attempts=0,
        hardware_qualified=False,scope='Recorded physical position/speed guard only; not powered motion admission')
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    if not report['all_passed']:raise RuntimeError('native pose gate conformance failed')
    print(json.dumps({'all_passed':report['all_passed'],'cases':len(results),'real_current_pose_admitted':results[0]['report']['admitted']}))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('binary','contract','hardware','capture','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();verify(a.binary,a.contract,a.hardware,a.capture,a.output)
