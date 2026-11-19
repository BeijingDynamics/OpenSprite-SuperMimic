"""Audit recorded powered runs; no robot connection or control parameter changes."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
import mujoco
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'outputs/cpufreq-powered-20261117'
CONTRACT = json.loads((ROOT/'outputs/deploy-20261116/live-bundle10000/contract.json').read_text())
XML = ROOT/'assets/opensprite/opensprite.xml'
assert hashlib.sha256(XML.read_bytes()).hexdigest() == CONTRACT['asset_mjcf_sha256']
MODEL = mujoco.MjModel.from_xml_path(str(XML))
STATE = mujoco.MjData(MODEL)
NAMES = CONTRACT['joint_names']
QIDS = [int(MODEL.joint(n).qposadr[0]) for n in NAMES]
DTYPE = np.dtype([('obs','<f4',(993,)),('raw','<f4',(31,)),('action','<f8',(31,)),('target','<f8',(31,))])

def stats(x):
    x=np.asarray(x)
    if not len(x): return None
    return dict(n=len(x),mean=float(x.mean()),p05=float(np.quantile(x,.05)),median=float(np.median(x)),p95=float(np.quantile(x,.95)),p99=float(np.quantile(x,.99)),min=float(x.min()),max=float(x.max()))

def pitch(g):
    return np.degrees(np.arctan2(g[:,0],np.hypot(g[:,1],g[:,2])))

def gravity(q):
    w,x,y,z=q.T
    return -np.column_stack([2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)])

def geometry(q,g):
    result=[]
    for pose,gr in zip(q,g):
        roll=np.arctan2(-gr[1],-gr[2]);p=np.arctan2(gr[0],np.hypot(gr[1],gr[2]))
        cr,sr,cp,sp=np.cos(roll),np.sin(roll),np.cos(p),np.sin(p)
        R=np.array([[cp,sp*sr,sp*cr],[0,cr,-sr],[-sp,cp*sr,cp*cr]])
        STATE.qpos[:3]=0;mujoco.mju_mat2Quat(STATE.qpos[3:7],R.ravel())
        STATE.qpos[QIDS]=pose;mujoco.mj_kinematics(MODEL,STATE)
        U=STATE.xmat[MODEL.body('waist_yaw_link').id].reshape(3,3)
        feet=[STATE.geom_xmat[MODEL.geom(s+'_foot_collision').id].reshape(3,3) for s in ('left','right')]
        normal=R.T@(feet[0][:,2]+feet[1][:,2]);normal/=np.linalg.norm(normal)
        sole_pitch=[np.degrees(np.arctan2(-F[2,0],np.hypot(F[2,1],F[2,2]))) for F in feet]
        result.append([np.degrees(np.arctan2(U[0,2],U[2,2])),np.degrees(np.arctan2(U[1,2],U[2,2])),pitch(-normal[None])[0],*sole_pitch])
    return np.asarray(result)

report={'sign':'Positive pelvis pitch and torso up-axis fore/aft lean mean forward. Negative means backward.',
        'operator_report':'Sling remained slack. No pushing while stationary. Push tests occurred while moving. Backward lean was most obvious at zero command; occasional backward travel occurred despite a forward command.',
        'method':'Exact-training-asset forward kinematics and timestamp-matched IMU packet audit, not dynamics replay.',
        'limitations':['No independent optical attitude or contact/force ground truth.',
                       'Gyro-only integration can drift and is not attitude ground truth.',
                       'Accelerometer tilt includes translational acceleration.',
                       'Flat-sole pelvis pitch assumes both soles are level; invalid during swing or edge contact.',
                       'Policy-target FK is an instantaneous geometry diagnostic, not an achieved pose.'], 'runs':[]}
completed=[]
for p in sorted(BASE.glob('xbox-powered-*')):
    if not p.is_dir(): continue
    r=json.loads((p/'commissioning.json').read_text())
    entry={k:r.get(k) for k in ['passed','failure','motor_enable_attempts','final_disabled_count','operator_events','timing_max_ms']}
    entry.update(run=p.name,cpu_restoration=r['cpu_frequency']['released']['status'])
    report['runs'].append(entry)
    if r.get('motor_enable_attempts')!=31: continue
    completed.append((p,entry))
fig,axs=plt.subplots(4,len(completed),figsize=(5*len(completed),13),squeeze=False,layout='constrained')
for col,(p,entry) in enumerate(completed):
    inp=np.fromfile(p/'native.bin.inputs','<f8').reshape(-1,71)
    rec=np.fromfile(p/'native.bin',DTYPE)
    ipc=np.genfromtxt(p/'native.bin.ipc.csv',delimiter=',',names=True,dtype=None,encoding=None)
    imu=np.genfromtxt(p/'imu.csv',delimiter=',',names=True)
    assert len(inp)==len(rec)==len(ipc)
    t=(ipc['infer_start_ns']-ipc['infer_start_ns'][0])/1e9
    events=imu['event_ns'];it=(events-ipc['infer_start_ns'][0])/1e9
    a=np.column_stack([imu[k+'_body'] for k in ['ax','ay','az']])
    w=np.column_stack([imu[k+'_body'] for k in ['wx','wy','wz']])
    g=np.column_stack([imu[k+'_body'] for k in ['gx','gy','gz']])
    q=np.column_stack([imu[k] for k in ['qw','qx','qy','qz']])
    ready=imu['bias_ready']>0;first=np.flatnonzero(ready)[0]
    raw_acc=-np.column_stack([imu[k] for k in ['raw_az','raw_ay','raw_ax']])*4*9.80665/32768
    raw_w=-np.column_stack([imu[k] for k in ['raw_gz','raw_gy','raw_gx']])*2000*np.pi/180/32768
    bias=np.mean(raw_w[:first+1],axis=0)
    entry['sensor_check']={'raw_accel_conversion_max_error':float(abs(raw_acc-a).max()),
       'gyro_bias_rad_s':bias.tolist(),'corrected_gyro_max_error':float(abs(raw_w[ready]-bias-w[ready]).max()),
       'quaternion_gravity_max_error':float(abs(gravity(q[ready])-g[ready]).max()),
       'observation_gravity_max_error':float(abs(rec['obs'][:,987:990]-inp[:,65:68]).max()),
       'observation_gyro_max_error':float(abs(rec['obs'][:,957:960]-inp[:,62:65]).max())}
    values=np.column_stack([w,g]);matched=[];age=[]
    for i,row in enumerate(ipc):
        lo=np.searchsorted(events,int(row['receive_ns'])-30_000_000);hi=np.searchsorted(events,row['infer_start_ns'],side='right')
        err=np.max(abs(values[lo:hi]-inp[i,62:68]),axis=1)
        hits=np.flatnonzero(err<1e-10)+lo
        assert len(hits)==1,(p.name,i,len(hits))
        matched.append(hits[0]);age.append((int(row['infer_start_ns'])-events[hits[0]])/1e6)
    matched=np.array(matched)
    entry['sensor_check']['matched_policy_frames']=len(matched)
    entry['sensor_check']['imu_age_at_inference_ms']=stats(age)
    gyro_q=q[first].copy();gyro_g=np.zeros_like(g)
    gyro_g[first]=gravity(gyro_q[None])[0]
    for j in range(first+1,len(imu)):
        mujoco.mju_quatIntegrate(gyro_q,w[j],(events[j]-events[j-1])/1e9)
        gyro_g[j]=gravity(gyro_q[None])[0]
    pel=pitch(inp[:,65:68]);acc=pitch(-a/np.linalg.norm(a,axis=1)[:,None]);gyrop=pitch(gyro_g)
    actual=geometry(inp[:,:31],inp[:,65:68]);target=geometry(rec['target'],inp[:,65:68])
    entry['segments']={}
    masks={'initial_0_1s':(t>=0)&(t<1),'zero_10_20s':(t>=10)&(t<20),
       'xbox_zero':(t>=20)&(t<50)&(abs(inp[:,68])<.01)&(abs(inp[:,70])<.02),
       'xbox_forward':(t>=20)&(t<50)&(inp[:,68]>.15),
       'early_22_32s':(t>=22)&(t<32),'late_40_50s':(t>=40)&(t<50),'stop_52_54s':t>=52}
    for label,m in masks.items():
        if not np.any(m):continue
        ii=matched[m]
        s={'pelvis_pitch_deg':stats(pel[m]),'torso_foreaft_lean_deg':stats(actual[m,0]),
           'target_torso_foreaft_lean_deg':stats(target[m,0]),'flat_sole_pelvis_pitch_deg':stats(actual[m,2]),
           'left_sole_pitch_deg':stats(actual[m,3]),'right_sole_pitch_deg':stats(actual[m,4]),
           'accel_pitch_deg':stats(acc[ii]),'gyro_only_pitch_deg':stats(gyrop[ii]),
           'fused_minus_gyro_pitch_deg':stats(pel[m]-gyrop[ii]),'command_mean':inp[m,68:].mean(0).tolist(),'joints':{}}
        for name in NAMES:
            if any(x in name for x in ['hip_pitch','knee','ankle_pitch','waist','shoulder_pitch']):
                k=NAMES.index(name);s['joints'][name]={'measured_deg':stats(np.degrees(inp[m,k])),
                    'target_deg':stats(np.degrees(rec['target'][m,k])),
                    'target_minus_measured_deg':stats(np.degrees(rec['target'][m,k]-inp[m,k]))}
        entry['segments'][label]=s
    inference=(ipc['infer_end_ns'][1:]-ipc['infer_start_ns'][1:])/1e6
    entry['inference_ms']=stats(inference)
    worst=int(np.argmax(inference))+1
    entry['worst_inference']={'time_s':float(t[worst]),'duration_ms':float(inference[worst-1]),'command':inp[worst,68:].tolist(),'pelvis_pitch_deg':float(pel[worst])}
    can=json.loads((p/'can.json').read_text());entry['can_errors']=can['errors']
    entry['motor_tracking']={k:{kk:v[kk] for kk in ['final_disabled','torque_saturation_count','max_command_torque_nm','max_feedback_torque_nm']} for k,v in can['motors'].items()}
    assert sum(v['final_disabled'] for v in can['motors'].values())==31
    for ax in axs[:,col]:ax.axvspan(20,50,color='green',alpha=.06);ax.grid(alpha=.2)
    axs[0,col].set_title(p.name.split('-')[-2]);axs[0,col].plot(t,inp[:,68],label='Forward command (m/s)');axs[0,col].plot(t,inp[:,70],label='Yaw command (rad/s)')
    axs[1,col].plot(t,pel,label='Fused pelvis pitch');axs[1,col].plot(t,gyrop[matched],label='Gyro-only pitch',alpha=.7)
    axs[1,col].plot(t,actual[:,0],label='Torso fore/aft lean',alpha=.7)
    axs[2,col].plot(t,actual[:,3],label='Left sole pitch');axs[2,col].plot(t,actual[:,4],label='Right sole pitch')
    axs[3,col].plot(t,actual[:,2],label='Pelvis pitch if both soles level');axs[3,col].plot(t,pel,label='Fused pelvis pitch')
    for i in [1,2,3]:axs[i,col].axhline(0,color='black',lw=.6);axs[i,col].set_ylabel('Degrees')
    for ax in axs[:,col]:ax.legend(fontsize=7)
    axs[3,col].set_xlabel('Control time (s)')
    np.savez_compressed(BASE/(p.name+'-posture.npz'),time=t,inputs=inp,measured=actual,target=target,gyro_pitch=gyrop[matched])
fig.savefig(BASE/'backward-lean.png',dpi=140)
sim=np.load(ROOT/'outputs/deploy-20261116/seed45-quarter10000/rollout.npz')
report['simulation_context']=[]
for env in [0,1,3,4]:
    poses=sim['qpos'][150:,env];g=gravity(poses[:,3:7]);geom=geometry(poses[:,7:],g)
    targets=sim['policy_action'][150:,env]*np.asarray(CONTRACT['action_scale'])+np.asarray(CONTRACT['default_joint_pos'])
    joint_errors={n:stats(np.degrees(targets[:,k]-poses[:,7+k])) for k,n in enumerate(NAMES) if 'ankle_pitch' in n or 'hip_pitch' in n}
    report['simulation_context'].append(dict(case=str(sim['case_names'][env]),pelvis_pitch_deg=stats(pitch(g)),torso_foreaft_lean_deg=stats(geom[:,0]),joint_target_error_deg=joint_errors))
source=ROOT/'opensprite_tracker/supermimic/joint_interfaces.py'
manifest=json.loads((ROOT/'outputs/retrain-20261114/seed45-lr-continuation/source_manifest.json').read_text())
assert hashlib.sha256(source.read_bytes()).hexdigest()==manifest['opensprite_tracker/supermimic/joint_interfaces.py']
report['reward_source_sha256']=manifest['opensprite_tracker/supermimic/joint_interfaces.py']
(BASE/'posture-report.json').write_text(json.dumps(report,indent=2)+'\n')
fig2,ax=plt.subplots(1,2,figsize=(12,4.4),layout='constrained')
labels=['Simulation']+[e['run'].split('-')[-2] for _,e in completed]
standing=[report['simulation_context'][0]['pelvis_pitch_deg']['median']]+[e['segments']['zero_10_20s']['pelvis_pitch_deg']['median'] for _,e in completed]
forward=[report['simulation_context'][1]['pelvis_pitch_deg']['median']]+[e['segments']['xbox_forward']['pelvis_pitch_deg']['median'] for _,e in completed]
x=np.arange(len(labels));ax[0].bar(x-.18,standing,.36,label='Zero command');ax[0].bar(x+.18,forward,.36,label='Forward command >0.15 m/s')
ax[0].set_xticks(x,labels,rotation=20);ax[0].set_ylabel('Median pelvis pitch (deg); negative = backward')
ax[0].set_title('Same policy; simulation is not a matched replay');ax[0].legend(fontsize=8)
last=np.load(BASE/(completed[-1][0].name+'-posture.npz'));tt=last['time']
ax[1].plot(tt,pitch(last['inputs'][:,65:68]),label='Fused pelvis pitch');ax[1].plot(tt,last['gyro_pitch'],label='Gyro-only pitch',alpha=.65)
ax[1].axvspan(20,50,color='green',alpha=.06);ax[1].set_title('Last trial: backward lean precedes motion')
ax[1].set_xlabel('Control time (s)');ax[1].set_ylabel('Pitch (deg)');ax[1].legend(fontsize=8)
for aa in ax:aa.axhline(0,color='black',lw=.7);aa.grid(axis='y',alpha=.2)
fig2.savefig(BASE/'backward-lean-summary.png',dpi=150)
print(BASE/'posture-report.json')
for e in report['runs']:
    print(e['run'],e['motor_enable_attempts'],e['final_disabled_count'],e['failure'])
    for k,s in e.get('segments',{}).items():print(k,'pelvis',round(s['pelvis_pitch_deg']['median'],2),'torso',round(s['torso_foreaft_lean_deg']['median'],2),'gyro',round(s['gyro_only_pitch_deg']['median'],2),'accel',round(s['accel_pitch_deg']['median'],2),'flat',round(s['flat_sole_pelvis_pitch_deg']['median'],2))
