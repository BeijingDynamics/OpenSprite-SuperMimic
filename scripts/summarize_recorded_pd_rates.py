"""Summarize paired closed-loop diagnostics and recorded IMU consistency."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/deploy-20261116/pd-rate-analysis'
OUT.mkdir(exist_ok=True)
BASE=ROOT/'outputs/deploy-20261116'
RUNS=['xbox-powered-20261116-152917-5629','xbox-powered-20261116-153129-5687']
PROFILES=['pd500_dt2','pd500_dt1','direct1000_dt1']
DTYPE=np.dtype([('obs','<f4',(993,)),('raw','<f4',(31,)),('action','<f8',(31,)),('target','<f8',(31,))])
fig,axes=plt.subplots(3,2,figsize=(13,10),sharex='col',layout='constrained')
report={'results':[],'imu_consistency':[],
    'imu_limitation':'Gyro-only integration is a consistency diagnostic, not attitude ground truth. Accelerometer direction contains motion acceleration. No causality inferred from correlation.'}
for col,run in enumerate(RUNS):
    real=np.load(BASE/'walking-diagnosis'/(run+'.npz'))
    p=BASE/'live-evidence/supermimic-pm01-xbox30-1116'/run
    record=np.fromfile(p/'native.bin',DTYPE)
    for i,arr in enumerate([real['measured'][:,0]*1000,real['measured'][:,5],np.degrees(real['inputs'][:,12])]):
        axes[i,col].plot(real['time'],arr,color='black',lw=1.4,label='Hardware feedback FK')
    axes[2,col].plot(real['time'],np.degrees(record['target'][:,12]),color='gray',alpha=.6,label='Hardware policy target')
    for folder,profiles in [('pd-rate-diagnostic-v2',PROFILES),('pd-rate-no-turn',['pd500_dt2']),('pd-rate-delay10',['pd500_dt2']),('pd-rate-delay20',['pd500_dt2'])]:
        for profile in profiles:
            z=np.load(BASE/folder/(run+'_'+profile+'.npz'));t=z['time']+20
            label=profile if folder=='pd-rate-diagnostic-v2' else folder.replace('pd-rate-','')
            mask=(t>=40)&(t<50);fwd=z['command'][:,0]>.15
            row=dict(run=run,variant=label,late_torso_roll_median_deg=float(np.median(z['metrics'][mask,2])),
                     late_waist_roll_median_deg=float(np.degrees(np.median(z['q'][mask,12]))),
                     late_waist_target_median_deg=float(np.degrees(np.median(z['target'][mask,12]))),
                     forward_width_median_mm=float(np.median(z['metrics'][fwd,0])*1000),
                     forward_vx_mean_m_s=float(z['metrics'][fwd,5].mean()))
            report['results'].append(row)
            if folder in ['pd-rate-diagnostic-v2','pd-rate-delay20']:
                for i,a in enumerate([z['metrics'][:,0]*1000,z['metrics'][:,2],np.degrees(z['q'][:,12])]):
                    axes[i,col].plot(t,a,alpha=.8,label=label)
    axes[0,col].set_title('Run '+str(col+1));axes[2,col].set_xlabel('Original policy time (s)')
    for i,title in enumerate(['Foot center width (mm)','Torso roll (deg)','Waist roll (deg)']):
        axes[i,col].set_ylabel(title);axes[i,col].grid(alpha=.2);axes[i,col].set_xlim(20,54)
    # Integrate corrected body gyro from the same fused orientation at t=20 s.
    imu=np.genfromtxt(p/'imu.csv',delimiter=',',names=True)
    ipc=np.genfromtxt(p/'native.bin.ipc.csv',delimiter=',',names=True)
    time=(imu['event_ns']-ipc['receive_ns'][0])*1e-9
    valid=(time>=20)&(time<=54)&(imu['bias_ready']!=0)
    imu=imu[valid];time=time[valid]
    quat=np.column_stack([imu[k] for k in ['qx','qy','qz','qw']])
    g=np.column_stack([imu[k] for k in ['gx_body','gy_body','gz_body']])
    omega=np.column_stack([imu[k] for k in ['wx_body','wy_body','wz_body']])
    rot=Rotation.from_quat(quat[0]);gyro_g=[]
    for i in range(len(time)):
        if i:rot=rot*Rotation.from_rotvec((omega[i-1]+omega[i])*.5*(time[i]-time[i-1]))
        gyro_g.append(-rot.as_matrix()[2])
    gyro_g=np.array(gyro_g)
    normalized=g/np.linalg.norm(g,axis=1)[:,None]
    mismatch=np.degrees(np.arccos(np.clip((gyro_g*normalized).sum(1),-1,1)))
    acc=np.column_stack([imu[k] for k in ['ax_body','ay_body','az_body']])
    roll=lambda x:np.degrees(np.arctan2(-x[:,1],-x[:,2]))
    late=(time>=40)&(time<50)
    report['imu_consistency'].append(dict(run=run,frames=len(time),
        gyro_only_vs_fused_gravity_deg_p95=float(np.quantile(mismatch,.95)),
        gyro_only_vs_fused_gravity_deg_final=float(mismatch[-1]),
        late_fused_pelvis_roll_median_deg=float(np.median(roll(normalized)[late])),
        late_gyro_only_pelvis_roll_median_deg=float(np.median(roll(gyro_g)[late])),
        acceleration_norm_p05_p95=np.quantile(np.linalg.norm(acc,axis=1),[.05,.95]).tolist(),
        quaternion_norm_max_error=float(abs(np.linalg.norm(quat,axis=1)-1).max()),
        gravity_norm_max_error=float(abs(np.linalg.norm(g,axis=1)-1).max())))
    np.savez_compressed(OUT/(run+'-imu.npz'),time=time,fused_gravity=g,gyro_only_gravity=gyro_g,mismatch_deg=mismatch)
axes[0,0].legend(fontsize=8);axes[2,1].legend(fontsize=8)
fig.savefig(OUT/'rate_and_posture.png',dpi=140)
(OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,indent=2))
