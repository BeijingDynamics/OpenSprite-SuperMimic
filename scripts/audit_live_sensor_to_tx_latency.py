"""Join recorded CAN, actor, IMU and TX events without hardware access.

All host deltas use monotonic time. Adapter raw clocks are never subtracted
from host clocks. TX completion receipt bounds transmission, not motor action.
"""
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'outputs/deploy-20261116/live-evidence/supermimic-pm01-xbox30-1116'
OUT=ROOT/'outputs/deploy-20261116/sensor-to-tx-latency'
RUNS=['xbox-powered-20261116-152917-5629','xbox-powered-20261116-153129-5687']

def read(path):
    with path.open() as f:return list(csv.DictReader(f))

def stat(a):
    a=np.asarray(a,dtype=float)
    if len(a)==0:return None
    return dict(n=len(a),mean=float(a.mean()),p50=float(np.median(a)),p95=float(np.quantile(a,.95)),p99=float(np.quantile(a,.99)),maximum=float(a.max()),minimum=float(a.min()))

def audit(run):
    p=BASE/run
    actor={int(r['sequence']):{k:int(v) for k,v in r.items()} for r in read(p/'native.bin.ipc.csv')}
    can={int(r['sequence']):r for r in read(p/'can.json.timing.csv')}
    sends=read(p/'action-motor-latency.csv');groups={}
    for s in sends:groups.setdefault(int(s['sequence']),[]).append(s)
    assert len(groups)==2700 and set(groups)==set(actor)-{1}
    inputs=np.fromfile(p/'native.bin.inputs','<f8').reshape(-1,71)
    imu=np.genfromtxt(p/'imu.csv',delimiter=',',names=True)
    iv=np.column_stack([imu[k] for k in ['wx_body','wy_body','wz_body','gx_body','gy_body','gz_body']])
    events=imu['event_ns'];frame_rows=[];motor_rows=[];unmatched=[];ambiguous=0;match_errors=[]
    for seq,packet_group in sorted(groups.items()):
        a=actor[seq];c=can[seq]
        assert a['feedback_ns']==int(c['feedback_ns'])
        assert a['target_ns']==int(c['actor_target_ns'])
        assert len(packet_group)==31 and len({s['motor'] for s in packet_group})==31
        times=[a['feedback_ns'],int(c['publish_ns']),a['receive_ns'],a['infer_start_ns'],a['infer_end_ns'],a['target_ns'],a['sent_ns'],int(c['receive_ns']),int(c['accept_ns'])]
        assert all(y>=x for x,y in zip(times,times[1:])),(seq,times)
        # Identify the consumed IMU sample by its six actual policy input values,
        # not by assuming the actor consumed the newest sensor packet on disk.
        lo=np.searchsorted(events,a['receive_ns']-30_000_000)
        hi=np.searchsorted(events,a['infer_start_ns'],side='right')
        error=np.max(abs(iv[lo:hi]-inputs[seq-1,62:68]),axis=1)
        match=np.flatnonzero(error<1e-10)+lo
        match=match[imu['read_end_ns'][match]<=a['infer_start_ns']]
        if len(match):
            imu_lo=int(events[match].min());imu_hi=int(events[match].max())
            ambiguous+=len(match)>1;match_errors.append(float(error[match[0]-lo]))
        else:imu_lo=imu_hi=None;unmatched.append(seq)
        end_driver=max(int(s['driver_mono_upper_ns']) for s in packet_group)
        end_hw=max(int(s['physical_tx_mono_upper_ns']) for s in packet_group)
        row=dict(sequence=seq,policy_time_s=(seq-1)*.02,oldest_motor=c['oldest_motor'],
            feedback_to_publish_ms=(times[1]-times[0])/1e6,
            state_ipc_ms=(times[2]-times[1])/1e6,
            preprocess_ms=(times[3]-times[2])/1e6,
            inference_ms=(times[4]-times[3])/1e6,
            postprocess_ms=(times[5]-times[4])/1e6,
            actor_send_ms=(times[6]-times[5])/1e6,
            target_ipc_to_accept_ms=(times[8]-times[6])/1e6,
            accepted_to_all_driver_upper_ms=(end_driver-times[8])/1e6,
            target_to_all_driver_upper_ms=(end_driver-times[5])/1e6,
            feedback_to_target_ms=(times[5]-times[0])/1e6,
            feedback_to_all_driver_upper_ms=(end_driver-times[0])/1e6,
            feedback_to_all_tx_receipt_upper_ms=(end_hw-times[0])/1e6,
            imu_sample_match_count=len(match),
            imu_age_at_actor_lower_ms=None if imu_hi is None else (times[2]-imu_hi)/1e6,
            imu_age_at_actor_upper_ms=None if imu_lo is None else (times[2]-imu_lo)/1e6,
            imu_to_target_upper_ms=None if imu_lo is None else (times[5]-imu_lo)/1e6,
            imu_to_all_driver_upper_ms=None if imu_lo is None else (end_driver-imu_lo)/1e6,
            imu_to_all_tx_receipt_upper_ms=None if imu_lo is None else (end_hw-imu_lo)/1e6)
        stages=['feedback_to_publish_ms','state_ipc_ms','preprocess_ms','inference_ms','postprocess_ms','actor_send_ms','target_ipc_to_accept_ms','accepted_to_all_driver_upper_ms']
        assert abs(sum(row[k] for k in stages)-row['feedback_to_all_driver_upper_ms'])<1e-9
        frame_rows.append(row)
        for s in packet_group:
            d=dict(sequence=seq,motor=s['motor'],kind='ankle' if 'ankle_motor' in s['motor'] else 'direct',
                target_to_driver_upper_ms=float(s['actor_target_to_driver_upper_ms']),
                feedback_to_driver_upper_ms=float(s['oldest_feedback_to_driver_upper_ms']),
                feedback_to_tx_receipt_upper_ms=(int(s['physical_tx_mono_upper_ns'])-a['feedback_ns'])/1e6,
                driver_upper_ns=int(s['driver_mono_upper_ns']))
            motor_rows.append(d)
    result=dict(run=run,frames=len(frame_rows),matched_imu_frames=len(frame_rows)-len(unmatched),unmatched_imu_sequences=unmatched,
        ambiguous_imu_frames=ambiguous,maximum_input_matching_error=max(match_errors) if match_errors else None,segments={},motor_groups={})
    for label,predicate in [('all',lambda r:True),('walking',lambda r:20<=r['policy_time_s']<50),('late',lambda r:40<=r['policy_time_s']<50)]:
        rows=[r for r in frame_rows if predicate(r)]
        result['segments'][label]={k:stat([r[k] for r in rows if r[k] is not None]) for k in frame_rows[0] if k.endswith('_ms')}
    for group in ['direct','ankle']:
        result['motor_groups'][group]={k:stat([r[k] for r in motor_rows if r['kind']==group]) for k in ['target_to_driver_upper_ms','feedback_to_driver_upper_ms','feedback_to_tx_receipt_upper_ms']}
    result['worst_frame']=max(frame_rows,key=lambda r:r['feedback_to_all_tx_receipt_upper_ms'])
    result['oldest_motor_counts']={n:sum(r['oldest_motor']==n for r in frame_rows) for n in sorted(set(r['oldest_motor'] for r in frame_rows))}
    result['per_motor']={}
    for name in sorted(set(r['motor'] for r in motor_rows)):
        rows=[r for r in motor_rows if r['motor']==name]
        result['per_motor'][name]={'target_to_driver_upper_ms':stat([r['target_to_driver_upper_ms'] for r in rows]),
                                  'new_action_driver_interval_ms':stat(np.diff([r['driver_upper_ns'] for r in rows])/1e6)}
    with (OUT/(run+'-frames.csv')).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(frame_rows[0]));w.writeheader();w.writerows(frame_rows)
    with (OUT/(run+'-motors.csv')).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(motor_rows[0]));w.writeheader();w.writerows(motor_rows)
    return result,frame_rows

def main():
    OUT.mkdir(exist_ok=True)
    report=dict(scope='Powered-run event joins. All 31 first transmissions of each accepted action are audited.',
        limitations=['CAN feedback_ns is conservative host conversion of kernel software RX, not the motor encoder sampling timestamp.',
        'Physical TX host interval is send-call entry to hardware completion receipt, not an exact motor execution time.',
        'Hardware clocks are not synchronized across adapters or with the host.',
        'IMU DRDY age excludes intrinsic sensor/filter group delay.',
        'Per-motor totals use the oldest source motor of the action, not each motor own original encoder timestamp.'],runs=[],amp=[])
    fig,axes=plt.subplots(2,2,figsize=(13,8),layout='constrained')
    for col,run in enumerate(RUNS):
        result,rows=audit(run);report['runs'].append(result)
        t=[r['policy_time_s'] for r in rows]
        for key,label in [('feedback_to_all_driver_upper_ms','Oldest feedback -> all driver TX (upper)'),('feedback_to_all_tx_receipt_upper_ms','Oldest feedback -> all TX receipts (upper)'),('inference_ms','Inference')]:
            axes[0,col].plot(t,[r[key] for r in rows],label=label,alpha=.8,lw=.7)
        for key,label in [('imu_age_at_actor_upper_ms','IMU DRDY age at actor'),('imu_to_all_tx_receipt_upper_ms','IMU DRDY -> all TX receipts (upper)')]:
            axes[1,col].plot(t,[r[key] for r in rows],label=label,alpha=.8,lw=.7)
        axes[0,col].axhline(20,color='red',ls='--',label='20 ms feedback deadline')
        axes[0,col].set_title('Run '+str(col+1))
        for ax in axes[:,col]:ax.set_ylabel('Milliseconds');ax.set_xlabel('Policy time (s)');ax.legend(fontsize=8);ax.grid(alpha=.2)
    amp=ROOT.parent/'v5_joint_run002/results/outputs/walk025-direct-20261110/xbox-timing-reference'
    for p in sorted(amp.glob('*.json.native/actor.bin.timing')):
        a=np.genfromtxt(p,delimiter=',',names=True)
        report['amp'].append(dict(source=p.parent.name,frames=len(a),compute_ms=stat(a['compute_ms']),
            source_age_ms=stat(a['source_age_ms']),imu_age_before_send_ms=stat(a['imu_age_ms']),
            limitation='compute_ms includes more than ONNX inference; old source_age starts at state publication, not oldest motor feedback. No matched per-action physical TX record here.'))
    (OUT/'summary.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    fig.savefig(OUT/'sensor_to_tx.png',dpi=140)
    for r in report['runs']:
        print(r['run'],'IMU matched',r['matched_imu_frames'],'ambiguous',r['ambiguous_imu_frames'])
        for k,v in r['segments']['all'].items():print(k,v)
    for r in report['amp']:print('AMP',r['source'],r['compute_ms'])

if __name__=='__main__':main()
