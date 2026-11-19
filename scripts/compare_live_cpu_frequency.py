"""Join accepted action timing in disabled CPU-frequency integration runs."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def read(path):
    with path.open() as f:return list(csv.DictReader(f))


def stats(a):
    a=np.asarray(a,float)
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.quantile(a,.95)),p99=float(np.quantile(a,.99)),maximum=float(a.max()),minimum=float(a.min()))


def audit(path):
    report=json.loads((path/'commissioning.json').read_text())
    tx=json.loads((path/'action-tx-audit.json').read_text())
    assert tx['passed'] and report['final_disabled_count']==31 and report['motor_enable_attempts']==0
    actor={int(r['sequence']):{k:int(v) for k,v in r.items()} for r in read(path/'native.bin.ipc.csv')}
    timing={int(r['sequence']):r for r in read(path/'can.json.timing.csv')}
    groups={}
    for r in read(path/'action-motor-latency.csv'):groups.setdefault(int(r['sequence']),[]).append(r)
    rows=[]
    for seq,g in sorted(groups.items()):
        assert len(g)==31 and len({x['motor'] for x in g})==31
        a=actor[seq];c=timing[seq]
        assert a['feedback_ns']==int(c['feedback_ns']) and a['target_ns']==int(c['actor_target_ns'])
        last_driver=max(int(x['driver_mono_upper_ns']) for x in g)
        last_hw=max(int(x['physical_tx_mono_upper_ns']) for x in g)
        hip=next(x for x in g if x['motor']=='left_hip_pitch_motor')
        r=dict(sequence=seq,time_s=(seq-1)*.02,inference_ms=(a['infer_end_ns']-a['infer_start_ns'])/1e6,
            feedback_to_publish_ms=(int(c['publish_ns'])-a['feedback_ns'])/1e6,
            feedback_to_target_ms=(a['target_ns']-a['feedback_ns'])/1e6,
            target_to_all_driver_ms=(last_driver-a['target_ns'])/1e6,
            feedback_to_all_driver_ms=(last_driver-a['feedback_ns'])/1e6,
            feedback_to_all_tx_receipt_ms=(last_hw-a['feedback_ns'])/1e6,
            hip_driver_upper_ns=int(hip['driver_mono_upper_ns']),hip_hw_ns=int(hip['hardware_raw_ns']))
        if rows:
            assert seq==rows[-1]['sequence']+1
            r['hip_driver_interval_ms']=(r['hip_driver_upper_ns']-rows[-1]['hip_driver_upper_ns'])/1e6
            r['hip_hw_interval_ms']=(r['hip_hw_ns']-rows[-1]['hip_hw_ns'])/1e6
        else:r['hip_driver_interval_ms']=r['hip_hw_interval_ms']=None
        rows.append(r)
    assert len(groups)==tx['groups'] if 'groups' in tx else len(groups)>0
    inferred=[a for seq,a in actor.items() if seq>1 and a['infer_end_ns']>a['infer_start_ns']>0]
    summary=dict(name=path.name,passed=report['passed'],failure=report['failure'],frames_dispatched=len(rows),
                 final_disabled_count=31,motor_enable_attempts=0,cpu_frequency=report.get('cpu_frequency'),
                 metrics={k:stats([r[k] for r in rows if r[k] is not None]) for k in rows[0] if k.endswith('_ms')},
                 all_inference_ms_including_rejected=stats([(a['infer_end_ns']-a['infer_start_ns'])/1e6 for a in inferred]),
                 first_sequence=rows[0]['sequence'],last_sequence=rows[-1]['sequence'],
                 unmatched_actor_sequences=[seq for seq in actor if seq>1 and seq not in groups])
    return summary,rows,report['fingerprints']


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('base',type=Path);p.add_argument('--runs',nargs='+',required=True)
    args=p.parse_args();out=args.base.parent/'comparison';out.mkdir(exist_ok=True)
    summaries=[];fingerprints=[]
    for name in args.runs:
        s,rows,fp=audit(args.base/name);summaries.append(s);fingerprints.append(fp)
        with (out/(name+'.csv')).open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        print(name,s['passed'],'dispatched',s['frames_dispatched'])
        for key in ['inference_ms','feedback_to_all_driver_ms','feedback_to_all_tx_receipt_ms','hip_driver_interval_ms','hip_hw_interval_ms']:print(key,s['metrics'][key])
    stable=['policy.onnx','sprite_icm45686','sprite_can_shadow','sprite_supermimic_protected_ipc','motors.tsv','kinematics.tsv','joints.tsv','feedback.bin','target.bin','input.bin','loaded_kcan_srcversion']
    for key in stable:assert len({f[key] for f in fingerprints})==1,key
    result=dict(runs=summaries,unchanged_fingerprints={k:fingerprints[0][k] for k in stable},
                limitations=['Failed baseline statistics cover only accepted/dispatched actions; its rejected inference is reported separately.',
                             'Feedback timestamp is a conservative host conversion of RX, not encoder sampling.',
                             'TX receipt is a host upper bound, not motor-internal execution.',
                             'Same-adapter hardware intervals do not compare different adapter clocks.',
                             'Disabled zero-gain transport is not powered-motion qualification.'])
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
