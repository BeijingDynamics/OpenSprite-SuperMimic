"""Align CPU replay intervals with privileged CPU/EMC clock telemetry."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def stats(a):
    return dict(n=len(a),median=float(np.median(a)),p99=float(np.quantile(a,.99)),maximum=float(np.max(a)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path);p.add_argument('--prefix',required=True)
    args=p.parse_args();root=args.directory;prefix=args.prefix
    clock=np.genfromtxt(root/(prefix+'-clocks.csv'),delimiter=',',names=True)
    assert np.all(np.diff(clock['monotonic_ns'])>0)
    out=root.parent/'power-control';out.mkdir(exist_ok=True)
    fig,axes=plt.subplots(3,1,figsize=(12,7),sharex=True);report={};joined=[]
    anchor=None
    for stage,color,label in [('before','tab:blue','Original dynamic clocks'),('cpufixed','tab:orange','CPU floor = existing 1344 MHz cap'),('restored','tab:green','Original floor restored')]:
        a=np.genfromtxt(root/(prefix+'-'+stage+'-00-pinned.csv'),delimiter=',',names=True)
        meta=json.loads((root/(prefix+'-'+stage+'-00-pinned.json')).read_text())
        assert meta['parity_passed'] and meta['main_affinity']=='4' and meta['worker_affinity']=='5'
        if anchor is None:anchor=a['start_ns'][0]
        idx=np.searchsorted(clock['monotonic_ns'],a['start_ns'],side='right')-1
        assert np.all(idx>=0) and a['end_ns'][-1]<=clock['monotonic_ns'][-1]
        age=(a['start_ns']-clock['monotonic_ns'][idx])/1e6
        mask=(clock['monotonic_ns']>=a['start_ns'][0])&(clock['monotonic_ns']<=a['end_ns'][-1])
        report[stage]=dict(wall_ms=stats(a['wall_ms']),frames_above_6ms=int((a['wall_ms']>6).sum()),
                           clock_sample_age_ms=stats(age),max_action_error=meta['max_action_error'],
                           sampled_cpu_khz=sorted(set(clock['cpu4_khz'][mask].tolist())),
                           sampled_emc_hz=sorted(set(clock['emc_hz'][mask].tolist())),emc_groups={})
        for emc in np.unique(clock['emc_hz'][idx]):
            matched=clock['emc_hz'][idx]==emc
            report[stage]['emc_groups'][str(int(emc))]=dict(wall_ms=stats(a['wall_ms'][matched]),frames_above_6ms=int((a['wall_ms'][matched]>6).sum()))
        for i,row in enumerate(a):
            joined.append(dict(stage=stage,frame=int(row['frame']),start_ns=int(row['start_ns']),end_ns=int(row['end_ns']),wall_ms=float(row['wall_ms']),preceding_cpu_khz=int(clock['cpu4_khz'][idx[i]]),preceding_emc_hz=int(clock['emc_hz'][idx[i]]),clock_sample_age_ms=float(age[i])))
        t=(a['start_ns']-anchor)/1e9
        axes[0].plot(t,a['wall_ms'],color=color,lw=.7,label=label)
        for ax in axes:ax.axvspan(t[0],t[-1],color=color,alpha=.07)
    t=(clock['monotonic_ns']-anchor)/1e9
    axes[1].step(t,clock['cpu4_khz']/1000,where='post',lw=.7)
    axes[2].step(t,clock['emc_hz']/1e6,where='post',lw=.7)
    axes[0].set_ylabel('Inference (ms)');axes[0].legend(fontsize=8,ncol=3)
    axes[1].set_ylabel('CPU 4/5 (MHz)');axes[2].set_ylabel('EMC (MHz)');axes[2].set_xlabel('Monotonic elapsed time (s)')
    for ax in axes:ax.grid(alpha=.2)
    fig.suptitle('50 Hz CPU replay: original clocks -> fixed CPU floor -> restore; same model and outputs')
    fig.tight_layout();fig.savefig(out/'cpu_emc_control.png',dpi=150)
    report['limitations']='Clock samples are asynchronous (10-20 ms typical), not per-instruction measurements. CPU floor intervention also changed EMC behavior; independent CPU versus memory contributions are not isolated.'
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    with (out/'frame-clock-join.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(joined[0]));w.writeheader();w.writerows(joined)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
