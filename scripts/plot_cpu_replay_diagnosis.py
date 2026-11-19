"""Plot CPU-only pacing controls and summarize operator profile attribution."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    args=p.parse_args();root=args.directory
    selected=[('baseline-05-single4','Single thread'),('baseline-04-shared','Two threads / shared mask'),
              ('baseline-03-pinned','Two threads / split affinity'),('busy-repeat-00-pinned','Split affinity / active wait'),
              ('sleep-return-00-pinned','Split affinity / return to sleep')]
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    values=[np.genfromtxt(root/(name+'.csv'),delimiter=',',names=True) for name,label in selected]
    positions=np.arange(len(values));med=[np.median(v['wall_ms']) for v in values];p99=[np.quantile(v['wall_ms'],.99) for v in values]
    axes[0,0].barh(positions-.17,med,height=.32,label='Median')
    axes[0,0].barh(positions+.17,p99,height=.32,label='P99')
    axes[0,0].set_yticks(positions,[label for name,label in selected],fontsize=8)
    axes[0,0].set_xlabel('Inference wall time (ms)');axes[0,0].legend();axes[0,0].invert_yaxis()
    for name,label,color in [('diag-01-pinned','Sleep between frames','tab:blue'),('busy-00-pinned','Active wait between frames','tab:orange')]:
        data=np.genfromtxt(root/(name+'.csv'),delimiter=',',names=True)
        axes[0,1].scatter(data['freq4_before']/1000,data['wall_ms'],s=8,alpha=.25,label=label,color=color)
        axes[1,0].scatter(data['main_cpu_ms'],data['wall_ms'],s=8,alpha=.25,label=label,color=color)
    axes[0,1].set_xlabel('CPU 4 frequency before inference (MHz)');axes[0,1].set_ylabel('Inference (ms)');axes[0,1].legend(fontsize=8)
    axes[1,0].plot([0,9],[0,9],'k--',lw=.6);axes[1,0].set_xlabel('Main-thread CPU time (ms)');axes[1,0].set_ylabel('Inference wall time (ms)')
    for index in [2,3,4]:
        data=values[index];axes[1,1].plot(data['frame']*.02,data['wall_ms'],lw=.6,alpha=.7,label=selected[index][1])
    axes[1,1].set_xlabel('Replay time (s)');axes[1,1].set_ylabel('Inference (ms)');axes[1,1].legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.2)
    fig.suptitle('Same policy and recorded inputs; CPU-only replay at 50 Hz')
    fig.tight_layout();fig.savefig(root.parent/'cpu_replay_controls.png',dpi=150)
    profile=next(root.glob('profile-00-pinned_*.json'))
    events=json.loads(profile.read_text());runs=[];nodes=[]
    for e in events:
        if e['cat']=='Node':nodes.append(e)
        if e['name']=='model_run':runs.append((e,nodes));nodes=[]
    assert len(runs)==600
    result={}
    for label,pred in [('fast_under4ms',lambda t:t<4000),('slow_6to10ms',lambda t:6000<t<10000)]:
        selected=[nodes for s,nodes in runs[100:] if pred(s['dur'])];durations=defaultdict(list);wait=defaultdict(list)
        for nodes in selected:
            for n in nodes:
                durations[n['name']].append(n['dur'])
                wait[n['name']].append(n['args'].get('thread_scheduling_stats',{}).get('main_thread',{}).get('Wait',0))
        result[label]=dict(frames=len(selected),nodes={name:dict(median_duration_us=float(np.median(values)),mean_duration_us=float(np.mean(values)),median_wait_us=float(np.median(wait[name]))) for name,values in durations.items()})
    result['limitation']='Instrumented profile, excludes 100 warmups. Operator profiling changes overhead; use unprofiled trials for timing qualification.'
    (root.parent/'operator-attribution.json').write_text(json.dumps(result,indent=2)+'\n')


if __name__=='__main__':main()
