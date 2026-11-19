"""Summarize isolated native CPU replay evidence without modifying raw files."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def stats(a):
    a=np.asarray(a,float)
    if not len(a):return None
    return dict(n=len(a),median=float(np.median(a)),p95=float(np.quantile(a,.95)),
                p99=float(np.quantile(a,.99)),maximum=float(a.max()),minimum=float(a.min()))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    args=p.parse_args();reports=[]
    for f in sorted(args.directory.glob('*.csv')):
        if not f.with_suffix('.json').exists():continue
        meta=json.loads(f.with_suffix('.json').read_text())
        with f.open() as stream: rows=list(csv.DictReader(stream))
        data={k:np.asarray([float(r[k]) for r in rows]) for k in rows[0]}
        wall=data['wall_ms']; tail=wall>=6;fast=wall<4
        r=dict(name=f.stem,metadata=meta,wall_ms=stats(wall),
               over_6ms=int(tail.sum()),over_8ms=int((wall>=8).sum()),
               migrations_within_inference=int((data['cpu_before']!=data['cpu_after']).sum()),
               final_half_wall_ms=stats(wall[len(wall)//2:]),groups={})
        if meta['diagnostic']:
            for label,mask in [('all',np.ones(len(wall),bool)),('fast_under4',fast),('tail_over6',tail)]:
                r['groups'][label]={k:stats(data[k][mask]) for k in ['wall_ms','process_cpu_ms','main_cpu_ms','main_wait_ms','worker_wait_ms','involuntary','voluntary','freq4_before','freq4_after']}
                r['groups'][label]['wall_minus_main_cpu_ms']=stats((wall-data['main_cpu_ms'])[mask])
            r['frequency_groups']={str(int(freq)):stats(wall[data['freq4_before']==freq]) for freq in np.unique(data['freq4_before'])}
            r['worst_frames']=[rows[int(i)] for i in np.argsort(wall)[-5:][::-1]]
        reports.append(r)
        print(f'{f.stem}: p50={np.median(wall):.3f} p99={np.quantile(wall,.99):.3f} max={wall.max():.3f} >=6ms={tail.sum()}/{len(wall)} parity={meta["parity_passed"]}')
    (args.directory.parent/'probe-summary.json').write_text(json.dumps(reports,indent=2)+'\n')


if __name__=='__main__':main()
