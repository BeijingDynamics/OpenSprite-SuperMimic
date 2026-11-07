"""Convert private CAN receive timing into an offline simulation sensitivity trace.

No device access. Kernel realtime -> monotonic conversion is estimated from the
minimum observed clock offset, with an explicit additional sample-age margin.
This is not an identified motor sampling model or a hardware admission test.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def prepare(capture, output, seconds, sample_margin_ms):
    if not np.isfinite(sample_margin_ms) or sample_margin_ms < 0 or not 1 <= seconds <= 50:
        raise ValueError('invalid bounded trace settings')
    inventory=[line.split() for line in (capture/'fixtures/inventory.tsv').read_text().splitlines()]
    if len(inventory)!=31 or len({r[1] for r in inventory})!=31:raise ValueError('invalid motor inventory')
    arrivals=[[] for _ in inventory];software=[[] for _ in inventory]
    with (capture/'can.csv').open() as stream:
        for row in csv.DictReader(stream):
            i=int(row['motor_index']);arrivals[i].append(int(row['monotonic_ns']));software[i].append(int(row['software_timestamp_ns']))
    arrivals=[np.array(v,dtype=np.int64) for v in arrivals]
    software=[np.array(v,dtype=np.int64) for v in software]
    if any(len(v)<2 or np.any(np.diff(v)<0) for v in arrivals):raise ValueError('missing/reordered receive times')
    offset=min(int((a-s).min()) for a,s in zip(arrivals,software))
    start=max(int(v[0]) for v in arrivals)+250_000_000
    query=start+np.arange(seconds*2000,dtype=np.int64)*500_000
    if min(v[-1] for v in arrivals)<query[-1]:raise ValueError('capture too short; trace is never looped or extended')
    age=np.empty((len(query),31),dtype=np.int32)
    for i,(arrival,sw) in enumerate(zip(arrivals,software)):
        idx=np.searchsorted(arrival,query,side='right')-1
        sample=sw[idx]+offset-round(sample_margin_ms*1e6)
        sample_tick=np.floor_divide(sample-start,500_000)
        if np.any(np.diff(sample_tick)<0):raise ValueError('timestamp conversion not monotonic; clock assumption invalid')
        age[:,i]=np.arange(len(query))-sample_tick
    if age.min()<0 or age.max()>60:raise ValueError('future/stale trace; 30ms guard not relaxed')
    output.parent.mkdir(parents=True,exist_ok=True)
    if output.exists():raise FileExistsError(output)
    np.savez_compressed(output,age_ticks=age,motor_names=np.array([v[1] for v in inventory]),
        dt=np.array(.0005),source_sha256=np.array(hashlib.sha256((capture/'can.csv').read_bytes()).hexdigest()))
    report=dict(frames=len(query),seconds=seconds,dt=.0005,additional_sample_age_margin_ms=sample_margin_ms,
        age_ms={'mean':float(age.mean()*.5),'p99':float(np.percentile(age,99)*.5),'max':float(age.max()*.5)},
        source_sha256=hashlib.sha256((capture/'can.csv').read_bytes()).hexdigest(),
        trace_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),hardware_tx=0,hardware_qualified=False,
        limitations=['Estimated clock conversion from minimum receive-minus-software timestamp offset; not exact captured clock calibration.',
          'Additional sample-age margin is a sensitivity assumption; internal motor sampling time was not measured.',
          'Receive schedule is from disabled zero-gain 500Hz traffic; enabled repeated commands can differ.'])
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=int,default=50);p.add_argument('--sample-margin-ms',type=float,default=1.)
    a=p.parse_args();prepare(a.capture,a.output,a.seconds,a.sample_margin_ms)
