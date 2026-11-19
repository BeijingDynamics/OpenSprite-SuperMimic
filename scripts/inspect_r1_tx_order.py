"""Inspect per-bus physical timestamp reversals without device access."""
import csv
import json
import sys
from pathlib import Path
from collections import defaultdict
p=Path(sys.argv[1]);streams=defaultdict(list)
with (p/'can.json.tx.csv').open() as f:
    for row in csv.DictReader(f):
        r={k:(v if k=='interface' else int(v)) for k,v in row.items()}
        if r['sequence']>1:streams[r['interface']].append(r)
for name,rows in streams.items():
    ordered=sorted((r for r in rows if r['hardware_raw_ns']),key=lambda r:r['hardware_raw_ns'])
    reversals=[(a,b) for a,b in zip(ordered,ordered[1:]) if a['sequence']>b['sequence']]
    print(name,'reversals',len(reversals))
    for a,b in reversals[:5]:
        print(json.dumps(dict(new=a,old=b)))
        ids={a['packet_id'],b['packet_id']}
        for r in rows:
            if any(abs(r['packet_id']-i)<=2 for i in ids):print(json.dumps(r))
