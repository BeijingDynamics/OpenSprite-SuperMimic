"""Inspect existing transport traces without opening hardware."""
import csv
import json
from pathlib import Path
from collections import defaultdict
import sys

p=Path(sys.argv[1])
groups=defaultdict(list)
with (p/'can.json.tx.csv').open() as f:
    for row in csv.DictReader(f):
        r={k:(v if k=='interface' else int(v)) for k,v in row.items()}
        if r['group']:groups[r['sequence']].append(r)
worst=sorted(groups,key=lambda s:max(r['call_begin_ns']-r['accept_ns'] for r in groups[s]),reverse=True)[:5]
for seq in worst:
    rows=groups[seq];a=rows[0]['accept_ns']
    print(json.dumps(dict(sequence=seq, first_ms=(rows[0]['call_begin_ns']-a)/1e6,
        last_ms=(rows[-1]['call_begin_ns']-a)/1e6,
        sends=[(r['interface'],r['motor_id'],round((r['call_begin_ns']-a)/1e6,4),round((r['call_end_ns']-r['call_begin_ns'])/1e6,4)) for r in rows])))
