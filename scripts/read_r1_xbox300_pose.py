"""Read a bounded zero-gain pose and compare it with unchanged actor joint limits."""
import csv
import fcntl
import json
from pathlib import Path
import subprocess
import numpy as np

stage = Path('/home/tony/sprite-sm-walk-r1-xbox300-20261119')
source = stage/'interrupt-01'
out = stage/'pose-after-reboot-01'
out.mkdir()
with (source/'motors.tsv').open() as stream:
    motors = list(csv.DictReader(stream, delimiter='\t'))
inventory = out/'inventory.tsv'
inventory.write_text(''.join('\t'.join(row[k] for k in ['interface','motor_name','can_id','master_id'])+'\n' for row in motors))
with Path('/home/tony/supermimic-run002-20261106/commissioning.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    subprocess.run([str(stage/'build/sprite_zero_gain_can_probe'), str(inventory), '2', '50',
                    str(out/'probe'), 'ZERO_GAIN_DISABLED_MOTORS_ONLY'], check=True, timeout=10)
last = {}
with (out/'probe.csv').open() as stream:
    for row in csv.DictReader(stream):
        last[int(row['motor_index'])] = bytes.fromhex(row['payload_hex'])
assert len(last) == 31
positions = []
status = []
for i, motor in enumerate(motors):
    payload = last[i]
    status.append(payload[0] >> 4)
    low, high = float(motor['position_min_rad']), float(motor['position_max_rad'])
    positions.append(((payload[1]<<8)|payload[2])*(high-low)/65535+low)
assert all(x == 0 for x in status), status
feedback = np.frombuffer((source/'feedback.bin').read_bytes(), dtype='<f8', offset=16)
q = feedback[31:].reshape(31,31) @ (np.array(positions)-feedback[:31])
limits = np.frombuffer((source/'target.bin').read_bytes(), dtype='<f8', offset=16, count=62).reshape(2,31)
names = json.loads((stage/'bundle/contract.json').read_text())['joint_names']
rows = [dict(joint=name, position_deg=float(np.degrees(q[i])),
             lower_deg=float(np.degrees(limits[0,i])), upper_deg=float(np.degrees(limits[1,i])),
             outside=bool(q[i]<limits[0,i] or q[i]>limits[1,i])) for i,name in enumerate(names)]
result = dict(motor_enable_attempts=0, final_disabled_count=31, joints=rows,
              outside=[row for row in rows if row['outside']])
(out/'report.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(dict(final_disabled_count=31, outside=result['outside']), indent=2))
