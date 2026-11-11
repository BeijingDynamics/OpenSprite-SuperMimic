"""Rate-only transport profile conversion; never opens hardware."""
import csv
from pathlib import Path


def apply_all_joint_500hz(path):
    path = Path(path)
    lines = path.read_text().splitlines()
    column = lines[0].split('\t').index('poll_rate_hz')
    assert len(lines) == 32, 'expected 31 named motor rows'
    output = [lines[0]]
    for line in lines[1:]:
        fields = line.split('\t')
        assert fields[column] in ('50', '500'), 'unexpected original transport rate'
        fields[column] = '500'
        output.append('\t'.join(fields))
    path.write_text('\n'.join(output)+'\n')


def require_rate_only_change(original, candidate):
    def read(path):
        with Path(path).open() as stream:
            return list(csv.DictReader(stream, delimiter='\t'))
    before, after = read(original), read(candidate)
    if len(before) != 31 or len(after) != 31:
        raise ValueError('motor profile must contain 31 entries')
    for old, new in zip(before, after):
        if old.pop('poll_rate_hz') not in ('50', '500') or new.pop('poll_rate_hz') != '500' or old != new:
            raise ValueError('500 Hz profile changed a non-rate field or endpoint ordering')
