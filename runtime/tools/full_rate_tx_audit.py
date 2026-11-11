"""Streaming audit of every 500 Hz control tick and actual RX/TX timestamps."""
import csv
import json
from pathlib import Path


def audit_full_rate(folder, can, names):
    folder = Path(folder)
    indexes = {key: index for index, key in enumerate(names)}
    complete_mask = (1 << 31) - 1
    ticks = {}
    last_tx = {}
    max_tx_gap = max_feedback_age = max_completion = 0.
    active_rows = 0
    completion_after_period_count = 0
    with (folder/'can.json.tx.csv').open() as stream:
        for raw in csv.DictReader(stream):
            tick = int(raw['control_tick'])
            if not tick:
                continue
            row = {key: value if key == 'interface' else int(value) for key, value in raw.items()}
            key = row['interface'], row['motor_id']
            assert key in indexes
            assert row['hardware_raw_ns'] > 0 and row['software_realtime_ns'] > 0
            assert row['hardware_received_ns'] >= row['call_begin_ns']
            scheduled = row['scheduled_ns']
            assert scheduled <= row['call_begin_ns'] <= row['call_end_ns'] < scheduled + 2_000_000
            # Completion notification arrival is an upper bound, not the wire
            # send time. Preserve the original per-new-action 20 ms audit.
            completion_after_period_count += row['hardware_received_ns'] >= scheduled + 2_000_000
            age = row['call_begin_ns'] - row['motor_feedback_ns']
            assert age >= 0
            if tick > 1:
                max_feedback_age = max(max_feedback_age, age/1e6)
            max_completion = max(max_completion, (row['hardware_received_ns']-scheduled)/1e6)
            state = ticks.setdefault(tick, dict(mask=0, sequence=row['sequence'], scheduled=scheduled,
                                                group=row['group'], first=row['call_begin_ns'], last=0))
            bit = 1 << indexes[key]
            assert not state['mask'] & bit, 'duplicate motor in control tick'
            assert (state['sequence'],state['scheduled'],state['group']) == (row['sequence'],scheduled,row['group']), 'mixed action in 500 Hz group'
            state['mask'] |= bit
            state['first'] = min(state['first'], row['call_begin_ns'])
            state['last'] = max(state['last'], row['call_end_ns'])
            if key in last_tx:
                previous_tick, previous_hw = last_tx[key]
                assert tick == previous_tick + 1, 'motor skipped a 500 Hz control tick'
                gap = row['hardware_raw_ns'] - previous_hw
                assert 0 < gap < 4_000_000, 'physical sends stalled or caught up out of order'
                max_tx_gap = max(max_tx_gap, gap/1e6)
            last_tx[key] = tick, row['hardware_raw_ns']
            active_rows += 1
    assert ticks and len(ticks) == can['full_rate_ticks']
    expected_ticks = int(round(can['duration_s']*500))
    if can['passed']:
        assert len(ticks) == expected_ticks
    assert sorted(ticks) == list(range(1,len(ticks)+1))
    epoch = ticks[1]['scheduled']
    for tick,state in ticks.items():
        assert state['mask'] == complete_mask, 'incomplete 31-motor tick'
        assert state['scheduled'] == epoch + (tick-1)*2_000_000
        assert state['last'] - state['first'] < 2_000_000
    assert active_rows == 31*len(ticks)
    end = epoch + len(ticks)*2_000_000
    rx = {key: dict(count=0,last=0,max_gap_ns=0) for key in names}
    with (folder/'can.json.rx.csv').open() as stream:
        for row in csv.DictReader(stream):
            key = row['interface'],int(row['motor_id'])
            stamp, dequeued = int(row['feedback_ns']), int(row['dequeue_ns'])
            assert key in rx and 0 < stamp <= dequeued
            if not epoch <= stamp < end:
                continue
            item = rx[key]
            if item['last']:
                assert stamp > item['last']
                item['max_gap_ns'] = max(item['max_gap_ns'],stamp-item['last'])
            item['last'] = stamp
            item['count'] += 1
    assert all(.95*len(ticks) <= item['count'] <= len(ticks)+2 for item in rx.values()), '500 Hz actual feedback coverage failed'
    result = dict(passed=True,control_hz=500,policy_hz=50,control_ticks=len(ticks),
                  motor_dispatches=active_rows,maximum_physical_tx_gap_ms=max_tx_gap,
                  maximum_feedback_age_at_send_ms=max_feedback_age,
                  maximum_tx_completion_from_boundary_ms=max_completion,
                  completion_notifications_after_period=completion_after_period_count,
                  actual_rx={names[key]: dict(count=item['count'],
                      rate_hz=item['count']/(len(ticks)*.002),max_gap_ms=item['max_gap_ns']/1e6)
                      for key,item in rx.items()})
    (folder/'full-rate-tx-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
