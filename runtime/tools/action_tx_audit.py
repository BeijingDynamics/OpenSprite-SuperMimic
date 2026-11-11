"""Read-only audit of per-action CAN TX evidence (no device access).

SW timestamps mean driver handoff, not motor execution. Hardware raw clocks are
reported separately: this adapter has no PHC, so cross-channel clock alignment
must not be invented. Host clock conversion is bounded by the recorded syscall.
"""
import csv
import json
from collections import defaultdict
from pathlib import Path


def audit(folder):
    folder=Path(folder)
    can=json.loads((folder/'can.json').read_text())
    assert can['coherent_action_send'], 'coherent sending was not enabled'
    with (folder/'motors.tsv').open() as f: motors=list(csv.DictReader(f,delimiter='\t'))
    names={(m['interface'],int(m['can_id'])):m['motor_name'] for m in motors}
    assert len(names)==31
    for motor in can['motors'].values():
        assert 0 < motor['cadenced_monitor_samples'] <= motor['nominal_sends']
        assert motor['cadenced_monitor_samples'] >= .95*motor['nominal_sends'], 'nominal monitor cadence lost'
    with (folder/'can.json.timing.csv').open() as f:
        timing={int(r['sequence']):r for r in csv.DictReader(f)}
    groups=defaultdict(list)
    sockets={}
    with (folder/'can.json.tx.csv').open() as f:
        for raw in csv.DictReader(f):
            r={k:v if k=='interface' else int(v) for k,v in raw.items()}
            assert (r['interface'],r['motor_id']) in names
            state=sockets.setdefault(r['interface'],dict(packet=0,seq=0,previous_hw=0,current_hw=0))
            assert r['packet_id']==state['packet']
            state['packet']+=1
            if r['sequence']>1:
                assert r['sequence']>=state['seq'], 'old action sent after new one'
                if r['sequence']>state['seq']:
                    state['previous_hw']=state['current_hw']
                    state['seq']=r['sequence']
                if r['hardware_raw_ns']:
                    assert r['hardware_raw_ns']>=state['previous_hw'], 'hardware sent old action after new one'
                    state['current_hw']=max(state['current_hw'],r['hardware_raw_ns'])
            if r['group']: groups[r['sequence']].append(r)
    assert len(groups)==can['coherent_action_groups']
    expected={s for s,t in timing.items() if s>1 and int(t['accept_ns'])}
    assert set(groups)==expected, 'accepted action lacks whole-group dispatch'
    output=[]
    spans=[]
    hardware_spans=[]
    for seq,group in sorted(groups.items()):
        assert len(group)==31 and {(r['interface'],r['motor_id']) for r in group}==set(names)
        hardware=defaultdict(list)
        low=[];high=[]
        for r in group:
            assert r['software_realtime_ns']>0, 'missing driver TX timestamp'
            assert r['hardware_raw_ns']>0, 'missing adapter TX timestamp'
            assert r['hardware_received_ns']>=r['call_begin_ns'], 'invalid hardware completion receipt'
            assert r['powered']==int(can['motor_enable_attempts']>0)
            t=timing[seq]
            assert r['feedback_ns']==int(t['feedback_ns']) and r['accept_ns']==int(t['accept_ns'])
            delta=r['software_realtime_ns']-r['realtime_before_ns']
            lower=r['call_begin_ns']+delta
            upper=r['call_end_ns']+delta
            assert upper>=lower and lower>=r['accept_ns']
            assert upper-r['feedback_ns']<20_000_000, 'oldest-feedback to driver TX exceeds original 20 ms'
            assert r['hardware_received_ns']-r['feedback_ns']<20_000_000, 'hardware TX completion not confirmed within 20 ms'
            assert r['call_begin_ns']-r['accept_ns']<2_500_000, 'action waited beyond next mapping boundary'
            low.append(lower);high.append(upper)
            hardware[r['interface']].append(r['hardware_raw_ns'])
            output.append(dict(sequence=seq,motor=names[r['interface'],r['motor_id']],
                actor_target_to_driver_upper_ms=(upper-int(t['actor_target_ns']))/1e6,
                accept_to_driver_upper_ms=(upper-r['accept_ns'])/1e6,
                oldest_feedback_to_driver_upper_ms=(upper-r['feedback_ns'])/1e6,
                driver_mono_lower_ns=lower,driver_mono_upper_ns=upper,
                physical_tx_delay_lower_ms=(r['call_begin_ns']-int(t['actor_target_ns']))/1e6,
                physical_tx_delay_upper_ms=(r['hardware_received_ns']-int(t['actor_target_ns']))/1e6,
                physical_tx_mono_lower_ns=r['call_begin_ns'],physical_tx_mono_upper_ns=r['hardware_received_ns'],
                hardware_raw_ns=r['hardware_raw_ns']))
        spans.append((max(high)-min(low))/1e6)
        hardware_spans.extend((max(v)-min(v))/1e6 for v in hardware.values())
    assert groups, 'no formal action TX evidence'
    assert max(spans)<2., 'whole-group driver handoff spans more than 2 ms'
    assert max(hardware_spans)<2., 'one-bus hardware group spans more than 2 ms'
    with (folder/'action-motor-latency.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(output[0]));writer.writeheader();writer.writerows(output)
    summary=dict(passed=True,groups=len(groups),motor_dispatches=len(output),
        whole_group_driver_span_max_ms=max(spans),per_bus_hardware_span_max_ms=max(hardware_spans),
        actor_target_to_driver_upper_max_ms=max(r['actor_target_to_driver_upper_ms'] for r in output),
        accept_to_driver_upper_max_ms=max(r['accept_to_driver_upper_ms'] for r in output),
        oldest_feedback_to_driver_upper_max_ms=max(r['oldest_feedback_to_driver_upper_ms'] for r in output),
        physical_tx_delay_upper_max_ms=max(r['physical_tx_delay_upper_ms'] for r in output),
        final_disabled_count=sum(m['final_disabled'] for m in can['motors'].values()),
        limitations=['Driver TX is not motor execution acknowledgement.',
                    'Physical TX host-time bounds use send-call entry and hardware completion receipt; no cross-clock subtraction.',
                    'Raw hardware clock is not assumed synchronized between adapters or with the host.',
                    'Disabled validation measures zero-gain frame transport, not powered motion.'])
    assert summary['final_disabled_count']==31
    if can.get('all_joint_500hz', False):
        import sys
        sys.path.insert(0,str(Path(__file__).resolve().parent))
        from full_rate_tx_audit import audit_full_rate
        summary['full_rate'] = audit_full_rate(folder,can,names)
    (folder/'action-tx-audit.json').write_text(json.dumps(summary,indent=2)+'\n')
    return summary


if __name__=='__main__':
    import sys
    print(json.dumps(audit(sys.argv[1]),indent=2))
