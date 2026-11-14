"""Build an inspectable, forward-priority ledger from completed physical audits."""
import argparse
import json
from pathlib import Path


def read(path):
    return json.loads(path.read_text()) if path.exists() else None


def collect(audit,long_path):
    seeds=[]
    for seed in [101,202,303]:
        summary=read(audit/f'velocity-seed{seed}/summary.json')
        geometry=read(audit/f'locomotion-geometry-seed{seed}.json')
        if summary is None:continue
        rows={item['case_name']:item for item in summary['results']}
        geometries={item['case_name']:item for item in geometry['results']} if geometry else {}
        priority=[row for name,row in rows.items() if name not in ['reverse','lateral_left','lateral_right']]
        forward=rows.get('clean_0.25',{})
        feet=geometries.get('clean_0.25',{}).get('geometry',{})
        matched=read(audit/f'velocity-seed{seed}/command_matched_recovery.json')
        original=read(audit/f'velocity-seed{seed}/recovery_quality.json')
        events=[event for row in matched['results'] for event in row['events']] if matched else []
        original_events=[event for row in original['results'] for event in row['force_events']] if original else []
        seeds.append(dict(seed=seed,priority_survivors=sum(row['survived'] for row in priority),priority_cases=len(priority),
                          forward_vx_m_s=forward.get('mean_vx_m_s'),
                          forward_foot_event_peak_median_m=feet.get('event_peak_median_m'),
                          forward_loaded_foot_heading_deg=feet.get('loaded_foot_heading_relative_pelvis_median_deg'),
                          forward_torso_heading_deg=feet.get('torso_heading_relative_pelvis_median_deg'),
                          left_yaw_rad_s=rows.get('left',{}).get('mean_yaw_rad_s'),
                          right_yaw_rad_s=rows.get('right',{}).get('mean_yaw_rad_s'),
                          original_recovered_events=sum(event['recovered_for_1s'] for event in original_events) if original_events else None,
                          matched_recovered_events=sum(event['matched_recovery_confirmed_seconds'] is not None for event in events) if events else None,
                          disturbance_events=len(events) if events else None))
    long=read(long_path)
    long_rows={item['case_name']:item for item in long['results']} if long else {}
    return dict(seeds=seeds,long_command_results=long_rows,
                review_complete=len(seeds)==3 and all((audit/f'locomotion-geometry-seed{seed}.json').exists() for seed in [101,202,303]) and long is not None)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--comparison',type=Path,required=True)
    parser.add_argument('--parent-audit',type=Path,required=True);args=parser.parse_args()
    rows=[dict(candidate='parent-7629',**collect(args.parent_audit,args.comparison/'parent-7629-long-command/summary.json'))]
    for arm in ['control','actor-quarter']:
        for audit in sorted((args.comparison/arm/'training').glob('recovery-audit-*')):
            rows.append(dict(candidate=arm+'-'+audit.name.split('-')[-1],**collect(audit,audit/'long-command-seed101/summary.json')))
    report=dict(candidates=rows,selected_baseline='parent-7629',
                automatic_promotion=False,hardware_qualified=False,
                scope='Forward, turns, stops and perturbations. Reverse/lateral scores are retained in source evidence but excluded from priority survival counts. No score alone establishes gait quality.')
    (args.comparison/'comparison-ledger.json').write_text(json.dumps(report,indent=2)+'\n')
    lines=['# PM01 actor learning-rate comparison','',
           'The retained research baseline is parent 7629. This ledger does not automatically promote candidates or qualify hardware. Missing values mean that an audit is pending, not that the result is zero.','',
           '| Candidate | Seed | Priority survival | Forward vx (m/s) | Foot event peaks L/R (mm) | Turn yaw L/R (rad/s) | 60 s zero speed error (m/s) |',
           '| --- | ---: | --- | ---: | --- | --- | ---: |']
    def number(value):return 'pending' if value is None else f'{value:.4f}'
    for candidate in rows:
        zero=candidate['long_command_results'].get('zero60',{}).get('velocity_error_m_s')
        for seed in candidate['seeds']:
            feet=seed['forward_foot_event_peak_median_m']
            peaks='pending' if feet is None else '/'.join('none' if value is None else f'{value*1000:.1f}' for value in feet)
            turns=number(seed['left_yaw_rad_s'])+'/'+number(seed['right_yaw_rad_s'])
            lines.append(f"| {candidate['candidate']} | {seed['seed']} | {seed['priority_survivors']}/{seed['priority_cases']} | {number(seed['forward_vx_m_s'])} | {peaks} | {turns} | {number(zero)} |")
    lines+=['','Use the JSON ledger and original rollouts for individual perturbations, signed drift, accumulated heading, stops and failure timing. Lower learning rate is a hypothesis, not an assumed improvement.','']
    (args.comparison/'COMPARISON.md').write_text('\n'.join(lines))
    print(json.dumps(dict(candidates=len(rows),completed=sum(row['review_complete'] for row in rows),selected_baseline='parent-7629')))


if __name__=='__main__':main()
