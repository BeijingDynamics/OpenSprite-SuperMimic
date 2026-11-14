"""Compare perturbed motion with a same-command nominal case, preserving failures.

This supplements instantaneous recovery metrics; it does not replace them or
qualify hardware. Same-command environments need not have identical initial
states. A one-second trailing mean reduces ordinary stride oscillation without
discarding signed velocity bias, heading error, displacement, or falls.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def trailing_mean(values, window):
    result=np.full(values.shape,np.nan,dtype=float)
    if len(values)<window:return result
    accumulated=np.concatenate([np.zeros_like(values[:1]),np.cumsum(values,axis=0)],axis=0)
    result[window-1:]=(accumulated[window:]-accumulated[:-window])/window
    return result


def yaw(quaternion):
    w,x,y,z=quaternion.T
    return np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def summarize(directory):
    source=json.loads((directory/'summary.json').read_text())
    with np.load(directory/'rollout.npz',allow_pickle=False) as archive:
        trace={key:archive[key] for key in archive.files}
    fps=float(trace['fps']);window=round(fps)
    nominal={tuple(row['label']):row for row in source['results'] if not row.get('forces') and not row.get('stages')}
    rows=[]
    for case in source['results']:
        if not case.get('forces'):continue
        baseline=nominal.get(tuple(case['label']))
        if baseline is None:continue
        index=case['env'];reference=baseline['env']
        failures=trace['failed'][:,index].astype(bool)
        baseline_failures=trace['failed'][:,reference].astype(bool)
        first=np.flatnonzero(failures|baseline_failures)
        end=int(first[0]) if len(first) else len(failures)
        mean_velocity=trailing_mean(trace['velocity'][:end,index,:2],window)
        reference_velocity=trailing_mean(trace['velocity'][:end,reference,:2],window)
        mean_yaw=trailing_mean(trace['yaw_rate'][:end,index],window)
        reference_yaw=trailing_mean(trace['yaw_rate'][:end,reference],window)
        difference=np.linalg.norm(mean_velocity-reference_velocity,axis=-1)
        yaw_difference=abs(mean_yaw-reference_yaw)
        quaternion=trace['qpos'][:end,index,3:7]
        tilt=np.rad2deg(np.arccos(np.clip(1-2*(quaternion[:,1]**2+quaternion[:,2]**2),-1,1)))
        comparable=(difference<.05)&(yaw_difference<.10)&(tilt<12)
        events=[]
        for force in case['forces']:
            release=round((force['at_seconds']+force['duration_seconds'])*fps)
            recovered=None
            # A full post-release averaging window must precede the hold window.
            for start in range(release+window-1,end-window+1):
                if comparable[start:start+window].all():
                    recovered=(start+window-1-release)/fps;break
            events.append(dict(force,matched_recovery_confirmed_seconds=recovered))
        tail=max(0,end-round(5*fps))
        row=dict(case_name=case['case_name'],nominal_case=baseline['case_name'],
                 survived=case['survived'],nominal_survived=baseline['survived'],
                 first_failure_seconds=case['first_failure_seconds'],events=events)
        if end>window:
            heading=yaw(quaternion);reference_heading=yaw(trace['qpos'][:end,reference,3:7])
            heading_delta=(heading[-1]-heading[0])-(reference_heading[-1]-reference_heading[0])
            displacement=trace['qpos'][end-1,index,:2]-trace['qpos'][0,index,:2]
            nominal_displacement=trace['qpos'][end-1,reference,:2]-trace['qpos'][0,reference,:2]
            row.update(last5s_velocity_m_s=trace['velocity'][tail:end,index,:2].mean(0).tolist(),
                       last5s_signed_command_error_m_s=(trace['velocity'][tail:end,index,:2]-trace['commands'][tail:end,index,:2]).mean(0).tolist(),
                       last5s_nominal_velocity_difference_m_s=(trace['velocity'][tail:end,index,:2]-trace['velocity'][tail:end,reference,:2]).mean(0).tolist(),
                       last5s_mean_yaw_rad_s=float(trace['yaw_rate'][tail:end,index].mean()),
                       pelvis_tilt_p95_deg=float(np.quantile(tilt,.95)),
                       accumulated_heading_difference_deg=float(np.rad2deg(heading_delta)),
                       world_displacement_difference_m=(displacement-nominal_displacement).tolist())
        rows.append(row)
    return dict(iteration=source['iteration'],seed=source['seed'],results=rows,
                criterion='Supplement only: one-second trailing mean differs from same-command nominal by <0.05 m/s and <0.10 rad/s, raw tilt <12 deg, sustained for one second after a full post-release averaging window; truncate at either first failure.',
                limits='Command-matched, not identical initial states. Retain original recovery_quality.json, raw trajectories, signed command error and accumulated heading/displacement; no hardware protection changes.',
                hardware_qualified=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--evaluation',type=Path,required=True)
    args=parser.parse_args()
    report=summarize(args.evaluation)
    (args.evaluation/'command_matched_recovery.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(iteration=report['iteration'],seed=report['seed'],cases=len(report['results']))))
