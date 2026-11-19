"""Compare a labeled external friction prior with measured simulation torque."""
import argparse
import json
from pathlib import Path
import numpy as np


def stats(x):
    return dict(mean=float(np.mean(x)),p50=float(np.median(x)),p95=float(np.quantile(x,.95)),maximum=float(np.max(x)))


def summarize(directory, hardware):
    contract=json.loads((directory/'torque-contract.json').read_text())
    summary=json.loads((directory/'summary.json').read_text())
    with np.load(directory/'torque-500hz.npz') as data:
        a={key:data[key] for key in data.files}
    with np.load(directory/'rollout.npz') as data:
        forces=data['contact_force_mean_n'];failed=data['failed']
    names=contract['joint_names']
    selected={row['policy_joint'] for row in hardware['motor_map'].values()
              if row.get('policy_joint') and row['model']=='DM-J4340P-2EC V1.1 (48V)'}
    rows=[]
    t=np.arange(len(a['qd']))*.002
    for case in summary['results']:
        e=case['env'];alive=np.repeat(~failed[:,e],10)
        base=(t>=3)&alive
        for j,name in enumerate(names):
            if name not in selected:continue
            side=0 if name.startswith('left_') else 1 if name.startswith('right_') else None
            phases={'all':np.ones(len(t),dtype=bool)}
            if side is not None and any(s in name for s in ['hip_','knee_']):
                for label,condition in [('unloaded',(forces[:,e,side]<5)&(forces[:,e,1-side]>5)),('loaded',forces[:,e,side]>5)]:
                    stable=condition.copy();stable[1:]&=condition[:-1];stable[:-1]&=condition[1:]
                    stable[[0,-1]]=False
                    phases[label]=np.repeat(stable,10)
            for phase,phase_mask in phases.items():
                mask=base&phase_mask
                if not mask.any():continue
                velocity=a['qd'][mask,e,j];torque=a['applied'][mask,e,j]
                speed=abs(velocity);magnitude=abs(torque);moving=speed>=.05
                # Dynamic prior is evaluated only away from zero-speed stiction.
                fc=np.where(velocity>=0,.3330806719612709,.3453774421308448)
                friction=fc+.18397297113085181*speed
                control=contract['controls'][name]
                row=dict(case=case['case_name'],joint=name,phase=phase,samples=int(mask.sum()),
                    moving_fraction=float(moving.mean()),kp=control['kp'],kd=control['kd'],cap_nm=control['command_effort_nm'],
                    torque_abs_nm=stats(magnitude),velocity_abs_rad_s=stats(speed),
                    p_term_abs_nm=stats(abs(a['p_term'][mask,e,j])),d_term_abs_nm=stats(abs(a['d_term'][mask,e,j])),
                    target_error_abs_deg=stats(np.rad2deg(abs(a['target'][mask,e,j]-a['q'][mask,e,j]))),
                    saturation_fraction=float((abs(a['raw'][mask,e,j])>control['command_effort_nm']).mean()),
                    equivalent_coulomb_position_error_deg=float(np.rad2deg(.339229057/control['kp'])))
                if moving.any():
                    fm=friction[moving];tm=magnitude[moving];vm=speed[moving]
                    row.update(moving_torque_abs_nm=stats(tm),moving_friction_abs_nm=stats(fm),
                        moving_coulomb_to_abs_torque_sum=float(fc[moving].sum()/max(tm.sum(),1e-9)),
                        moving_total_friction_to_abs_torque_sum=float(fm.sum()/max(tm.sum(),1e-9)),
                        moving_friction_work_to_absolute_actuator_work=float((fm*vm).sum()/max((tm*vm).sum(),1e-9)),
                        moving_friction_exceeds_pd_fraction=float((fm>tm).mean()),
                        moving_nearzero_pd_fraction=float((tm<.2).mean()))
                rows.append(row)
    return dict(checkpoint_sha256=contract['checkpoint_sha256'],results=rows,
        definition='All statistics use pre-failure t>=3s. Unloaded/loaded phases use stable 50Hz foot-force tags (<5N with opposite foot >5N / >5N), expanded to 500Hz. Boundary frames excluded. Dynamic friction ratios use |qd|>=0.05rad/s and ratio of sums, not mean of near-zero denominators.',
        limitations='Counterfactual external J4340P friction on nominal trajectories; not a measured hardware loss or closed-loop causal effect. No J4310P/J3507/J6248P or parallel-joint extrapolation. Static friction at rest is load-dependent, not a permanently subtracted constant.')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--evaluation',type=Path,required=True);parser.add_argument('--hardware-map',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=summarize(args.evaluation,json.loads(args.hardware_map.read_text()))
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    for row in report['results']:
        if row['case']=='forward025' and row['phase']=='unloaded':
            print(row['joint'],'PD',round(row['moving_torque_abs_nm']['mean'],3),'friction',round(row['moving_friction_abs_nm']['mean'],3),'ratio',round(row['moving_total_friction_to_abs_torque_sum'],3),'speed',round(row['velocity_abs_rad_s']['mean'],3))
