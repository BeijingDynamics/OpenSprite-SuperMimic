"""Physical teleoperation screen; low-speed walking is diagnostic, not a veto."""
import json
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import BODY_JOINT_NAMES

def assess(folder,baseline=None):
    folder=Path(folder)
    geom={r['case_name']:r for r in json.loads((folder/'locomotion-geometry.json').read_text())['results']}
    post={r['case_name']:r for r in json.loads((folder/'posture-width.json').read_text())['results']}
    checks={};rows={}
    with np.load(folder/'rollout.npz') as z:
        names=list(z['case_names']);k=[7+BODY_JOINT_NAMES.index(s+'_knee_joint') for s in ['left','right']]
        knees=np.abs(np.rad2deg(np.median(z['qpos'][150:,names.index('zero')][:,k],axis=0)))
    for name in ['zero','forward025','forward050','left_curve','right_curve','start_stop']:
        g,p=geom[name],post[name];h=g.get('geometry',{}).get('event_peak_median_m',[None,None])
        checks[name+'/survival']=bool(g['survived'])
        checks[name+'/spacing']=p['center_p05_m'] is not None and p['center_p05_m']>=.105 and p['crossed_fraction']==0
        rows[name]=dict(pitch=p['pitch_median_deg'],spacing=p['center_p05_m'],peaks=h,vx=g['mean_vx_m_s'],yaw=g['mean_yaw_rad_s'])
        if name not in ['zero','start_stop']:
            checks[name+'/bilateral_lift']=min(x or 0 for x in h)>=.010
            checks[name+'/lift_balance']=min(x or 0 for x in h)/max(max(x or 0 for x in h),1e-9)>=.60
        if name.startswith('forward'):
            checks[name+'/progress']=g['mean_vx_m_s'] is not None and g['mean_vx_m_s']>=(.15 if name=='forward025' else .25)
        if name.endswith('curve'):
            checks[name+'/turn']=g['mean_yaw_rad_s'] is not None and g['mean_yaw_rad_s']*(1 if name=='left_curve' else -1)>=.12
    checks['zero/upright']=post['zero']['pitch_median_deg'] is not None and abs(post['zero']['pitch_median_deg'])<=4
    checks['zero/knee_extension']=float(knees.max())<=30
    tail=geom['start_stop'].get('last5s_mean_velocity_m_s')
    checks['start_stop/stopped']=tail is not None and np.linalg.norm(tail[:2])<.05
    if baseline is not None:
        for name,row in rows.items():
            anchor=baseline['cases'][name]
            checks[name+'/spacing_retained']=row['spacing'] is not None and row['spacing']>=anchor['spacing']-.005
            if name not in ['zero','start_stop']:
                checks[name+'/clearance_retained']=all((h or 0)>=.85*(old or 0) for h,old in zip(row['peaks'],anchor['peaks']))
                checks[name+'/pitch_retained']=row['pitch'] is not None and abs(row['pitch'])<=abs(anchor['pitch'])+2
    checks={key:bool(value) for key,value in checks.items()}
    failed=[k for k,v in checks.items() if not v]
    return dict(checks=checks,failed_checks=failed,passed=not failed,median_knee_magnitudes_deg=knees.tolist(),
        cases=rows,rank=[len(failed),float(knees.max()),abs(post['zero']['pitch_median_deg']) if post['zero']['pitch_median_deg'] is not None else 90],
        continue_training=all(geom[n]['survived'] for n in ['zero','forward025','forward050']),
        hardware_qualified=False,relative_baseline_used=baseline is not None,
        scope='Single-seed nominal physical screen; deployment admission remains separate.')

if __name__=='__main__':
    p=Path(sys.argv[1]);r=assess(p);(p/'teleop-screen.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r))
