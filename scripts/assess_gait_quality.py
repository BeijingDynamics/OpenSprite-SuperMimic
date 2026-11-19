"""Multi-metric candidate selection, separate from permission to keep researching."""
import argparse
import json
import math
from pathlib import Path

CASES=('zero','forward025','forward015','left_curve','right_curve')
WALKING=CASES[1:]

def assess(base_pose,base_geometry,pose,geometry):
    b={x['case_name']:x for x in base_pose['results']}
    c={x['case_name']:x for x in pose['results']}
    bg={x['case_name']:x.get('geometry',{}) for x in base_geometry['results']}
    cg={x['case_name']:x.get('geometry',{}) for x in geometry['results']}
    checks={};observed={}
    def finite(x):return isinstance(x,(int,float)) and math.isfinite(x)
    def check(name,condition):checks[name]=bool(condition)
    check('checkpoint_identity',pose['checkpoint_sha256']==geometry['checkpoint_sha256'])
    for name in CASES:
        check(name+'/survival',name in c and c[name].get('survived') is True)
        if name not in c or name not in b:continue
        r=c[name];p=r.get('pitch_median_deg');p95=r.get('pitch_abs_p95_deg')
        check(name+'/upright_median',finite(p) and abs(p)<=3.0)
        check(name+'/upright_p95',finite(p95) and p95<=max(5.0,b[name].get('pitch_abs_p95_deg',0)+1.0))
        observed[name]={k:r.get(k) for k in ['pitch_median_deg','mean_vx_m_s','mean_yaw_rad_s','center_p05_m']}
        if name not in WALKING:continue
        center=r.get('center_p05_m');gap=r.get('edge_gap_p05_m')
        check(name+'/center_margin',finite(center) and center>=.10)
        check(name+'/edge_margin',finite(gap) and gap>=.02)
        check(name+'/no_crossing',r.get('crossed_fraction',1)==0)
        peaks=cg.get(name,{}).get('event_peak_median_m',[])
        base_peaks=bg.get(name,{}).get('event_peak_median_m',[])
        counts=cg.get(name,{}).get('clearance_event_counts',[])
        base_counts=bg.get(name,{}).get('clearance_event_counts',[])
        valid=len(peaks)==len(base_peaks)==len(counts)==len(base_counts)==2 and all(finite(x) and x>0 for x in peaks+base_peaks)
        check(name+'/bilateral_swings_present',valid)
        if valid:
            for j,side in enumerate(['left','right']):
                check(name+'/'+side+'_clearance_retained',peaks[j]>=.95*base_peaks[j])
                check(name+'/'+side+'_step_count_retained',counts[j]>=.85*base_counts[j])
            check(name+'/swing_height_symmetry',max(peaks)/min(peaks)<=max(1.5,max(base_peaks)/min(base_peaks)))
            observed[name]['swing_peak_median_m']=peaks
    for name,target in [('forward025',.25),('forward015',.15)]:
        v=c.get(name,{}).get('mean_vx_m_s');allowed=max(.04,abs(b[name]['mean_vx_m_s']-target)+.01)
        check(name+'/speed_tracking_retained',finite(v) and abs(v-target)<=allowed)
    for name,sign in [('left_curve',1),('right_curve',-1)]:
        yaw=c.get(name,{}).get('mean_yaw_rad_s');err=c.get(name,{}).get('yaw_error_rad_s')
        check(name+'/turn_direction_and_response',finite(yaw) and sign*yaw>=.90*abs(b[name]['mean_yaw_rad_s']))
        check(name+'/turn_tracking_retained',finite(err) and err<=b[name]['yaw_error_rad_s']+.025)
    return dict(revision='posture_width_clearance_turn_selection_v1',checkpoint_sha256=pose['checkpoint_sha256'],
        accepted_for_multiseed_validation=all(checks.values()),checks=checks,
        failed_checks=[k for k,v in checks.items() if not v],observed=observed,
        scope='Nominal 20-second screening only; no hardware qualification or claim of disturbance robustness.',
        tolerances='3-degree median upright target, 95% baseline bilateral swing peaks, 85% swing counts, at most 1.5 height ratio, 90% baseline signed turn response. Physical robot protections are untouched.')

def assess_directory(directory,baseline=None):
    directory=Path(directory);baseline=Path(baseline) if baseline else directory.parent/'baseline'
    read=lambda p:json.loads(p.read_text())
    result=assess(read(baseline/'posture-width.json'),read(baseline/'locomotion-geometry.json'),
                  read(directory/'posture-width.json'),read(directory/'locomotion-geometry.json'))
    (directory/'gait-quality-selection.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path);parser.add_argument('--baseline',type=Path)
    args=parser.parse_args();result=assess_directory(args.directory,args.baseline)
    print(json.dumps(result,indent=2))
