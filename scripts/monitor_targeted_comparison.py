"""Job-bound read-only foot audits and improvement-retention screening."""
import datetime as dt
import json
from pathlib import Path
import subprocess
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'outputs/targeted-upright-clearance-20261118/evidence'
BASELINE=ROOT/'outputs/retrain-posture-width-20261117/evidence/eval-5000'
REMOTE='/root/gpufree-data/supermimic_posture_width_20261117'
JOB=REMOTE+'/outputs/targeted-upright-clearance-20261118'
HOST='root@120.209.70.195'

def retain(directory,anchor,parent):
    read=lambda folder,name:json.loads((folder/name).read_text())
    rows=lambda data:{r['case_name']:r for r in data['results']}
    p=rows(read(directory,'posture-width.json'));a=rows(read(anchor,'posture-width.json'))
    g=rows(read(directory,'locomotion-geometry.json'));ag=rows(read(anchor,'locomotion-geometry.json'));pg=rows(read(parent,'locomotion-geometry.json'))
    checks={};observed={}
    checks['standing_lean_retained']=abs(p.get('zero',{}).get('pitch_median_deg',999))<=abs(a['zero']['pitch_median_deg'])+.3
    for name,ref in a.items():
        r=p.get(name,{})
        checks[name+'/survival']=r.get('survived') is True
        checks[name+'/spacing_retained']=r.get('center_p05_m',0)>=ref['center_p05_m']-.005
        checks[name+'/no_crossing']=r.get('crossed_fraction',1)==0
        observed[name]={k:r.get(k) for k in ['pitch_median_deg','center_p05_m','mean_vx_m_s','mean_yaw_rad_s']}
        if name=='zero':continue
        peaks=g.get(name,{}).get('geometry',{}).get('event_peak_median_m',[])
        observed[name]['swing_peak_median_m']=peaks
        target=(ag if name in ['forward025','left_curve'] else pg)[name]['geometry']['event_peak_median_m']
        valid=len(peaks)==2 and all(x is not None and x>0 for x in peaks)
        checks[name+'/bilateral_clearance']=valid and all(v>=.95*t for v,t in zip(peaks,target))
        counts=g.get(name,{}).get('geometry',{}).get('clearance_event_counts',[0,0])
        target_counts=pg[name]['geometry']['clearance_event_counts']
        checks[name+'/steps_retained']=len(counts)==2 and all(v>=.85*t for v,t in zip(counts,target_counts))
    quality=read(directory,'gait-quality-selection.json')
    return {'checkpoint_sha256':quality['checkpoint_sha256'],'preservation_checks':checks,
            'preservation_passed':all(checks.values()),'original_quality_passed':quality['accepted_for_multiseed_validation'],
            'candidate_for_further_validation':all(checks.values()) and quality['accepted_for_multiseed_validation'],
            'failed_preservation_checks':[k for k,v in checks.items() if not v],'observed':observed,
            'scope':'Additional selection screen, not hardware admission or a training change.',
            'tolerances':'Standing pitch no more than 0.3 degree worse than clearance600; P05 spacing within 5 mm; retain 95% of clearance600 forward/left-turn peaks and parent5000 low-speed/right-turn peaks; 85% of parent step counts.'}

def audit_remote(label):
    destination=OUT/(label+'-foot-reward-audit.json')
    if destination.exists():return
    # Labels come only from known experiment state, not arbitrary shell text.
    if not all(c.isalnum() or c=='-' for c in label):raise ValueError('Unexpected experiment label')
    remote=JOB+'/'+label+'-foot-reward-audit.json'
    command=f'test -f {remote} || (cd {REMOTE} && .venv/bin/python scripts/audit_velocity_clearance.py {JOB}/{label} --output {remote} > {JOB}/{label}-foot-reward-audit.log 2>&1)'
    subprocess.run(['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','-p','30179',HOST,command],check=True,timeout=180)
    subprocess.run(['scp','-o','BatchMode=yes','-o','ConnectTimeout=10','-P','30179',HOST+':'+remote,str(destination)],check=True,timeout=60)

def main():
    anchor=OUT/'eval-upright-clearance-600'
    status=OUT/'diagnostic-monitor-status.json';errors=[]
    deadline=dt.datetime(2026,11,19,14,55,tzinfo=dt.timezone(dt.timedelta(hours=8))).timestamp()
    while time.time()<deadline:
        state={};results={};ok=False
        try:
            state=json.loads((OUT/'remote-status.json').read_text())
            saved=json.loads((OUT/'backup-status.json').read_text())['saved']
            for label in state['evaluations']:
                if label not in saved:continue
                directory=OUT/label
                result=retain(directory,anchor,BASELINE)
                (OUT/(label+'-retention-screen.json')).write_text(json.dumps(result,indent=2)+'\n')
                audit_remote(label);results[label]=result
            (OUT/'progress-comparison.json').write_text(json.dumps(results,indent=2)+'\n')
            ok=len(results)==len(state['evaluations'])
        except Exception:errors.append(traceback.format_exc())
        status.write_text(json.dumps({'updated_unix':time.time(),'audited':list(results),'errors':errors[-5:]},indent=2)+'\n')
        if ok and state.get('phase') in ['failed','complete-awaiting-result-review']:return
        time.sleep(45)

if __name__=='__main__':main()
