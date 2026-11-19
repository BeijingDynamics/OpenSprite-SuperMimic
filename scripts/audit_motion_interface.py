"""Separate reference-tracking ability from velocity-conditioned gait selection."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import traceback
import numpy as np
from audit_sole_tracking import physical_trace

ROOT=Path(__file__).resolve().parents[1]

def audit(evaluation):
    evaluation=Path(evaluation);output=evaluation/'motion-interface-audit'
    report=evaluation/'motion-interface-diagnosis.json'
    if report.exists():return
    result={'hardware_access':False,'motion_reference_changed':False}
    try:
        suffix=evaluation.name.removeprefix('eval-');iteration=int(suffix.rsplit('-',1)[1])
        checkpoint=evaluation.parent/('train-'+suffix)/f'model_{iteration:06d}.pt'
        expected=json.loads((evaluation/'summary.json').read_text())['checkpoint_sha256']
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest()!=expected:raise ValueError('Velocity/motion checkpoint mismatch')
        motion=ROOT/'outputs/seed45-lr-continuation/reference/pm01_clearance_v1.npz'
        with (evaluation/'motion-interface-audit.log').open('x') as log:
            subprocess.run([sys.executable,str(ROOT/'scripts/evaluate_joint.py'),'--checkpoint',str(checkpoint),
                '--motion',str(motion),'--interface','motion','--output',str(output),'--seed','101'],cwd=ROOT,
                stdout=log,stderr=subprocess.STDOUT,check=True,timeout=240)
        summary=json.loads((output/'summary.json').read_text());metrics=physical_trace(output/'rollout.npz',0,3.)
        if 'clearance_events' in metrics:
            metrics['event_peak_median_m']=[float(np.median([x['peak_m'] for x in foot])) if foot else None for foot in metrics['clearance_events']]
        result.update(status='completed',checkpoint_sha256=expected,reference_sha256=hashlib.sha256(motion.read_bytes()).hexdigest(),
            motion_summary=summary,physical_clearance=metrics,
            interpretation='Compare actual motion-interface clearance against the corrected reference and velocity-interface clearance. This separates interfaces, not causes such as decoder capacity versus world-model error.',
            limitation='Motion evaluation begins from the reference state and runs its 11.42-second clip; velocity evaluation begins at rest. Compare clearance after 3 seconds, not raw event counts across unequal durations.')
    except Exception:
        result.update(status='diagnostic-failed',error=traceback.format_exc())
    with report.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({'motion_interface_audit':result['status'],'output':str(report)}))

if __name__=='__main__':audit(sys.argv[1])
