"""Prepare a clearly labelled half-speed feasibility candidate; never train it."""
import json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from retarget.processing import robot_contract,kinematics,geometry_quality
from retarget.convert_umr import pack

source=ROOT/'data/motions/contact_v5_v11';output=ROOT/'data/motions/contact_v5_v11_slow2'
output.mkdir(exist_ok=True);model,_,_,_=robot_contract();paths=[]
manifest=json.loads((ROOT/'data/sources/lafan1-resolved/selected_segments.json').read_text())
for row in manifest:
    name=row['name'];path=source/(name+'.npz')
    with np.load(path,allow_pickle=False) as data:payload={k:data[k] for k in data.files}
    prior=payload['qpos'];old_time=np.arange(len(prior));sample=np.arange(2*len(prior)-1)/2
    poses=np.stack([np.interp(sample,old_time,prior[:,i]) for i in range(prior.shape[1])],-1)
    poses[:,3:7]=Slerp(old_time,Rotation.from_quat(prior[:,[4,5,6,3]]))(sample).as_quat()[:,[3,0,1,2]]
    payload.update(kinematics(model,poses),qpos=poses,time=np.arange(len(poses))/50,
                   postprocess=np.asarray(str(payload['postprocess'])+';time_stretch_2x'),playback_speed=np.asarray(.5),
                   motion_quality=np.asarray('numeric_and_visual_review_required_not_trained'))
    target=output/(name+'.npz');np.savez_compressed(target,**payload);paths.append(target)
    report=json.loads(path.with_suffix('.json').read_text())
    report.update(final_geometry=geometry_quality(model,poses),playback_speed=.5,training_started=False,
                  retiming_reason='Reduce physical velocity by2 and acceleration by4 before a separate feasibility test; not a claim of solved gait')
    target.with_suffix('.json').write_text(json.dumps(report,indent=2))
pack(paths,output/'v5_locomotion.npz')
print('HALF_SPEED_CANDIDATE_PREPARED_NOT_TRAINED',output,flush=True)
