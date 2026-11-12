"""Explicit slower-playback reference augmentation; never auto-admit a bank."""
import argparse,hashlib,importlib.util,json,math,sys
from pathlib import Path
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation,Slerp

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT));sys.path.insert(0,str(ROOT/'scripts'))
from retarget.bvh_tools.motion_postprocess import process_motion
from retarget.processing import robot_contract,kinematics,geometry_quality


def retime(args):
    if not math.isfinite(args.speed) or not .25<=args.speed<=1:
        raise ValueError('Candidate speed must be between 0.25 and 1.0')
    original=json.loads(args.motion.with_suffix('.json').read_text())
    if original.get('numeric_candidate_passed') is not True:
        raise ValueError('Only a numerically qualified parent may be retimed')
    digest=original['refiner_sha256'];frozen=ROOT/'scripts'/('_frozen_neutral_'+digest[:8]+'.py')
    if hashlib.sha256(frozen.read_bytes()).hexdigest()!=digest:
        raise ValueError('Original refiner snapshot is missing or changed')
    spec=importlib.util.spec_from_file_location('frozen_neutral_'+digest[:8],frozen);refiner=importlib.util.module_from_spec(spec);spec.loader.exec_module(refiner)
    if hashlib.sha256(args.converted.read_bytes()).hexdigest()!=original['native_input_sha256']:
        raise ValueError('Converted native input differs from the parent audit')
    source=ROOT/'data/sources/lafan1-resolved'/(original['motion']+'.bvh')
    if hashlib.sha256(source.read_bytes()).hexdigest()!=original['source_sha256']:
        raise ValueError('Source crop changed')
    with np.load(args.motion,allow_pickle=False) as z:payload={k:z[k] for k in z.files}
    with np.load(args.converted,allow_pickle=False) as z:prior=z['qpos']
    targets,provenance,prior=refiner.source_targets_neutral(source,prior,args.native)
    model,joints,limits,speeds=robot_contract()
    result,times,resampling=process_motion(payload['qpos'],50*args.speed,50,model=model)
    source_time=times*args.speed;old_time=np.arange(len(prior))/50
    def linear(values):
        shape=np.asarray(values).shape;flat=np.asarray(values).reshape(len(old_time),-1)
        return np.stack([np.interp(source_time,old_time,flat[:,i]) for i in range(flat.shape[1])],-1).reshape((len(times),)+shape[1:])
    def rotations(values):
        return Slerp(old_time,Rotation.from_matrix(np.asarray(values)))(source_time).as_matrix()
    resized=[]
    for desired,yaw,contact in targets:
        resized.append((linear(desired),linear(yaw),linear(contact.astype(float))>=.5))
    upper=[]
    for row in provenance['source_upper_body_vectors']:
        direction=linear(row['directions']);direction/=np.linalg.norm(direction,axis=-1,keepdims=True)
        upper.append(dict(row,directions=direction.tolist()))
    head=rotations(provenance['source_head_orientation_world']);torso=rotations(provenance['source_torso_orientation_world'])
    accepted,metrics=refiner.feasible_trajectory(result,resized)
    upper_quality=refiner.source_upper_body_error(model,mujoco.MjData(model),result,upper,head)
    deviation=np.abs(result[:,:3]-linear(prior[:,:3])).max(0);acc=np.abs(np.diff(result[:,:3],n=2,axis=0)*2500).max(0)
    knees=(result[:,7+joints.index('left_knee_joint')].min()>=.02-1e-6 and result[:,7+joints.index('right_knee_joint')].max()<=-.02+1e-6)
    accepted=accepted and upper_quality['passed'] and knees and bool((deviation<=np.array([.02,.02,.04])+1e-5).all()) and bool((acc<=3.001).all())
    name=original['motion']+f'_speed{round(args.speed*1000):04d}'
    payload.update(kinematics(model,result),qpos=result,time=times,parent_time=source_time,source_name=np.array(name),postprocess=np.array(str(payload['postprocess'])+f'+retime_{args.speed:g}'),retime_speed=np.array(args.speed),parent_source_name=np.array(original['motion']))
    args.output.mkdir(parents=True,exist_ok=False);out=args.output/(name+'.npz');np.savez_compressed(out,**payload)
    report={**original,'motion':name,'postprocess':str(payload['postprocess']),'numeric_candidate_passed':bool(accepted),'accepted_for_training':False,'visual_review_passed':False,
            'parent_motion':original['motion'],'parent_motion_sha256':hashlib.sha256(args.motion.read_bytes()).hexdigest(),'retime_speed':args.speed,'retime_resampling':resampling,
            'retime_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'parent_sqp_iterations':original['sqp_iterations'],'sqp_iterations':0,
            'retime_note':'Explicit slower-playback augmentation, not original capture timing; source split stays with the parent. No changed pose target or joint-limit clipping.',
            'source_torso_orientation_world':torso.tolist(),'source_head_orientation_world':head.tolist(),'source_upper_body_vectors':upper,
            'source_contacts':[dict(provenance['source_contacts'][i],contact_fraction=float(contact.mean()),plant_intervals=refiner.intervals(contact),maximum_target_swing_clearance_m=float(desired[:,2].max()-.0005)) for i,(desired,yaw,contact) in enumerate(resized)],
            'source_upper_body_quality':upper_quality,'root_translation_deviation_max_m':deviation.tolist(),'root_acceleration_max_m_s2':acc.tolist(),
            'maximum_plant_error_m':metrics['plant_max_m'],'metrics':metrics,'final_geometry':geometry_quality(model,result),'knee_branch_passed':bool(knees)}
    out.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(out),numeric_candidate_passed=bool(accepted),speed=args.speed,metrics=metrics,source_upper_body=upper_quality)),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--motion',type=Path,required=True);p.add_argument('--converted',type=Path,required=True);p.add_argument('--native',type=Path,required=True);p.add_argument('--speed',type=float,required=True);p.add_argument('--output',type=Path,required=True)
    retime(p.parse_args())
