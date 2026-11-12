"""Isolated Neutral contact refinement with source-bounded smooth root motion."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np,mujoco,osqp
from scipy import sparse
from scipy.spatial.transform import Rotation,Slerp
from scipy.signal import savgol_filter
from scipy.ndimage import binary_closing,binary_opening
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from refine_reference_contacts import measured_foot,rot_error,feasible_trajectory,intervals,C
from retarget.bvh_tools.bvh_adapters import inspect_bvh,read_transforms
from retarget.processing import robot_contract,kinematics,geometry_quality,strict_float32_angles
from audit_geno_source_frame import verify_source_frame,NATIVE_Y_UP_TO_Z_UP
from retarget.bvh_tools.motion_postprocess import process_motion

ARM_SEGMENTS=(('Arm','ForeArm','shoulder_pitch_link','elbow_link'),
              ('ForeArm','Hand','elbow_link','wrist_roll_link'))


def unit_direction_and_jacobian(vector,jacobian):
    """Differentiate a direction without introducing a robot/human length target."""
    length=np.linalg.norm(vector)
    if length<1e-6:raise ValueError('Degenerate limb direction')
    direction=vector/length
    return direction,(np.eye(3)-np.outer(direction,direction))@jacobian/length


def source_upper_body_error(model,data,poses,upper_body,head_rotations):
    tasks=[(model.body(row['start_body']).id,model.body(row['end_body']).id,np.asarray(row['directions'])) for row in upper_body]
    arm=[];head=[];head_id=model.body('head_yaw_link').id
    for i,pose in enumerate(poses):
        data.qpos[:]=pose;mujoco.mj_forward(model,data)
        for a,b,target in tasks:
            actual=data.xpos[b]-data.xpos[a];actual/=np.linalg.norm(actual)
            arm.append(np.rad2deg(np.arccos(np.clip(actual@target[i],-1,1))))
        head.append(np.rad2deg(np.linalg.norm(rot_error(head_rotations[i],data.xmat[head_id].reshape(3,3)))))
    report=dict(arm_direction_p95_deg=float(np.quantile(arm,.95)),arm_direction_max_deg=float(max(arm)),
                head_rotation_p95_deg=float(np.quantile(head,.95)),head_rotation_max_deg=float(max(head)))
    report['passed']=all(report[k]<=v for k,v in [('arm_direction_p95_deg',10),('arm_direction_max_deg',20),('head_rotation_p95_deg',10),('head_rotation_max_deg',20)])
    return report

def stabilize_support_yaw(yaw,contact):
    """Retain planted headings; interpolate undefined toe-off yaw between plants."""
    plants=intervals(contact)
    if not plants:raise ValueError('Foot has no source support interval')
    result=np.asarray(yaw,float).copy()
    anchors=np.unwrap(np.array([np.median(yaw[a:b]) for a,b in plants]))
    for (a,b),angle in zip(plants,anchors):result[a:b]=angle
    result[:plants[0][0]]=anchors[0];result[plants[-1][1]:]=anchors[-1]
    for i in range(len(plants)-1):
        first=plants[i][1]-1;last=plants[i+1][0]
        t=np.arange(last-first+1)/(last-first)
        blend=t**3*(10-15*t+6*t*t)
        result[first:last+1]=anchors[i]+blend*(anchors[i+1]-anchors[i])
    return result

def continuous_rigid_sole_swing(desired,contact):
    """Retain source support anchors, phases and peaks with smooth rigid-sole travel."""
    result=desired.copy()
    for begin,end in intervals(~contact):
        if begin==0 or end==len(contact):continue
        first,last=begin-1,end
        s=np.arange(last-first+1)/(last-first)
        travel=np.clip((s-.1)/.8,0.,1.)
        blend=travel**3*(10-15*travel+6*travel*travel)
        result[first:last+1,:2]=desired[first,:2]+blend[:,None]*(desired[last,:2]-desired[first,:2])
        peak=max(0.,float(desired[begin:end,2].max())-.0005)
        result[first:last+1,2]=.0005+peak*64*s**3*(1-s)**3
    return result

def source_targets_neutral(source,prior,native):
    info=inspect_bvh(source)
    frame_audit=verify_source_frame(source)
    _,names,_,_,_,positions,rotations=read_transforms(source)
    human=(positions/100)@C.T
    ts=np.arange(len(human))/info['fps'];t=np.arange(len(prior))/50
    p=np.stack([np.stack([np.interp(t,ts,human[:,j,k]) for k in range(3)],-1) for j in range(len(names))],1)
    with np.load(native,allow_pickle=True) as d:scale=float(np.asarray(d['smpl_scale']).reshape(-1)[0])
    pelvis=p[:,names.index('Hips')]
    X=scale*pelvis[:,:2];Y=prior[:,:2]
    u,_,vt=np.linalg.svd((X-X.mean(0)).T@(Y-Y.mean(0)))
    correction=np.eye(2);correction[-1,-1]=np.sign(np.linalg.det(u@vt))
    fitted_rotation=u@correction@vt;fitted_offset=Y.mean(0)-X.mean(0)@fitted_rotation
    fit=X@fitted_rotation+fitted_offset
    rms=float(np.sqrt(np.mean((fit-Y)**2)))
    # The native source frame is known and independently verified before IK.
    # A drifting optimized robot trajectory is not a coordinate calibration.
    calibrated=NATIVE_Y_UP_TO_Z_UP@C.T
    rot=calibrated[:2,:2].T
    # UMR's surface-fit scale is a whole-body scale. Sprite has different leg
    # proportions; use actual hip-knee-ankle lengths for locomotion distances.
    model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/opensprite.xml'));state=mujoco.MjData(model);mujoco.mj_forward(model,state)
    robot_length=sum(np.linalg.norm(state.xpos[model.body(a).id]-state.xpos[model.body(b).id]) for a,b in [('left_hip_pitch_link','left_knee_link'),('left_knee_link','left_ankle_pitch_link')])
    human_length=sum(np.median(np.linalg.norm(p[:,names.index(a)]-p[:,names.index(b)],axis=-1)) for a,b in [('LeftUpLeg','LeftLeg'),('LeftLeg','LeftFoot')])
    surface_scale=scale;scale=float(robot_length/human_length)
    offset=prior[0,:2]-scale*pelvis[0,:2]@rot
    source_rotation=Slerp(ts,Rotation.from_matrix(rotations[:,names.index('Hips')]))(t).as_matrix()
    root_rotation=calibrated@C@source_rotation@C.T
    heading=np.unwrap(np.arctan2(root_rotation[:,1,0],root_rotation[:,0,0]))
    heading_rotation=Rotation.from_euler('z',heading[:,None]).as_matrix()
    tilt=Rotation.from_matrix(heading_rotation.transpose(0,2,1)@root_rotation).as_rotvec()
    heading=savgol_filter(heading,15,3);tilt=savgol_filter(tilt,15,3,axis=0)
    root_rotation=Rotation.from_euler('z',heading[:,None]).as_matrix()@Rotation.from_rotvec(tilt).as_matrix()
    # Surface-only IK can settle into a wrong waist twist, especially at clip
    # entry. Preserve the measured human torso orientation as an explicit task.
    human_torso=Slerp(ts,Rotation.from_matrix(rotations[:,names.index('Spine3')]))(t).as_matrix()
    torso_rotation=calibrated@C@human_torso@C.T
    relative_torso=Rotation.from_matrix(root_rotation.transpose(0,2,1)@torso_rotation).as_rotvec()
    relative_torso=savgol_filter(relative_torso,15,3,axis=0)
    torso_rotation=root_rotation@Rotation.from_rotvec(relative_torso).as_matrix()
    upper_body=[]
    for side in ['left','right']:
        for a,b,ra,rb in ARM_SEGMENTS:
            vector=(p[:,names.index(side.title()+b)]-p[:,names.index(side.title()+a)])@calibrated.T
            vector=savgol_filter(vector,15,3,axis=0)
            vector=vector/np.linalg.norm(vector,axis=-1,keepdims=True)
            upper_body.append(dict(start_body=side+'_'+ra,end_body=side+'_'+rb,directions=vector.tolist()))
    head_rotation=calibrated@C@Slerp(ts,Rotation.from_matrix(rotations[:,names.index('Head')]))(t).as_matrix()@C.T
    relative_head=Rotation.from_matrix(torso_rotation.transpose(0,2,1)@head_rotation).as_rotvec()
    head_rotation=torso_rotation@Rotation.from_rotvec(savgol_filter(relative_head,15,3,axis=0)).as_matrix()
    adapted_prior=prior.copy();adapted_prior[:,:2]=scale*pelvis[:,:2]@rot+offset
    adapted_prior[:,3:7]=Rotation.from_matrix(root_rotation).as_quat()[:,[3,0,1,2]]
    adapted_prior[:,2]=np.median(prior[:,2])+scale*(pelvis[:,2]-np.median(pelvis[:,2]))
    targets=[];report=[]
    for side in ['Left','Right']:
        ankle=p[:,names.index(side+'Foot')]
        toe=p[:,names.index(side+'ToeBase')]
        foot=.5*(ankle+toe)
        va=np.linalg.norm(np.gradient(savgol_filter(ankle[:,:2],9,2,axis=0),.02,axis=0),axis=-1)
        vt_=np.linalg.norm(np.gradient(savgol_filter(toe[:,:2],9,2,axis=0),.02,axis=0),axis=-1)
        ha=ankle[:,2]-np.quantile(ankle[:,2],.05);ht=toe[:,2]-np.quantile(toe[:,2],.05)
        contact=(np.minimum(ha,ht)<.035)&(np.minimum(va,vt_)<.10)
        pad=np.pad(contact,3,mode='edge')
        contact=binary_opening(binary_closing(pad,structure=np.ones(3)),structure=np.ones(3))[3:-3]
        xy=scale*foot[:,:2]@rot+offset
        direction=(toe[:,:2]-ankle[:,:2])@rot
        yaw=np.unwrap(np.arctan2(direction[:,1],direction[:,0]))
        # Preserve the human's measured vertical swing at the native UMR scale.
        clearance_scale=1.
        height=np.maximum(0,clearance_scale*scale*np.minimum(ha,ht))
        # Preserve continuous source clearance at lift-off and touchdown.
        # A fixed positive floor during every swing sample creates a target jump.
        height=np.where(contact,0.,height)
        desired=np.c_[xy,height+.0005]
        for begin,end in intervals(contact):
            desired[begin:end,:2]=np.median(xy[begin:end],axis=0)
            desired[begin:end,2]=.0005
            yaw[begin:end]=np.median(yaw[begin:end])
        # Carry the measured swing path continuously between the corrected
        # plant anchors; a median anchor must not introduce a touchdown jump.
        for begin,end in intervals(~contact):
            first=max(0,begin-1);last=min(len(contact)-1,end)
            left=desired[first,:2]-xy[first];right=desired[last,:2]-xy[last]
            blend=np.linspace(0,1,end-begin+2)[1:-1,None]
            desired[begin:end,:2]=xy[begin:end]+(1-blend)*left+blend*right
        report.append(dict(side=side,contact_fraction=float(contact.mean()),plant_intervals=intervals(contact),vertical_clearance_scale=clearance_scale,maximum_target_swing_clearance_m=float(height.max())))
        yaw=stabilize_support_yaw(yaw,contact)
        desired=continuous_rigid_sole_swing(desired,contact)
        targets.append((desired,yaw,contact))
    return targets,dict(human_to_robot_scale=scale,native_surface_scale=surface_scale,robot_leg_length_m=float(robot_length),human_leg_length_m=float(human_length),
                       root_orientation_source='BVH Hips SO(3), independently verified native Geno source frame',
                       source_frame_audit=frame_audit,source_frame_revision='native_geno_y_up_to_z_up_v1',
                       source_frame_verifier_sha256=hashlib.sha256((ROOT/'scripts/audit_geno_source_frame.py').read_bytes()).hexdigest(),
                       root_alignment_rotation=rot.tolist(),root_alignment_offset=offset.tolist(),native_root_fit_rms_diagnostic_m=rms,
                       native_root_fit_note='Retained diagnostic, never used to infer or admit the known source coordinate transform.',source_contacts=report,
                       torso_orientation_source='Geno Spine3 world orientation, same calibration as Hips; 15-frame relative SO(3) smoothing',
                       source_torso_orientation_world=torso_rotation.tolist(),
                       source_upper_body_vectors=upper_body,source_head_orientation_world=head_rotation.tolist()),adapted_prior


def retime_source_tasks(prior,targets,provenance,speed,model):
    """Resample fixed source tasks before optimization; keep source contact phases."""
    retimed,times,metadata=process_motion(prior,50*speed,50,model=model)
    sample=times*speed;original_time=np.arange(len(prior))/50
    def linear(values):
        values=np.asarray(values);flat=values.reshape(len(prior),-1)
        return np.stack([np.interp(sample,original_time,flat[:,j]) for j in range(flat.shape[1])],-1).reshape((len(times),)+values.shape[1:])
    result=[(linear(x),linear(yaw),linear(contact.astype(float))>=.5) for x,yaw,contact in targets]
    updated=dict(provenance)
    for key in ['source_torso_orientation_world','source_head_orientation_world']:
        updated[key]=Slerp(original_time,Rotation.from_matrix(np.asarray(provenance[key])))(sample).as_matrix().tolist()
    updated['source_upper_body_vectors']=[]
    for row in provenance['source_upper_body_vectors']:
        direction=linear(row['directions']);direction/=np.linalg.norm(direction,axis=-1,keepdims=True)
        updated['source_upper_body_vectors'].append(dict(row,directions=direction.tolist()))
    updated['source_contacts']=[dict(provenance['source_contacts'][i],contact_fraction=float(c.mean()),plant_intervals=intervals(c)) for i,(_,_,c) in enumerate(result)]
    updated.update(source_speed=speed,source_retiming=metadata,source_timing_note='Explicit time scaling before constrained trajectory optimization; reference commands are recomputed from the final trajectory.')
    return retimed,result,updated,times


def solve_trajectory(m,d,prior,targets,geoms,qids,vids,limits,speeds,name,stop_when_feasible=False,torso_rotations=None,upper_body=None,head_rotations=None):
    """Optimize the complete clip so swing preparation can precede touchdown.

    Frame-by-frame IK with a previous-frame velocity bound cannot anticipate a
    planted foot; the trajectory QP enforces the same limits across all frames.
    """
    T,V=len(prior),m.nv;eye=np.eye(V);current=prior.copy()
    neutral=json.loads((ROOT/'data/command_default_pose.json').read_text())
    for j in range(1,m.njnt):
        joint=m.joint(j).name
        if joint.startswith(('left_hip','right_hip','left_knee','right_knee','left_ankle','right_ankle')):
            current[:,int(m.jnt_qposadr[j])]=neutral[joint]
    debug=ROOT/'outputs/refinement-debug';debug.mkdir(exist_ok=True)
    arm_tasks=[(m.body(row['start_body']).id,m.body(row['end_body']).id,np.asarray(row['directions'])) for row in (upper_body or [])]
    head=m.body('head_yaw_link').id
    def objective(poses):
        value=.05*np.square((np.diff(poses[:,:3],n=2,axis=0)-np.diff(prior[:,:3],n=2,axis=0))*2500).sum()
        value+=1e-4*np.square(np.diff(poses[:,qids],n=2,axis=0)*2500).sum()
        for frame,pose in enumerate(poses):
            d.qpos[:]=pose;mujoco.mj_forward(m,d)
            value+=3*np.square(pose[qids]-prior[frame,qids]).sum()+100*np.square(pose[:3]-prior[frame,:3]).sum()
            target_root=Rotation.from_quat(prior[frame,[4,5,6,3]]).as_matrix()
            value+=5*np.square(rot_error(target_root,d.xmat[m.body('pelvis_link').id].reshape(3,3))).sum()
            if torso_rotations is not None:
                value+=1000*np.square(rot_error(torso_rotations[frame],d.xmat[m.body('waist_yaw_link').id].reshape(3,3))).sum()
            for a,b,arm_targets in arm_tasks:
                actual=d.xpos[b]-d.xpos[a];actual/=np.linalg.norm(actual)
                value+=1000*np.square(arm_targets[frame]-actual).sum()
            if head_rotations is not None:
                value+=1000*np.square(rot_error(head_rotations[frame],d.xmat[head].reshape(3,3))).sum()
            for side,g in enumerate(geoms):
                desired,yaw,contact=targets[side];measured,_=measured_foot(m,d,g);error=desired[frame]-measured
                value+=1e7*np.square(error).sum() if contact[frame] else 1e5*np.square(error[:2]).sum()+1e5*error[2]**2
                value+=1e3*np.square(rot_error(Rotation.from_euler('z',yaw[frame]).as_matrix(),d.geom_xmat[g].reshape(3,3))).sum()
            value+=1e8*sum(max(0.,-c.dist)**2 for c in d.contact[:d.ncon])
        return value
    current_objective=objective(current)
    difference=sparse.diags([-np.ones(T-1),np.ones(T-1)],[0,1],shape=(T-1,T),format='csc')
    position_ids=np.r_[0,1,2,qids];velocity_ids=np.r_[0,1,2,vids]
    joint_selector=sparse.csc_matrix(eye[velocity_ids])
    velocity=sparse.kron(difference,joint_selector,format='csc')
    second_difference=sparse.diags([np.ones(T-2),-2*np.ones(T-2),np.ones(T-2)],[0,1,2],shape=(T-2,T),format='csc')
    root_acceleration=sparse.kron(second_difference,sparse.csc_matrix(eye[:3]),format='csc')
    joint_acceleration=sparse.kron(second_difference,sparse.csc_matrix(eye[vids]),format='csc')
    for iteration in range(80):
        matrices=[];vectors=[];blocks=[];low=[];high=[]
        for frame in range(T):
            d.qpos[:]=current[frame];mujoco.mj_forward(m,d)
            rows=[];errors=[];constraints=[];lo=[];hi=[]
            def cost(jac,error,weight):rows.append(np.sqrt(weight)*jac);errors.append(np.sqrt(weight)*error)
            cost(eye[vids],prior[frame,qids]-current[frame,qids],3.)
            cost(eye[:3],prior[frame,:3]-current[frame,:3],100.)
            jp=np.zeros((3,V));jr=np.zeros_like(jp);mujoco.mj_jacBody(m,d,jp,jr,m.body('pelvis_link').id)
            target_root=Rotation.from_quat(prior[frame,[4,5,6,3]]).as_matrix()
            cost(jr,rot_error(target_root,d.xmat[m.body('pelvis_link').id].reshape(3,3)),5.)
            if torso_rotations is not None:
                jp=np.zeros((3,V));jr=np.zeros_like(jp);body=m.body('waist_yaw_link').id
                mujoco.mj_jacBody(m,d,jp,jr,body)
                cost(jr,rot_error(torso_rotations[frame],d.xmat[body].reshape(3,3)),1000.)
            for a,b,arm_targets in arm_tasks:
                ja=np.zeros((3,V));jb=np.zeros((3,V));unused=np.zeros((3,V))
                mujoco.mj_jacBody(m,d,ja,unused,a);mujoco.mj_jacBody(m,d,jb,unused,b)
                actual,jacobian=unit_direction_and_jacobian(d.xpos[b]-d.xpos[a],jb-ja)
                cost(jacobian,arm_targets[frame]-actual,1000.)
            if head_rotations is not None:
                jp=np.zeros((3,V));jr=np.zeros_like(jp);mujoco.mj_jacBody(m,d,jp,jr,head)
                cost(jr,rot_error(head_rotations[frame],d.xmat[head].reshape(3,3)),1000.)
            for side,g in enumerate(geoms):
                desired,yaw,contact=targets[side]
                jp=np.zeros((3,V));jr=np.zeros_like(jp);mujoco.mj_jacGeom(m,d,jp,jr,g)
                measured,support=measured_foot(m,d,g)
                jb=np.zeros((3,V));rb=np.zeros_like(jb);mujoco.mj_jac(m,d,jb,rb,support,int(m.geom_bodyid[g]))
                foot_jac=np.vstack((jp[:2],jb[2:]))
                if contact[frame]:cost(foot_jac,desired[frame]-measured,1e7)
                else:
                    cost(foot_jac[:2],desired[frame,:2]-measured[:2],1e5)
                    cost(foot_jac[2:],desired[frame,2:]-measured[2:],1e5)
                    # Every corner must clear the floor during a source swing.
                    R=d.geom_xmat[g].reshape(3,3)
                    for x in [-1,1]:
                        for y in [-1,1]:
                            corner=d.geom_xpos[g]+R@(m.geom_size[g]*np.array([x,y,-1]))
                            jc=np.zeros((3,V));rc=np.zeros_like(jc);mujoco.mj_jac(m,d,jc,rc,corner,int(m.geom_bodyid[g]))
                            constraints.append(jc[2:]);lo.append(np.array([.0003-corner[2]]));hi.append(np.array([np.inf]))
                # Sprite's rigid sole cannot reproduce a human's articulated
                # toe roll. Preserve measured sole clearance with a flat sole.
                cost(jr,rot_error(Rotation.from_euler('z',yaw[frame]).as_matrix(),d.geom_xmat[g].reshape(3,3)),1e3)
            for contact in d.contact[:d.ncon]:
                b1,b2=[int(m.geom_bodyid[g]) for g in [contact.geom1,contact.geom2]]
                jacobians=[]
                for body in [b1,b2]:
                    jac=np.zeros((3,V));rotjac=np.zeros_like(jac)
                    if body:mujoco.mj_jac(m,d,jac,rotjac,contact.pos,body)
                    jacobians.append(jac)
                constraints.append((contact.frame[:3]@(jacobians[1]-jacobians[0]))[None])
                clearance=(.0003 if not b1 or not b2 else .0005)-contact.dist
                lo.append(np.array([min(.5*clearance,.005) if clearance>0 else clearance]));hi.append(np.array([np.inf]))
            lower=np.full(V,-.3);upper=np.full(V,.3);lower[:3]=-.05;upper[:3]=.05
            root_bound=np.array([.02,.02,.04])
            lower[:3]=np.maximum(lower[:3],prior[frame,:3]-root_bound-current[frame,:3])
            upper[:3]=np.minimum(upper[:3],prior[frame,:3]+root_bound-current[frame,:3])
            lower[vids]=np.maximum(lower[vids],limits[:,0]+1e-5-current[frame,qids])
            upper[vids]=np.minimum(upper[vids],limits[:,1]-1e-5-current[frame,qids])
            # Root orientation follows the calibrated source orientation.
            # Unconstrained free-root tilt can otherwise solve foot IK by
            # leaning the entire robot instead of articulating its legs.
            lower[3:6]=0.;upper[3:6]=0.
            constraints.append(eye);lo.append(lower);hi.append(upper)
            A=np.concatenate(rows);error=np.concatenate(errors)
            matrices.append(sparse.csc_matrix(A.T@A+1.*eye));vectors.append(-A.T@error)
            blocks.append(sparse.csc_matrix(np.concatenate(constraints)));low.append(np.concatenate(lo));high.append(np.concatenate(hi))
        residual=np.diff(current[:,position_ids],axis=0).ravel();bound=np.tile(np.r_[1.,1.,.5,.98*speeds]/50,T-1)
        accel_residual=np.diff(current[:,:3],n=2,axis=0).ravel();accel_bound=np.full_like(accel_residual,3./2500)
        constraints=sparse.vstack([sparse.block_diag(blocks,format='csc'),velocity,root_acceleration],format='csc')
        accel_weight=.05*2500**2
        hessian=sparse.block_diag(matrices,format='csc')+accel_weight*(root_acceleration.T@root_acceleration)
        gradient=np.concatenate(vectors)+accel_weight*(root_acceleration.T@(accel_residual-np.diff(prior[:,:3],n=2,axis=0).ravel()))
        joint_accel_weight=1e-4*2500**2
        hessian=hessian+joint_accel_weight*(joint_acceleration.T@joint_acceleration)
        gradient=gradient+joint_accel_weight*(joint_acceleration.T@np.diff(current[:,qids],n=2,axis=0).ravel())
        solver=osqp.OSQP();solver.setup(P=sparse.triu(hessian).tocsc(),q=gradient,
            A=constraints,l=np.r_[np.concatenate(low),-bound-residual,-accel_bound-accel_residual],u=np.r_[np.concatenate(high),bound-residual,accel_bound-accel_residual],
            verbose=False,eps_abs=1e-5,eps_rel=1e-5,max_iter=30000,polishing=False)
        answer=solver.solve()
        if answer.info.status_val not in (1,2):raise RuntimeError(f'Trajectory SQP {iteration}: {answer.info.status}')
        delta=answer.x.reshape(T,V)
        alpha=1.
        for _ in range(9):
            candidate=current.copy()
            for frame in range(T):mujoco.mj_integratePos(m,candidate[frame],delta[frame],alpha)
            candidate_objective=objective(candidate)
            if candidate_objective<=current_objective+1e-7:break
            alpha*=.5
        if candidate_objective>current_objective+1e-7:
            print('LINE_SEARCH_STALLED',iteration,flush=True);break
        current=candidate;current_objective=candidate_objective;delta*=alpha
        if iteration%5==0:np.savez_compressed(debug/(name+'.npz'),qpos=current,iteration=iteration,objective=current_objective)
        plant=[]
        for frame in range(T):
            d.qpos[:]=current[frame];mujoco.mj_forward(m,d)
            plant.extend([float(np.linalg.norm(measured_foot(m,d,g)[0]-targets[side][0][frame])) for side,g in enumerate(geoms) if targets[side][2][frame]])
        print('TRAJECTORY_SQP',iteration,'delta_max',float(abs(delta).max()),'plant_max',max(plant or [0]),'objective',current_objective,'alpha',alpha,flush=True)
        if stop_when_feasible and iteration>=10 and iteration%5==0:
            accepted,quality=feasible_trajectory(current,targets)
            if upper_body is not None and head_rotations is not None:
                quality['source_upper_body']=source_upper_body_error(m,d,current,upper_body,head_rotations)
                accepted=accepted and quality['source_upper_body']['passed']
            if torso_rotations is not None:
                errors=[]
                for frame,pose in enumerate(current):
                    d.qpos[:]=pose;mujoco.mj_forward(m,d)
                    actual=d.xmat[m.body('waist_yaw_link').id].reshape(3,3)
                    error=np.arctan2(actual[1,0],actual[0,0])-np.arctan2(torso_rotations[frame,1,0],torso_rotations[frame,0,0])
                    errors.append(abs(np.arctan2(np.sin(error),np.cos(error))))
                quality['source_torso_yaw_max_error_deg']=float(np.rad2deg(max(errors)))
                accepted=accepted and max(errors)<np.deg2rad(5)
            print('TRAJECTORY_FEASIBILITY',iteration,json.dumps(quality),flush=True)
            if accepted:
                print('STOP_AT_FEASIBLE_ITERATE: independent bank QA and visual review still required',flush=True)
                break
        if abs(delta).max()<5e-5:break
    return current,iteration+1


def main():
    p=argparse.ArgumentParser();p.add_argument('--name',required=True)
    p.add_argument('--input-dir',type=Path,required=True);p.add_argument('--native-dir',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--source-speed',type=float,default=1.,help='Explicit source time scaling before constrained optimization')
    a=p.parse_args()
    if not np.isfinite(a.source_speed) or not .25<=a.source_speed<=1:
        p.error('Source speed must be between 0.25 and 1.0')
    src=a.input_dir/(a.name+'.npz')
    with np.load(src,allow_pickle=False) as z:payload={k:z[k] for k in z.files}
    source=ROOT/'data/sources/lafan1-resolved'/(a.name+'.bvh')
    targets,provenance,prior=source_targets_neutral(source,np.asarray(payload['qpos'],float),a.native_dir/a.name/'native.npz')
    model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/retarget_scene.xml'));data=mujoco.MjData(model)
    times=np.arange(len(prior))/50;name=a.name
    provenance['source_speed']=a.source_speed
    if a.source_speed!=1:
        prior,targets,provenance,times=retime_source_tasks(prior,targets,provenance,a.source_speed,model)
        name+=f'_speed{round(a.source_speed*1000):04d}'
    feet=[model.geom(side+'_foot_collision').id for side in ('left','right')]
    qids=np.array([model.jnt_qposadr[j] for j in range(1,model.njnt)],int)
    vids=np.array([model.jnt_dofadr[j] for j in range(1,model.njnt)],int)
    plain,joints,limits,speeds=robot_contract();model.geom_margin[:]=.005
    optimization_limits=limits.copy()
    optimization_limits[joints.index('left_knee_joint'),0]=.02
    optimization_limits[joints.index('right_knee_joint'),1]=-.02
    result,count=solve_trajectory(model,data,prior,targets,feet,qids,vids,optimization_limits,speeds,name+'-smooth',True,
                                  np.asarray(provenance['source_torso_orientation_world']),
                                  provenance['source_upper_body_vectors'],np.asarray(provenance['source_head_orientation_world']))
    result[:,7:]=strict_float32_angles(result[:,7:],limits)
    accepted,metrics=feasible_trajectory(result,targets)
    deviation=np.abs(result[:,:3]-prior[:,:3]).max(0)
    acceleration=np.abs(np.diff(result[:,:3],n=2,axis=0)*2500).max(0)
    accepted=accepted and bool((deviation<=np.array([.02,.02,.04])+1e-5).all()) and bool((acceleration<=3.001).all())
    knee_branch=(result[:,qids[joints.index('left_knee_joint')]].min()>=.02-1e-6 and
                 result[:,qids[joints.index('right_knee_joint')]].max()<=-.02+1e-6)
    accepted=accepted and bool(knee_branch)
    upper_quality=source_upper_body_error(model,data,result,provenance['source_upper_body_vectors'],np.asarray(provenance['source_head_orientation_world']))
    accepted=accepted and upper_quality['passed']
    method='neutral_verified_source_frame_contact_sqp_v10'
    payload.update(kinematics(plain,result),qpos=result,time=times,source_name=np.array(name),source_speed=np.array(a.source_speed),parent_source_name=np.array(a.name),postprocess=np.array(method),dataset_revision=np.array('100style-neutral-v5-research-20261112'))
    a.output_dir.mkdir(parents=True,exist_ok=False);out=a.output_dir/(name+'.npz');np.savez_compressed(out,**payload)
    report=dict(motion=name,parent_source_name=a.name,final_geometry=geometry_quality(plain,result),postprocess=method,
                numeric_candidate_passed=bool(accepted),accepted_for_training=False,visual_review_passed=False,
                root_translation_deviation_max_m=deviation.tolist(),root_acceleration_max_m_s2=acceleration.tolist(),
                refiner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                shared_refiner_sha256=hashlib.sha256((ROOT/'scripts/refine_reference_contacts.py').read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),native_input_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
                maximum_plant_error_m=metrics['plant_max_m'],sqp_iterations=count,source_torso_task_enabled=True,
                swing_height_note='Complete source swing intervals use C2 source-peak clearance with horizontal travel in the middle80percent; partial boundary swings retain their source path. No constant swing-height floor.',
                joint_acceleration_weight=1e-4,
                source_upper_body_quality=upper_quality,
                upper_body_note='Source shoulder-elbow and elbow-wrist unit directions plus source head orientation; all official joint limits retained.',
                swing_yaw_note='Measured planted headings with quintic interpolation through toe-off projection singularities.',
                knee_branch_note='Reference IK stays on the natural flexion branch; physical model and hardware bounds are unchanged.',
                knee_branch_passed=bool(knee_branch),metrics=metrics,**provenance)
    out.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('source_torso_orientation_world','source_upper_body_vectors','source_head_orientation_world')}),flush=True)


if __name__=='__main__':main()
