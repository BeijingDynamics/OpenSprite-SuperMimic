"""Source-contact-constrained SQP refinement of native UMR, on actual v5 geometry.

Foot plant intervals and swing paths come from the original human BVH. No
procedural stepping or synthetic root turn is introduced. QA thresholds stay fixed.
"""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np,mujoco,osqp
from scipy import sparse
from scipy.spatial.transform import Rotation,Slerp
from scipy.ndimage import binary_closing,binary_opening
from scipy.signal import savgol_filter
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from retarget.bvh_tools.bvh_adapters import read_transforms,inspect_bvh
from retarget.processing import robot_contract,kinematics,geometry_quality,strict_float32_angles

C=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
def intervals(mask):
    edges=np.diff(np.r_[False,mask,False].astype(int))
    return [(int(a),int(b)) for a,b in zip(np.flatnonzero(edges==1),np.flatnonzero(edges==-1))]

def measured_foot(model,data,geom):
    rotation=data.geom_xmat[geom].reshape(3,3)
    support=data.geom_xpos[geom]-rotation@(np.sign(rotation[2])*model.geom_size[geom])
    return np.r_[data.geom_xpos[geom,:2],support[2]],support

def source_targets(source,prior,native):
    info=inspect_bvh(source)
    _,names,_,_,_,positions,rotations=read_transforms(source)
    human=(positions/100)@C.T
    ts=np.arange(len(human))/info['fps'];t=np.arange(len(prior))/50
    p=np.stack([np.stack([np.interp(t,ts,human[:,j,k]) for k in range(3)],-1) for j in range(len(names))],1)
    with np.load(native,allow_pickle=True) as d:scale=float(np.asarray(d['smpl_scale']).reshape(-1)[0])
    pelvis=p[:,names.index('Hips')]
    X=scale*pelvis[:,:2];Y=prior[:,:2]
    u,_,vt=np.linalg.svd((X-X.mean(0)).T@(Y-Y.mean(0)))
    correction=np.eye(2);correction[-1,-1]=np.sign(np.linalg.det(u@vt))
    rot=u@correction@vt;offset=Y.mean(0)-X.mean(0)@rot
    fit=X@rot+offset
    rms=float(np.sqrt(np.mean((fit-Y)**2)))
    if rms>.15:raise ValueError(f'Human/native root alignment RMS too high: {rms}')
    # UMR's surface-fit scale is a whole-body scale. Sprite has different leg
    # proportions; use actual hip-knee-ankle lengths for locomotion distances.
    model=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/opensprite.xml'));state=mujoco.MjData(model);mujoco.mj_forward(model,state)
    robot_length=sum(np.linalg.norm(state.xpos[model.body(a).id]-state.xpos[model.body(b).id]) for a,b in [('left_hip_pitch_link','left_knee_link'),('left_knee_link','left_ankle_pitch_link')])
    human_length=sum(np.median(np.linalg.norm(p[:,names.index(a)]-p[:,names.index(b)],axis=-1)) for a,b in [('LeftUpLeg','LeftLeg'),('LeftLeg','LeftFoot')])
    surface_scale=scale;scale=float(robot_length/human_length)
    offset=prior[:,:2].mean(0)-(scale*pelvis[:,:2]@rot).mean(0)
    calibrated=np.eye(3);calibrated[:2,:2]=rot.T
    source_rotation=Slerp(ts,Rotation.from_matrix(rotations[:,names.index('Hips')]))(t).as_matrix()
    root_rotation=calibrated@C@source_rotation@C.T
    heading=np.unwrap(np.arctan2(root_rotation[:,1,0],root_rotation[:,0,0]))
    heading_rotation=Rotation.from_euler('z',heading[:,None]).as_matrix()
    tilt=Rotation.from_matrix(heading_rotation.transpose(0,2,1)@root_rotation).as_rotvec()
    heading=savgol_filter(heading,15,3);tilt=savgol_filter(tilt,15,3,axis=0)
    root_rotation=Rotation.from_euler('z',heading[:,None]).as_matrix()@Rotation.from_rotvec(tilt).as_matrix()
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
        # A moving rigid sole needs a small complete-foot clearance. Its box
        # center is not the bottom when it pitches or rolls.
        height=np.where(contact,0.,np.maximum(height,.012))
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
        targets.append((desired,yaw,contact))
    return targets,dict(human_to_robot_scale=scale,native_surface_scale=surface_scale,robot_leg_length_m=float(robot_length),human_leg_length_m=float(human_length),
                       root_orientation_source='BVH Hips SO(3), coordinate conversion and constant native world calibration',
                       root_alignment_rotation=rot.tolist(),root_alignment_offset=offset.tolist(),root_alignment_rms_m=rms,source_contacts=report),adapted_prior

def rot_error(target,current):return Rotation.from_matrix(target@current.T).as_rotvec()

def solve_trajectory(m,d,prior,targets,geoms,qids,vids,limits,speeds,name):
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
    def objective(poses):
        value=0.
        for frame,pose in enumerate(poses):
            d.qpos[:]=pose;mujoco.mj_forward(m,d)
            value+=3*np.square(pose[qids]-prior[frame,qids]).sum()+100*np.square(pose[:3]-prior[frame,:3]).sum()
            target_root=Rotation.from_quat(prior[frame,[4,5,6,3]]).as_matrix()
            value+=5*np.square(rot_error(target_root,d.xmat[m.body('pelvis_link').id].reshape(3,3))).sum()
            for side,g in enumerate(geoms):
                desired,yaw,contact=targets[side];measured,_=measured_foot(m,d,g);error=desired[frame]-measured
                value+=1e7*np.square(error).sum() if contact[frame] else 1000*np.square(error[:2]).sum()+1e5*error[2]**2
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
            for side,g in enumerate(geoms):
                desired,yaw,contact=targets[side]
                jp=np.zeros((3,V));jr=np.zeros_like(jp);mujoco.mj_jacGeom(m,d,jp,jr,g)
                measured,support=measured_foot(m,d,g)
                jb=np.zeros((3,V));rb=np.zeros_like(jb);mujoco.mj_jac(m,d,jb,rb,support,int(m.geom_bodyid[g]))
                foot_jac=np.vstack((jp[:2],jb[2:]))
                if contact[frame]:cost(foot_jac,desired[frame]-measured,1e7)
                else:
                    cost(foot_jac[:2],desired[frame,:2]-measured[:2],1000.)
                    cost(foot_jac[2:],desired[frame,2:]-measured[2:],1e5)
                    # Every corner must clear the floor during a source swing.
                    R=d.geom_xmat[g].reshape(3,3)
                    for x in [-1,1]:
                        for y in [-1,1]:
                            corner=d.geom_xpos[g]+R@(m.geom_size[g]*np.array([x,y,-1]))
                            jc=np.zeros((3,V));rc=np.zeros_like(jc);mujoco.mj_jac(m,d,jc,rc,corner,int(m.geom_bodyid[g]))
                            constraints.append(jc[2:]);lo.append(np.array([.010-corner[2]]));hi.append(np.array([np.inf]))
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
            lower[vids]=np.maximum(lower[vids],limits[:,0]+1e-5-current[frame,qids])
            upper[vids]=np.minimum(upper[vids],limits[:,1]-1e-5-current[frame,qids])
            # Root orientation is the original native retarget's orientation.
            # Unconstrained free-root tilt can otherwise solve foot IK by
            # leaning the entire robot instead of articulating its legs.
            lower[3:6]=0.;upper[3:6]=0.
            constraints.append(eye);lo.append(lower);hi.append(upper)
            A=np.concatenate(rows);error=np.concatenate(errors)
            matrices.append(sparse.csc_matrix(A.T@A+1.*eye));vectors.append(-A.T@error)
            blocks.append(sparse.csc_matrix(np.concatenate(constraints)));low.append(np.concatenate(lo));high.append(np.concatenate(hi))
        residual=np.diff(current[:,position_ids],axis=0).ravel();bound=np.tile(np.r_[1.,1.,.5,.98*speeds]/50,T-1)
        accel_residual=np.diff(current[:,:3],n=2,axis=0).ravel();accel_bound=np.full_like(accel_residual,8./2500)
        constraints=sparse.vstack([sparse.block_diag(blocks,format='csc'),velocity,root_acceleration],format='csc')
        solver=osqp.OSQP();solver.setup(P=sparse.triu(sparse.block_diag(matrices,format='csc')).tocsc(),q=np.concatenate(vectors),
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
        if abs(delta).max()<5e-5:break
    return current,iteration+1

def refine_clip(name,input_dir,native_dir,output_dir,recover_report=False):
    src=input_dir/(name+'.npz')
    with np.load(src,allow_pickle=False) as data:payload={k:data[k] for k in data.files}
    prior=np.asarray(payload['qpos'],float)
    source=ROOT/'data/sources/lafan1-resolved'/(name+'.bvh')
    targets,provenance,prior=source_targets(source,prior,native_dir/name/'native.npz')
    m=mujoco.MjModel.from_xml_path(str(ROOT/'assets/opensprite/retarget_scene.xml'));d=mujoco.MjData(m)
    geoms=[m.geom(side+'_foot_collision').id for side in ['left','right']]
    qids=np.array([m.jnt_qposadr[j] for j in range(1,m.njnt)],int)
    vids=np.array([m.jnt_dofadr[j] for j in range(1,m.njnt)],int)
    plain,joints,limits,speeds=robot_contract()
    # Broadphase includes near contacts for inequality linearization only.
    m.geom_margin[:]=.005
    if recover_report:
        with np.load(output_dir/(name+'.npz'),allow_pickle=False) as solved:
            if str(solved['postprocess'])!='source_contact_constrained_trajectory_sqp_v9':raise ValueError('Wrong saved solver version')
            result=solved['qpos'].copy()
        count=60
    else:result,count=solve_trajectory(m,d,prior,targets,geoms,qids,vids,limits,speeds,name)
    plant_errors=[]
    for frame,pose in enumerate(result):
        d.qpos[:]=pose;mujoco.mj_forward(m,d)
        plant_errors.extend([np.linalg.norm(measured_foot(m,d,g)[0]-targets[side][0][frame]) for side,g in enumerate(geoms) if targets[side][2][frame]])
    maximum=max(plant_errors or [0.])
    if maximum>.002:raise RuntimeError(f'{name}: final maximum plant residual {maximum} exceeds 2mm')
    write_result(name,src,source,output_dir,plain,result,payload,limits,provenance,maximum,count)
    return output_dir/(name+'.npz')

def write_result(name,src,source,output_dir,plain,result,payload,limits,provenance,max_plant_error,iteration_count):
    qpos=np.array(result);qpos[:,7:]=strict_float32_angles(qpos[:,7:],limits)
    quality=geometry_quality(plain,qpos)
    if quality['minimum_collision_z_m']<-.002 or quality['self_minimum_signed_distance_m']<-.01:raise RuntimeError('Final contact refinement geometry failed')
    payload.update(kinematics(plain,qpos),qpos=qpos,postprocess=np.array('source_contact_constrained_trajectory_sqp_v9'))
    output_dir.mkdir(parents=True,exist_ok=True);out=output_dir/(name+'.npz');np.savez_compressed(out,**payload)
    report=dict(motion=name,final_geometry=quality,postprocess='source_contact_constrained_trajectory_sqp_v9',
                refiner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),native_input_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),
                maximum_plant_error_m=max_plant_error,sqp_iterations=iteration_count,**provenance)
    out.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--name',required=True);p.add_argument('--input-dir',type=Path,required=True)
    p.add_argument('--native-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--recover-report',action='store_true');a=p.parse_args()
    refine_clip(a.name,a.input_dir,a.native_dir,a.output_dir,a.recover_report)
