"""Create a separate PM01 phase-preserving contact/clearance research candidate.

This is an engineered reference correction, not an unmodified demonstration.
Kinematic support phases are inferred from source foot motion; they are not
measured forces. Root orientation and every non-leg joint remain unchanged.
Trajectory mode permits bounded root translation and twelve leg joints on v5;
root orientation and all non-leg joint angles remain the source values.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.ndimage import binary_closing, binary_opening
from scipy.optimize import least_squares
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation
from scipy import sparse
import osqp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from retarget.processing import robot_contract, kinematics, geometry_quality
from opensprite_tracker.robot import ASSET_REVISION


def intervals(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def foot_state(model, data, geom):
    rotation = data.geom_xmat[geom].reshape(3, 3)
    corner = data.geom_xpos[geom] - rotation @ (np.sign(rotation[2]) * model.geom_size[geom])
    return np.r_[data.geom_xpos[geom, :2], corner[2]], rotation


def build_targets(model, poses, clearance):
    data = mujoco.MjData(model)
    geoms = [model.geom(side + '_foot_collision').id for side in ('left', 'right')]
    points, yaws = [], []
    for pose in poses:
        data.qpos[:] = pose
        mujoco.mj_forward(model, data)
        samples = [foot_state(model, data, g) for g in geoms]
        points.append([x[0] for x in samples])
        yaws.append([np.arctan2(x[1][1, 0], x[1][0, 0]) for x in samples])
    points = np.asarray(points)
    yaws = np.unwrap(np.asarray(yaws), axis=0)
    velocity = savgol_filter(np.gradient(points[..., :2], .02, axis=0), 9, 3, axis=0)
    speed = np.linalg.norm(velocity, axis=-1)
    contact = np.stack([binary_opening(binary_closing(np.pad(speed[:, j] < .12, 3, mode='edge'),
                        structure=np.ones(3)), structure=np.ones(3))[3:-3] for j in range(2)], -1)
    both = np.flatnonzero(contact.all(-1))
    eligible = both[(both >= 5) & (both < len(poses) - 5)]
    if len(eligible) < 2:
        raise ValueError('No common support boundaries for a complete reference segment')
    begin, end = int(eligible[0]), int(eligible[-1]) + 1
    points, yaws, contact = points[begin:end], yaws[begin:end], contact[begin:end]
    if (~contact.any(-1)).any():
        raise ValueError('Inferred flight interval: do not invent a support phase')
    targets = points.copy()
    for foot in range(2):
        for first, last in intervals(contact[:, foot]):
            targets[first:last, foot, :2] = np.median(points[first:last, foot, :2], axis=0)
            targets[first:last, foot, 2] = .0005
            yaws[first:last, foot] = np.median(yaws[first:last, foot])
        for first, last in intervals(~contact[:, foot]):
            if first == 0 or last == len(points):
                raise ValueError('Incomplete swing at a selected support boundary')
            phase = np.arange(1, last - first + 1) / (last - first + 1)
            blend = 10 * phase ** 3 - 15 * phase ** 4 + 6 * phase ** 5
            targets[first:last, foot, :2] = ((1 - blend[:, None]) * targets[first - 1, foot, :2]
                                            + blend[:, None] * targets[last, foot, :2])
            targets[first:last, foot, 2] = .0005 + clearance * 64 * phase ** 3 * (1 - phase) ** 3
            yaws[first:last, foot] = (1 - blend) * yaws[first - 1, foot] + blend * yaws[last, foot]
    return poses[begin:end].copy(), targets, yaws, contact, dict(
        original_frame_range_exclusive=[begin, end], source_phase_speed_threshold_m_s=.12,
        support_inference='Smoothed source foot planar speed, 3-frame morphology; no measured force labels.',
        source_support_intervals=[[(int(a), int(b)) for a, b in intervals(contact[:, j])] for j in range(2)],
        target_swing_peak_m=clearance + .0005, root_xy_and_orientation_preserved=True,
        non_leg_joint_angles_preserved=True,
        method='Source phase and plant-anchor preserving quintic XY swing; smooth vertical clearance correction')


def solve(model, poses, targets, yaws, limits, names):
    data = mujoco.MjData(model)
    geoms = [model.geom(side + '_foot_collision').id for side in ('left', 'right')]
    legs = [i for i, name in enumerate(names) if name.startswith(('left_hip_', 'right_hip_',
            'left_knee_', 'right_knee_', 'left_ankle_','right_ankle_'))]
    if len(legs) != 12:
        raise ValueError('Expected exactly twelve serial leg joints')
    qids = np.array([int(model.joint(names[i]).qposadr[0]) for i in legs])
    result = poses.copy()
    statuses = []
    for frame, prior in enumerate(poses):
        initial = np.r_[prior[2] - .008, prior[qids]]
        lower = np.r_[prior[2] - .06, limits[legs, 0] + 1e-5]
        upper = np.r_[prior[2] + .02, limits[legs, 1] - 1e-5]
        initial = np.clip(initial, lower + 1e-6, upper - 1e-6)
        desired_R = [Rotation.from_euler('z', yaws[frame, j]).as_matrix() for j in range(2)]

        def residual(x):
            data.qpos[:] = prior
            data.qpos[2] = x[0]
            data.qpos[qids] = x[1:]
            mujoco.mj_forward(model, data)
            terms = []
            for side, geom in enumerate(geoms):
                point, rotation = foot_state(model, data, geom)
                terms.extend(1000 * (point - targets[frame, side]))
                terms.extend(30 * Rotation.from_matrix(desired_R[side] @ rotation.T).as_rotvec())
            terms.extend(.5 * (x[1:] - prior[qids]))
            terms.append(15 * (x[0] - initial[0]))
            return np.asarray(terms)

        solution = least_squares(residual, initial, bounds=(lower, upper),
                                 xtol=1e-9, ftol=1e-9, gtol=1e-8, max_nfev=80)
        result[frame, 2] = solution.x[0]
        result[frame, qids] = solution.x[1:]
        statuses.append(solution.status)
        if frame % 100 == 0:
            print('PM01_FOOT_IK', frame, len(poses), 'cost', solution.cost, flush=True)
    return result, legs, statuses


def solve_fixed_height(model, poses, targets, yaws, limits, names, offset):
    """Keep the measured smooth root-Z shape and solve exact rigid-foot poses.

    A constant vertical offset removes the redundant root/leg-height degree
    of freedom. It does not itself create clearance: separate leg IK enforces
    the support anchors and swing targets. Final speed gates remain mandatory.
    """
    data=mujoco.MjData(model)
    legs=[i for i,n in enumerate(names) if n.startswith(('left_hip_','right_hip_','left_knee_',
          'right_knee_','left_ankle_','right_ankle_'))]
    qids=np.array([int(model.joint(names[i]).qposadr[0]) for i in legs])
    geoms=[model.geom(s+'_foot_collision').id for s in ('left','right')]
    result=poses.copy();result[:,2]-=offset;statuses=[]
    for t in range(len(poses)):
        initial=poses[t,qids].copy() if t==0 else result[t-1,qids].copy()
        desired_R=[Rotation.from_euler('z',yaws[t,j]).as_matrix() for j in range(2)]
        def residual(x):
            data.qpos[:]=result[t];data.qpos[qids]=x;mujoco.mj_forward(model,data)
            errors=[]
            for j,g in enumerate(geoms):
                target=targets[t,j].copy();target[2]+=model.geom_size[g,2]
                errors.extend(1000*(data.geom_xpos[g]-target))
                errors.extend(100*Rotation.from_matrix(desired_R[j]@data.geom_xmat[g].reshape(3,3).T).as_rotvec())
            return np.asarray(errors)
        lower=limits[legs,0]+1e-5;upper=limits[legs,1]-1e-5
        # Preserve the source's normal knee-flexion IK branch. Crossing the
        # straight-leg singularity can otherwise select hyperextension.
        for k,j in enumerate(legs):
            if names[j]=='left_knee_joint':
                lower[k]=max(lower[k],.02)
                if t==0:initial[k]=max(initial[k],.4)
            if names[j]=='right_knee_joint':
                upper[k]=min(upper[k],-.02)
                if t==0:initial[k]=min(initial[k],-.4)
        solved=least_squares(residual,np.clip(initial,lower+1e-7,upper-1e-7),bounds=(lower,upper),
            xtol=1e-10,ftol=1e-10,gtol=1e-9,max_nfev=100)
        result[t,qids]=solved.x;statuses.append(int(solved.status))
        if t%100==0:print('PM01_FIXED_HEIGHT_IK',t,len(poses),'cost',solved.cost,flush=True)
    return result,legs,statuses


def solve_trajectory(model, prior, targets, yaws, contact, limits, speeds, names, initial=None, root_acceleration_bound=7.9):
    """Whole-clip sparse SQP with unchanged hard position and speed bounds."""
    data = mujoco.MjData(model)
    legs = [i for i, n in enumerate(names) if n.startswith(('left_hip_', 'right_hip_',
            'left_knee_', 'right_knee_', 'left_ankle_', 'right_ankle_'))]
    qids = np.array([int(model.joint(names[i]).qposadr[0]) for i in legs])
    vids = np.r_[0, 1, 2, [int(model.joint(names[i]).dofadr[0]) for i in legs]]
    geoms = [model.geom(s + '_foot_collision').id for s in ('left', 'right')]
    current = prior.copy() if initial is None else initial.copy()
    if initial is None:
        current[:, 2] -= .008
    count, width = len(prior), len(vids)
    eye = np.eye(width)
    d1 = sparse.diags([-np.ones(count-1), np.ones(count-1)], [0,1], shape=(count-1,count), format='csc')
    d2 = sparse.diags([np.ones(count-2), -2*np.ones(count-2), np.ones(count-2)], [0,1,2], shape=(count-2,count), format='csc')
    speed_matrix = sparse.kron(d1, sparse.eye(width), format='csc')
    accel_matrix = sparse.kron(d2, sparse.csc_matrix(eye[:3]), format='csc')
    # Use a squared second-difference penalty with physical-time coefficients.
    d2_all = sparse.kron(d2, sparse.eye(width), format='csc')
    smoothing_weight = sparse.diags(np.tile(np.r_[np.full(3,2500**2*.001), np.full(width-3, 2500**2*1e-6)], count-2))
    smoothing_hessian = d2_all.T @ smoothing_weight @ d2_all
    desired_R = [[Rotation.from_euler('z', yaws[t,j]).as_matrix() for j in range(2)] for t in range(count)]
    statuses = []
    def merit(poses):
        value=.125*np.square(poses[:,qids]-prior[:,qids]).sum()+112.5*np.square(poses[:,:3]-(prior[:,:3]-np.array([0,0,.008]))).sum()
        vector=np.c_[poses[:,:3],poses[:,qids]].ravel()
        value+=.5*float(vector@(smoothing_hessian@vector))
        for t in range(count):
            data.qpos[:]=poses[t];mujoco.mj_forward(model,data)
            for j,g in enumerate(geoms):
                desired=targets[t,j]
                R=data.geom_xmat[g].reshape(3,3)
                measured,_=foot_state(model,data,g)
                value+=(.5e8 if contact[t,j] else .5e6)*np.square(desired-measured).sum()
                value+=.5e3*np.square(Rotation.from_matrix(desired_R[t][j]@R.T).as_rotvec()).sum()
                if contact[t,j]:value+=1e8*np.square(np.maximum(0,np.abs(measured-desired)-.0005)).sum()
                for sx in (-1,1):
                    for sy in (-1,1):
                        z=(data.geom_xpos[g]+R@(model.geom_size[g]*np.array([sx,sy,-1])))[2]
                        value+=1e8*max(0,max(-.0001,targets[t,j,2]-.0015)-z)**2
        return value
    previous_merit=merit(current)
    for iteration in range(30):
        hessians, gradients, blocks, lowers, uppers = [], [], [], [], []
        for t in range(count):
            data.qpos[:] = current[t]
            mujoco.mj_forward(model, data)
            jacobians, errors, constraints, lo, hi = [], [], [], [], []
            def cost(jac, residual, weight):
                jacobians.append(np.sqrt(weight)*jac)
                errors.append(np.sqrt(weight)*np.atleast_1d(residual))
            cost(eye[3:], prior[t,qids]-current[t,qids], .25)
            cost(eye[:3], prior[t,:3]-np.array([0,0,.008])-current[t,:3], 225)
            for j,g in enumerate(geoms):
                jp=np.zeros((3,model.nv));jr=np.zeros_like(jp)
                mujoco.mj_jacGeom(model,data,jp,jr,g)
                R=data.geom_xmat[g].reshape(3,3)
                measured,_=foot_state(model,data,g)
                bottom=data.geom_xpos[g]-R@(np.sign(R[2])*model.geom_size[g])
                jb=np.zeros((3,model.nv));rb=np.zeros_like(jb)
                mujoco.mj_jac(model,data,jb,rb,bottom,int(model.geom_bodyid[g]))
                foot_jac=np.vstack((jp[:2,vids],jb[2:3,vids]))
                cost(foot_jac,targets[t,j]-measured,1e8 if contact[t,j] else 1e6)
                cost(jr[:,vids],Rotation.from_matrix(desired_R[t][j]@R.T).as_rotvec(),1e3)
                if contact[t,j]:
                    constraints.append(foot_jac)
                    lo.append(targets[t,j]-measured-.0005);hi.append(targets[t,j]-measured+.0005)
                for sx in (-1,1):
                    for sy in (-1,1):
                        point=data.geom_xpos[g]+R@(model.geom_size[g]*np.array([sx,sy,-1]))
                        jc=np.zeros((3,model.nv));rc=np.zeros_like(jc)
                        mujoco.mj_jac(model,data,jc,rc,point,int(model.geom_bodyid[g]))
                        constraints.append(jc[2:3,vids])
                        lo.append(np.array([max(-.0001,targets[t,j,2]-.0015)-point[2]]))
                        hi.append(np.array([np.inf]))
            lower=np.r_[np.full(3,-.02),np.full(width-3,-.15)]
            upper=-lower
            lower[:3]=np.maximum(lower[:3],prior[t,:3]-np.array([.02,.02,.06])-current[t,:3])
            upper[:3]=np.minimum(upper[:3],prior[t,:3]+np.array([.02,.02,.02])-current[t,:3])
            lower[3:]=np.maximum(lower[3:],limits[legs,0]+1e-5-current[t,qids])
            upper[3:]=np.minimum(upper[3:],limits[legs,1]-1e-5-current[t,qids])
            for k,joint in enumerate(legs):
                if names[joint]=='left_knee_joint':lower[3+k]=max(lower[3+k],.02-current[t,qids[k]])
                if names[joint]=='right_knee_joint':upper[3+k]=min(upper[3+k],-.02-current[t,qids[k]])
            constraints.append(eye);lo.append(lower);hi.append(upper)
            J=np.concatenate(jacobians);e=np.concatenate(errors)
            hessians.append(sparse.csc_matrix(J.T@J+1e-3*eye));gradients.append(-J.T@e)
            blocks.append(sparse.csc_matrix(np.concatenate(constraints)));lowers.append(np.concatenate(lo));uppers.append(np.concatenate(hi))
        values=np.c_[current[:,:3],current[:,qids]].ravel()
        delta_speed=speed_matrix@values
        speed_bound=np.tile(np.r_[.98,.98,.4,.98*speeds[legs]]/50,count-1)
        delta_accel=accel_matrix@values
        accel_bound=np.full_like(delta_accel,root_acceleration_bound/2500)
        P=sparse.block_diag(hessians,format='csc')+smoothing_hessian
        q=np.concatenate(gradients)+smoothing_hessian@values
        A=sparse.vstack([sparse.block_diag(blocks),speed_matrix,accel_matrix],format='csc')
        lower=np.r_[np.concatenate(lowers),-speed_bound-delta_speed,-accel_bound-delta_accel]
        upper=np.r_[np.concatenate(uppers),speed_bound-delta_speed,accel_bound-delta_accel]
        qp=osqp.OSQP();qp.setup(P=sparse.triu(P/1e6).tocsc(),q=q/1e6,A=A,l=lower,u=upper,
            verbose=False,eps_abs=1e-5,eps_rel=1e-5,max_iter=100000,polishing=True)
        solved=qp.solve();statuses.append(solved.info.status)
        if solved.info.status_val not in (1,2):
            finite=solved.x is not None and np.isfinite(solved.x).all()
            actual=A@solved.x if finite else np.full_like(lower,np.nan)
            violation=float(np.maximum(lower-actual,actual-upper).clip(0).max()) if finite else float('inf')
            print('QP_DIAGNOSTIC','status',solved.info.status,'primal',solved.info.prim_res,
                  'dual',solved.info.dual_res,'unscaled_constraint_violation',violation,flush=True)
            # A bounded, feasible SQP step is usable without claiming QP
            # objective convergence. Independent nonlinear physical gates still
            # decide candidate acceptance; their thresholds do not change.
            if solved.info.status_val!=7 or violation>1e-5:
                raise RuntimeError(f'Trajectory QP failed at {iteration}: {solved.info.status}')
        delta=solved.x.reshape(count,width)
        alpha=1.
        for _ in range(12):
            candidate=current.copy()
            candidate[:,:3]+=alpha*delta[:,:3];candidate[:,qids]+=alpha*delta[:,3:]
            candidate_merit=merit(candidate)
            # The first iterate restores linear speed/acceleration feasibility.
            if iteration==0 or candidate_merit<previous_merit+1e-8:break
            alpha*=.5
        if candidate_merit>previous_merit+1e-8 and iteration>0:
            print('PM01_TRAJECTORY_LINE_SEARCH_STOP',iteration,flush=True)
            break
        current=candidate;previous_merit=candidate_merit;delta*=alpha
        print('PM01_TRAJECTORY_SQP',iteration,'maximum_delta',float(np.abs(delta).max()),
              'solver',solved.info.status,'alpha',alpha,'merit',candidate_merit,flush=True)
        if iteration>=3:
            check,_=assess(model,current,prior,targets,contact,names,limits,speeds,legs)
            print('PM01_TRAJECTORY_GATES',json.dumps(check['gates']),flush=True)
            if check['numeric_candidate_passed'] and np.abs(delta).max()<.002:
                break
        if np.abs(delta).max()<2e-5:
            break
    return current,legs,statuses


def assess(model, poses, prior, targets, contact, names, limits, speeds, legs):
    data = mujoco.MjData(model)
    geoms = [model.geom(side + '_foot_collision').id for side in ('left', 'right')]
    actual = []
    yaw = []
    for pose in poses:
        data.qpos[:] = pose
        mujoco.mj_forward(model, data)
        values = [foot_state(model, data, g) for g in geoms]
        actual.append([v[0] for v in values])
        yaw.append([np.arctan2(v[1][1, 0], v[1][0, 0]) for v in values])
    actual = np.asarray(actual)
    stable = contact[:-1] & contact[1:]
    velocity = np.linalg.norm(np.diff(actual[..., :2], axis=0) * 50, axis=-1)
    qids = [int(model.joint(n).qposadr[0]) for n in names]
    q = poses[:, qids]
    derivatives = kinematics(model, poses)
    ratio = float((np.abs(derivatives['joint_vel']) / speeds).max())
    adjacent_ratio = float((np.abs(np.diff(q, axis=0)) * 50 / speeds).max())
    upper = [i for i in range(len(names)) if i not in legs]
    plant_error = float(np.linalg.norm(actual - targets, axis=-1)[contact].max())
    slip = [float(np.sqrt(np.mean(velocity[:, j][stable[:, j]] ** 2))) for j in range(2)]
    peaks = [[float(actual[a:b, j, 2].max()) for a, b in intervals(~contact[:, j])] for j in range(2)]
    geometry = geometry_quality(model, poses)
    upper_change = float(np.abs(q[:, upper] - prior[:, np.array(qids)[upper]]).max())
    excess = float(np.maximum(limits[:, 0] - q, q - limits[:, 1]).clip(0).max())
    root_accel = np.abs(np.diff(poses[:, :3], n=2, axis=0) * 2500).max(0)
    gates = dict(hard_limits=excess <= 1e-6, joint_speed=max(ratio, adjacent_ratio) <= 1.,
                 support_residual=plant_error <= .002, support_slip=max(slip) <= .08,
                 bilateral_clearance=all(len(x) >= 2 and min(x) >= .02 for x in peaks),
                 geometry=geometry['minimum_collision_z_m'] >= -.002 and geometry['self_minimum_signed_distance_m'] >= -.01,
                 upper_body_preserved=upper_change < 1e-7, root_acceleration=bool((root_accel <= 8).all()))
    return dict(numeric_candidate_passed=all(gates.values()), gates=gates,
                accepted_for_training=False, visual_review_passed=False,
                maximum_plant_error_m=plant_error, support_foot_rms_speed_m_s=slip,
                swing_peak_height_per_step_m=peaks, max_joint_speed_ratio=ratio,
                max_adjacent_joint_speed_ratio=adjacent_ratio, max_joint_limit_excess_rad=excess,
                maximum_non_leg_joint_change_rad=upper_change, root_acceleration_peak_m_s2=root_accel.tolist(),
                minimum_sole_height_m=actual[..., 2].min(0).tolist(), geometry=geometry,
                qualification='Kinematic candidate only; physical tracking and velocity gait are separate gates'), derivatives


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--clearance', type=float, default=.025)
    p.add_argument('--trajectory', action='store_true')
    p.add_argument('--warm-start', type=Path)
    p.add_argument('--fixed-height-offset',type=float)
    p.add_argument('--root-acceleration-bound',type=float,default=7.9)
    args = p.parse_args()
    if not .02 <= args.clearance <= .04:
        raise ValueError('Research candidate swing peak must be in the declared 20-40mm range')
    if not .5<=args.root_acceleration_bound<=7.9:
        raise ValueError('Root acceleration bound must stay within the existing 8m/s2 gate')
    args.output.mkdir(parents=True, exist_ok=False)
    with np.load(args.source, allow_pickle=False) as archive:
        source = {key: archive[key] for key in archive.files}
    if float(source['fps']) != 50:
        raise ValueError('Expected the audited 50Hz PM01 input')
    model, names, limits, speeds = robot_contract()
    prior, targets, yaws, contact, provenance = build_targets(model, source['qpos'].astype(float), args.clearance)
    if args.fixed_height_offset is not None:
        if args.trajectory or not .005<=args.fixed_height_offset<=.04:
            raise ValueError('Fixed-height candidate must be a separate bounded 5-40mm experiment')
        result,legs,statuses=solve_fixed_height(model,prior,targets,yaws,limits,names,args.fixed_height_offset)
    elif args.trajectory:
        initial=None
        if args.warm_start is not None:
            with np.load(args.warm_start,allow_pickle=False) as archive:initial=archive['qpos']
            if initial.shape!=prior.shape:raise ValueError('Warm-start frame mismatch')
        result,legs,statuses=solve_trajectory(model,prior,targets,yaws,contact,limits,speeds,names,initial,args.root_acceleration_bound)
        provenance['root_xy_and_orientation_preserved']=False
        provenance['root_orientation_preserved']=True
        provenance['root_xy_translation_bound_m']=.02
    else:
        result, legs, statuses = solve(model, prior, targets, yaws, limits, names)
    report, fields = assess(model, result, prior, targets, contact, names, limits, speeds, legs)
    # Recompute all derivatives and FK in model-native named order.
    bank = args.output/'pm01_clearance_v1.npz'
    np.savez_compressed(bank, **fields, qpos=result, fps=np.array(50.),
                        joint_names=np.array(names), body_names=np.array([model.body(i).name for i in range(1, model.nbody)]),
                        clip_starts=np.array([0]), clip_lengths=np.array([len(result)]),
                        clip_names=np.array(['pm01_contact_clearance_v1']),
                        asset_revision=np.array(ASSET_REVISION), retarget_method=np.array('pm01_phase_contact_clearance_v1'))
    np.savez_compressed(args.output/'targets.npz', targets=targets, contact=contact, yaw=yaws, original_qpos=prior)
    report.update(asset_revision=ASSET_REVISION, motion_bank_sha256=hashlib.sha256(bank.read_bytes()).hexdigest(),
                  source_sha256=hashlib.sha256(args.source.read_bytes()).hexdigest(),
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  solver_statuses=statuses, trajectory_constraints_enabled=args.trajectory,
                  fixed_root_height_offset_m=args.fixed_height_offset,
                  solver_root_acceleration_bound_m_s2=args.root_acceleration_bound, **provenance)
    (args.output/'audit.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
