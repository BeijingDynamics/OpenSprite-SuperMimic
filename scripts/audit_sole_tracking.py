"""Audit sole clearance in an immutable reference and physical policy rollouts.

Measured vertical contact force selects loaded intervals. Near-floor material
points are selected by height, never by speed. This is an interval estimate,
not an assertion that a particular corner is the solver's contact point.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from opensprite_tracker.robot import BODY_JOINT_NAMES, ROBOT_XML


def load(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key] for key in archive.files}


def reference_poses(ref):
    """Read either native clips or packed banks without assuming body order."""
    names = ref['joint_names'].astype(str).tolist()
    joint = ref['joint_pos'][:, [names.index(name) for name in BODY_JOINT_NAMES]]
    if 'qpos' in ref:
        root = ref['qpos'][:, :7]
    else:
        pelvis = ref['body_names'].astype(str).tolist().index('pelvis_link')
        root = np.concatenate((ref['body_pos_w'][:, pelvis], ref['body_quat_w'][:, pelvis]), axis=-1)
    return np.concatenate((root, joint), axis=-1)


def geometry(poses):
    model = mujoco.MjModel.from_xml_path(str(ROBOT_XML))
    data = mujoco.MjData(model)
    joint_ids = [int(model.joint(name).qposadr[0]) for name in BODY_JOINT_NAMES]
    feet = [model.geom(side + '_foot_collision').id for side in ('left', 'right')]
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    points = []
    for pose in poses:
        data.qpos[:7] = pose[:7]
        data.qpos[joint_ids] = pose[7:]
        mujoco.mj_forward(model, data)
        points.append([(signs * model.geom_size[g]) @ data.geom_xmat[g].reshape(3, 3).T
                       + data.geom_xpos[g] for g in feet])
    return np.asarray(points)


def measure(points, fps, forces=None, clearance_m=.005, floor_band_m=.002):
    """Report both feet; thresholds are diagnostic resolution, not a gait pass."""
    heights = points[..., 2].min(-1)
    events = []
    for foot in range(2):
        above = heights[:, foot] > clearance_m
        edges = np.diff(np.r_[False, above, False].astype(int))
        intervals = [(a, b) for a, b in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))
                     if b - a >= 2]
        events.append([dict(begin_seconds=a / fps, duration_seconds=(b - a) / fps,
                            peak_m=float(heights[a:b, foot].max())) for a, b in intervals])
    relative = heights - heights.min(-1, keepdims=True)
    output = dict(samples=len(points), sole_height_min_m=heights.min(0).tolist(),
                  sole_height_max_m=heights.max(0).tolist(),
                  sole_height_p95_m=np.quantile(heights, .95, axis=0).tolist(),
                  height_above_lower_foot_max_m=relative.max(0).tolist(),
                  height_above_lower_foot_p95_m=np.quantile(relative, .95, axis=0).tolist(),
                  clearance_event_threshold_m=clearance_m, clearance_events=events,
                  clearance_event_counts=[len(x) for x in events])
    if forces is not None:
        loaded = (forces[:-1] > 5) & (forces[1:] > 5)
        near_floor = (points[:-1, ..., 2] <= floor_band_m) & (points[1:, ..., 2] <= floor_band_m)
        speed = np.linalg.norm(np.diff(points[..., :2], axis=0) * fps, axis=-1)
        mask = loaded[..., None] & near_floor
        output.update(loaded_interval_fraction=loaded.mean(0).tolist(),
                      loaded_near_floor_corner_samples=mask.sum((0, 2)).tolist(),
                      loaded_near_floor_corner_rms_speed_m_s=[
                          float(np.sqrt(np.mean(speed[:, j][mask[:, j]] ** 2))) if mask[:, j].any() else None
                          for j in range(2)])
    return output


def physical_trace(path, env, start_seconds, active_steps=None):
    trace = load(path)
    fps = float(trace['fps'])
    failed = np.flatnonzero(trace['failed'][:, env])
    end = int(failed[0]) if len(failed) else len(trace['qpos'])
    if active_steps is not None:
        end = min(end, active_steps)
    start = round(start_seconds * fps)
    if end <= start + 1:
        return dict(source=str(path), env=env, scoring_window_empty=True,
                    failed_before_scoring=bool(len(failed) and failed[0] <= start),
                    active_steps=active_steps)
    result = measure(geometry(trace['qpos'][start:end, env]), fps,
                     trace['contact_force_mean_n'][start:end, env])
    result.update(source=str(path), env=env, scoring_start_seconds=start / fps,
                  scoring_end_seconds=end / fps,
                  survived=not any(failed < (active_steps if active_steps is not None else len(trace['qpos']))))
    return result


def tracking_reward_diagnostic(motion_path, rollout_path, env=0):
    """Reconstruct arrived-at states from the next recorded proprioception.

    The last state is excluded because its next observation was not recorded.
    The analytical foot-Z counterfactual changes reward inputs only; it is not
    a physically realizable pose or a rollout prediction.
    """
    import torch
    from opensprite_tracker.supermimic.robot import make_kinematics, default_joint_pos, velocity_limits
    from opensprite_tracker.supermimic.motion import MotionLib
    from opensprite_tracker.supermimic.rotations import quat_to_mat, yaw_mat
    from opensprite_tracker.supermimic.sole_tracking import make_motion_mdp
    torch.set_num_threads(2)
    kin = make_kinematics('cpu', True)
    motion = MotionLib(motion_path, kin, 'cpu')
    summary_path = Path(rollout_path).parent / 'summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    mdp = make_motion_mdp(summary, kin, motion, default_joint_pos(), velocity_limits())
    z = load(rollout_path)
    start, length = int(motion.clip_start[env]), int(motion.clip_len[env])
    # An environment stops being a motion-tracking case at its own clip end,
    # even if other, longer clips keep the shared rollout running.
    count = min(len(z['qpos']) - 1, length - 1)
    if count < 1:
        return dict(valid_samples=0)
    tensor = lambda x: torch.tensor(x, dtype=torch.float32)
    qpos = tensor(z['qpos'][:count, env])
    obs_next = tensor(z['proprioception'][1:count + 1, env])
    if not torch.allclose(obs_next[:, :31] + default_joint_pos(), qpos[:, 7:], atol=1e-5, rtol=0):
        raise ValueError('The next observation does not describe the logged arrived-at joint state')
    if not np.allclose(z['proprioception'][1:count + 1, env, 68:99], z['policy_action'][:count, env], atol=1e-6, rtol=0):
        raise ValueError('Previous-action history is not aligned with the logged raw policy action')
    rotation = quat_to_mat(qpos[:, 3:7])
    velocity = (yaw_mat(rotation) @ tensor(z['velocity'][:count, env])[..., None]).squeeze(-1)
    omega = (rotation @ obs_next[:, 62:65, None]).squeeze(-1)
    state = (qpos[:, :3], rotation, velocity, omega, qpos[:, 7:], obs_next[:, 31:62])
    elapsed = torch.arange(1, count + 1)
    frame = start + elapsed
    actions = tensor(z['policy_action'][:count, env])
    previous = tensor(z['proprioception'][:count, env, 68:99])
    reward, terms = mdp.reward(state, frame, actions, previous, terms=True)
    bp, bR, _, _, tp, tR = mdp._bodies_and_target(state, frame)
    actual_z = kin.sole_bottom_z(bp[:, mdp.feet], bR[:, mdp.feet])
    target_z = kin.sole_bottom_z(tp[:, mdp.feet], tR[:, mdp.feet])
    epos = (tp - bp).square().sum(-1).mean(-1)
    foot_z_part = (tp[:, mdp.feet, 2] - bp[:, mdp.feet, 2]).square().sum(-1) / bp.shape[1]
    benefit = torch.exp(-(epos - foot_z_part) / .3 ** 2) - torch.exp(-epos / .3 ** 2)
    valid = (~tensor(z['failed'][:count, env]).bool()) & (elapsed >= 150)
    if not valid.any():
        return dict(valid_samples=0)
    return dict(valid_samples=int(valid.sum()),
                normalized_tracking_reward_mean=float(reward[valid].mean() / mdp.reward_scale),
                body_position_rmse_m=float(terms['body_error_m'][valid].mean()),
                sole_height_rmse_m=((actual_z - target_z)[valid].square().mean(0).sqrt()).tolist(),
                mean_raw_tracking_term=float(terms['per_reward/tracking'][valid].mean()),
                mean_raw_penalty=float(-terms['per_reward/penalties'][valid].mean()),
                mean_raw_body_position_reward_gain_if_foot_origin_z_errors_removed=float(benefit[valid].mean()),
                reference_floor_offset_warning='Audit absolute reference support heights before adding any absolute sole-height reward; floating support targets can encourage hopping.',
                limits='No contact-state claim from the reference; no causal claim from a reward-only counterfactual.')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--motion', type=Path, required=True)
    parser.add_argument('--motion-rollout', type=Path, required=True)
    parser.add_argument('--velocity-rollout', type=Path, required=True)
    parser.add_argument('--velocity-env', type=int, default=13)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    ref = load(args.motion)
    poses = reference_poses(ref)
    from opensprite_tracker.supermimic.motion import _clip_ranges
    ranges = _clip_ranges(ref, len(poses))
    clip_names = ref.get('clip_names', ref.get('motion_names', np.array([f'clip_{i}' for i in range(len(ranges))]))).astype(str).tolist()
    if len(clip_names) != len(ranges):
        raise ValueError('Reference clip names and boundaries differ')
    rollout = load(args.motion_rollout)
    if rollout['qpos'].shape[1] != len(ranges):
        raise ValueError('Motion rollout must contain one environment per reference clip')
    clips = []
    for env, ((start, length), name) in enumerate(zip(ranges, clip_names)):
        clips.append(dict(name=name, env=env, reference_start=start, reference_length=length,
                          reference=measure(geometry(poses[start:start + length]), float(ref['fps'])),
                          motion_tracking=physical_trace(args.motion_rollout, env, 3., active_steps=length - 1),
                          tracking_reward=tracking_reward_diagnostic(args.motion, args.motion_rollout, env)))
    report = dict(schema='sole_tracking_per_clip_v2', clips=clips,
                  velocity_tracking=physical_trace(args.velocity_rollout, args.velocity_env, 3.),
                  hashes={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                          (args.motion, args.motion_rollout, args.velocity_rollout)},
                  physical_contact_metric='Mean vertical force >5N at both endpoints; identical box material corner <=2mm above floor at both endpoints. Rolling and 50Hz sampling remain limitations.',
                  assessment='Low-clearance evidence; not a hardware admission or a trained repair.')
    if len(clips) == 1:
        # Preserve the established single-reference report fields.
        report.update({key: clips[0][key] for key in ('reference', 'motion_tracking', 'tracking_reward')})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({key: value for key, value in report.items() if key != 'reference'}, indent=2))


if __name__ == '__main__':
    main()
