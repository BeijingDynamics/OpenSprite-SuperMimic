"""Versioned training semantics; tensor dimensions alone are insufficient."""

V1_SCHEMA = 'sprite_v5_joint_supermimic_v1'
V2_SCHEMA = 'sprite_v5_joint_supermimic_v2'


def revision_contract(config):
    revision = config.get('algorithm_revision', 'legacy_v1')
    if revision == 'legacy_v1':
        return dict(schema=V1_SCHEMA, dynamics_mode='delta_v1',
                    goal_mode='reference_v1', goal_stride=1,
                    normalized_gelu=False, imu_frame='root',
                    critic_relative_positions=False, critic_upper_bound=False,
                    wm_warmup_iterations=0)
    if revision != 'cz_official_aligned_rl_v2':
        raise ValueError('Unknown joint algorithm revision: ' + revision)
    frame = config.get('imu_frame')
    if frame not in ('root', 'imu_site'):
        raise ValueError('V2 requires an explicit verified IMU frame')
    expected = dict(dynamics_mode='acceleration_v2', goal_mode='robot_relative_v2',
                    goal_stride=5, normalized_gelu=True,
                    critic_relative_positions=True, critic_upper_bound=True,
                    wm_warmup_iterations=500)
    for key, value in expected.items():
        if config.get(key) != value:
            raise ValueError('Incomplete v2 contract: ' + key)
    return dict(schema=V2_SCHEMA, imu_frame=frame, **expected)


def motion_goal(motion, frame, state, count=10):
    if getattr(motion, 'state_dependent_goal', False):
        return motion.goal(frame, count, state=state)
    return motion.goal(frame, count)
