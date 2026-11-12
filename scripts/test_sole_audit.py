"""Synthetic independent checks for the physical sole audit."""
import numpy as np
from audit_sole_tracking import measure


def run():
    frames = 30
    points = np.zeros((frames, 2, 8, 3))
    forces = np.full((frames, 2), 20.)
    static = measure(points, 50, forces)
    assert static['clearance_event_counts'] == [0, 0]
    assert static['loaded_near_floor_corner_rms_speed_m_s'] == [0., 0.]
    sliding = points.copy()
    sliding[..., 0] = np.arange(frames)[:, None, None] * .2 / 50
    report = measure(sliding, 50, forces)
    assert np.allclose(report['loaded_near_floor_corner_rms_speed_m_s'], [.2, .2])
    assert report['clearance_event_counts'] == [0, 0]
    swinging = points.copy()
    swinging[5:10, 0, :, 2] = .015
    swinging[16:21, 1, :, 2] = .015
    report = measure(swinging, 50)
    assert report['clearance_event_counts'] == [1, 1]
    # One-frame sensor/geometry spikes must not become repeated stepping.
    swinging = points.copy()
    swinging[5, 0, :, 2] = .1
    assert measure(swinging, 50)['clearance_event_counts'] == [0, 0]
    # An unloaded translating foot does not count as loaded slip.
    report = measure(sliding, 50, np.zeros_like(forces))
    assert report['loaded_near_floor_corner_rms_speed_m_s'] == [None, None]
    # High corners are excluded geometrically, never by selecting slow points.
    rolling = points.copy()
    rolling[..., 4:, 2] = .05
    rolling[..., 4:, 0] = np.arange(frames)[:, None, None] / 50
    assert measure(rolling, 50, forces)['loaded_near_floor_corner_rms_speed_m_s'] == [0., 0.]
    floating = points.copy()
    floating[..., 2] = .01
    report = measure(floating, 50)
    assert report['sole_height_max_m'] == [.01, .01]
    assert report['height_above_lower_foot_max_m'] == [0., 0.]
    print('SOLE_AUDIT_SEVEN_SYNTHETIC_CASES_PASSED')


if __name__ == '__main__':
    run()
