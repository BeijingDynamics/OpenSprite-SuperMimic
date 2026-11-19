# IMU processing comparison with AMP runtime main

Compared current SuperMimic runtime source with GitHub
`BeijingDynamics/open_sprite_runtime` main, observed head
`4515206c24404e18a2d5c168eccdb7468aea6ab7`, on November 16, 2026.
Downloaded source evidence is in
`outputs/deploy-20261116/imu-main-comparison/`.

After normalizing line endings and trailing whitespace, these files match:

- `native/src/imu_six_axis.hpp`: stationary startup bias and attitude filter.
- `native/icm45686/core.hpp`: sensor configuration, units and mounting axes.
- `native/icm45686/ipc.hpp`: sensor IPC server.
- `native/src/icm_packet.hpp`: ICM packet encoding/frame representation.
- `native/src/imu_packet.hpp`: packet utilities.
- `native/src/policy_imu_client.hpp`: packet validation and receiving thread.
- `native/src/policy_feedback.hpp`: motor/IMU conversion into policy inputs.

`native/icm45686/main.cpp` differs only by the additional SPI0 performance-policy
block: require or set runtime power control to `on`, retain it on exit, and
report its state. It does not modify sampling, bias, filter equations, or units.
The real second Xbox30 run's IMU log confirms 800 Hz capture, 15-second startup
calibration, and this persistent SPI performance setting.

Both paths use 4 g and 2000 deg/s sensor ranges, gyro converted to rad/s, and
body = (-sensorZ, -sensorY, -sensorX). Startup estimates frozen gyro bias;
initial roll/pitch comes from measured gravity, not a forced upright pose.
The quaternion propagates on sensor timestamps, with a 1/s proportional
accelerometer correction weighted by acceleration magnitude. Neither path uses
magnetic heading or online gyro-bias learning. The policy receives corrected
body angular velocity and unit projected gravity, not raw accelerometer data.

Both IMU clients request the latest fused packet every 5 ms, while capture and
fusion remain 800 Hz. The 50 Hz actor consumes the latest available snapshot.
Thus neither path runs the attitude integrator at policy frequency. Policy
history differs by design: AMP uses 8 frames/795 dimensions; SuperMimic uses
10 frames/993 dimensions. This does not change the IMU fusion equations.

This source audit does not prove identical end-to-end sensor age under different
actor workloads, nor establish external attitude ground truth. The larger
gyro-only versus fused discrepancy in the second real trial is not evidence of
a newly introduced fusion algorithm. Follow-up should compare motion-dependent
sensor effects and packet age, retaining the current filter until supported
by a controlled experiment. No hardware access or parameter change occurred.
