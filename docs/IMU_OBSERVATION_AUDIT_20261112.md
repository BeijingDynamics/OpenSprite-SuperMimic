# IMU observation and estimation audit

## Operator confirmation and unit contract

On November 12 the operator confirmed the sensor is rigidly mounted on the
pelvis. Production training now records verified `imu_frame=root`. No moving
waist transform is needed for this installation; retain the existing calibrated
sensor-to-pelvis rotation. Historical smoke and benchmark records are unchanged.

The register configuration selects a gyro range of +/-2000 deg/s and a 16-bit
signed register readout. The conversion is
`omega_sensor_rad_s = counts * (2000 / 32768) * (pi / 180)`.
Subtract the startup gyro bias in the same units and frame, then apply the
sensor-to-body matrix. For the current installation it maps `[x,y,z]` to
`[-z,-y,-x]`; this orthogonal rotation has no amplitude gain. Policy angular
velocity is rad/s, not deg/s and not angular displacement per 20 ms step.
Do not multiply it by policy dt, the 800/50 rate ratio, or an inherited 0.25
observation scale. This policy's learned RunningNorm is embedded in the exported
graph; the native input adapter supplies physical units without that extra scale.

The fusion estimator consumes acceleration in m/s^2 and gyro in rad/s with
timestamp-derived dt in seconds. Its quaternion is body-to-world, wxyz.
The policy receives `R_body_to_world.T @ [0,0,-1]`, a dimensionless unit vector
that is `[0,0,-1]` when upright. IPC also contains a gravity channel scaled by
9.80665, but the policy feedback adapter reconstructs the unit vector from the
quaternion rather than feeding that acceleration-valued channel to the actor.
The tilt-correction term used inside fusion is not added to the gyro observation.

Hardware-free verification on the rented server passed on November 12:
`icm_offline`, `icm_packet`, and `test_imu_six_axis`. These exercise configured
register ranges, SI conversion (16,384 gyro counts = 1,000 deg/s =
17.45329252 rad/s), mounting, one-second gyro integration at 400/800 Hz,
200 quaternion orientations through the packet/policy path, bias correction,
tilted startup and timestamp rejection. This is software-contract evidence,
not a fresh physical calibration measurement.

## What the policy receives

SuperMimic Appendix A, Table A.1 specifies joint position offsets, joint
velocities, base-frame angular velocity, projected gravity, and previous action,
with ten observations of history. Raw accelerometer specific force is not an
actor observation. The published G1 has 29 joints; our Sprite interface has 31,
giving 99 proprioceptive features per frame and 993 velocity-policy inputs.

For a body-to-world rotation R, projected gravity is R-transpose times
`[0, 0, -1]`. It describes tilt, not an absolute compass heading. Do not substitute
normalized instantaneous accelerometer output during locomotion: specific force
also contains translational/contact acceleration. The policy uses an MLP over
finite history, not a persistent attitude integrator. The differentiable world
model integrates predicted dynamics during training; it is not an IMU estimator
running inside the deployed velocity actor.

The paper specifies these observation semantics and sensor noise, but does not
specify the ICM-45686 estimator, its sample rate, or its filtering parameters.
Those are embodiment-specific implementation responsibilities. C. Zhang's
`supermimic/mdp.py::MDP.obs` computes body gyro and projected gravity from simulator
root state. Official `src/supermimic/representations.py` likewise uses base
angular velocity and projected gravity in the proprioceptive interface.

## Inspected local deployment path

The selected mixed-rate runtime's local source configures ICM-45686 acquisition
at 800 Hz. `native/icm45686/main.cpp` runs the C++ six-axis estimator independently
of policy inference. Each accepted DRDY sample supplies a sensor timestamp;
`SixAxis::update` propagates orientation using the measured time difference,
subtracts a startup gyro bias, and applies an accelerometer-based tilt correction.
It is a nonlinear complementary filter, not an EKF and not a learned estimator.

The startup window lasts at least 15 seconds. Initialization derives roll and
pitch from the stationary measured acceleration; it does not redefine the
current tilted pose as level. Yaw starts from an arbitrary zero without a
magnetometer. The policy adapter consumes corrected gyro and the estimated
quaternion, transforms them to the configured body frame, and constructs unit
projected gravity. Accelerometer readings are carried in IPC for validation but
are not passed as a raw acceleration channel into the policy.

The 50 Hz policy reads the latest IMU snapshot alongside motor feedback, then
updates its ten-frame observation history. High-rate orientation estimation
continues between policy updates. These are source-code findings, not a new
measurement of the live Jetson or a claim of lossless 800 Hz acquisition.
The register-based reader can coalesce/skip late DRDY events and is not FIFO
capture. Sequence continuity and accepted-sample intervals therefore matter.

## Remaining checks before new-policy qualification

- Physical pelvis mounting is confirmed. Preserve the sensor-axis/sign and unit
  checks when rebuilding or changing the IMU data path.
- Replay recorded raw accelerometer/gyro through the exact estimator, comparing
  reconstructed attitude and projected gravity with logged policy inputs.
- Quantify accepted-sample gaps, sensor-to-observation age, IMU/motor timestamp
  skew and observation-to-action latency, including tails and contact events.
- Inspect dynamic tilt error. The current correction trust uses acceleration
  magnitude; horizontal acceleration can change its direction while barely
  changing its norm. This is a plausible estimator limitation, not an established
  explanation for the observed foot dragging or body twist.
- Compare simulator-derived ideal gravity plus independent observation noise
  against estimator-filtered observations under the same motion. The current
  training does not explicitly reproduce the high-rate complementary filter,
  its correlated errors or sampling delays. Add measured delay/noise effects
  only after quantifying them; do not change hardware filters speculatively.

No IMU calibration, mounting transform, control frequency, motor gain or
protection threshold was changed by this audit. No hardware process was started.

## Sources

- Paper Appendix A / Table A.1: https://arxiv.org/pdf/2610.28480#page=13
- C. Zhang: https://github.com/zita-ch/supermimic_vibed/blob/2a26b2f8c4d616e16361a616fe659c7d4acdf317/supermimic/mdp.py
- Official representation: https://github.com/HybridRobotics/supermimic/blob/abc124c1bd5976341034668af3d12c087f2f1f55/src/supermimic/representations.py
- Local selected-runtime source: `../OpenSprite-SuperMimic/runtime/native/icm45686/main.cpp`,
  `native/src/imu_six_axis.hpp`, `native/src/policy_feedback.hpp`, and
  `native/src/sprite_supermimic_replay.cpp` within that runtime.
