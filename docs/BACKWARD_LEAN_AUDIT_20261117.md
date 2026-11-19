# Backward lean after CPU-frequency stabilization

## Scope and operator observations

Read-only investigation of the November 17 powered Xbox trials. No hardware
frames, motor enables, runtime changes, gain changes, or protection changes
were made during this investigation. Full evidence was copied locally to
`outputs/cpufreq-powered-20261117/`.

The operator reports that the sling stayed slack, there was no pushing while
stationary, and push tests occurred during motion. Backward lean was strongest
at zero command. Forward commands reduced it, but sometimes the robot traveled
backward despite a forward command. The earlier launcher requested sling
support during the initial phase; actual operator-reported conditions take
precedence when interpreting these recordings. Moving segments are not clean
undisturbed command-response trials.

## Recorded disposition

Six attempts were found, including four complete 54-second powered sessions:

| Start time | Enables | Disabled readbacks | Result |
| --- | ---: | ---: | --- |
| 15:00:03 | 31 | 31 | Completed and passed final assessment |
| 15:02:13 | 31 | 31 | Completed; final zero-command settling assessment failed |
| 15:06:29 | None recorded | 0 | Stationary IMU calibration failed before CAN control |
| 15:06:44 | 0 | 31 | Actor rejected initial pose outside limits; subsequent TX audit also failed because no nominal control samples existed |
| 15:07:28 | 31 | 31 | Completed; final zero-command settling assessment failed |
| 15:09:41 | 31 | 31 | Completed; final zero-command settling assessment failed |

All four powered runs have 2,701 recorded states and 2,700 formal actions, no
CAN errors, and all 31 per-motor final-disabled flags. Every attempt restored
the CPU frequency floor. The generic failure message about "tracked startup"
is misleading for the three completed powered runs: the Xbox window opened
and finished before the final settling assessment failed. Completion and safe
shutdown are not evidence of acceptable gait quality.

## The backward angle reaches the policy correctly

`scripts/diagnose_backward_lean_20261117.py` checks the exact training MJCF hash,
joint order, sensor conversion, quaternion/gravity consistency, consumed IMU
packets and the newest gravity/gyro entries in the 993-dimensional observation.

- Every one of 10,804 recorded policy frames matches exactly one captured IMU
  sample, using all six gyro/gravity values and host timestamp bounds.
- Raw acceleration conversion matches exactly. Corrected angular velocity
  matches the 2000 degrees/s range conversion into rad/s and the frozen startup
  bias to below 9e-16 rad/s.
- Quaternion-derived gravity matches the logged unit projected gravity to
  1.2e-16. Observation differences are only float32 rounding: below 3e-8 for
  gravity and 1.2e-7 rad/s for gyro.
- CPU inference consumes IMU samples about 3.3 ms old at the median; maximum
  observed sample age is 8.91 ms. This excludes intrinsic sensor/filter delay.
- In the stationary 10–20 second windows, accelerometer-derived pitch,
  gyro-only integration and fused pitch agree closely at the segment-median
  level. Thus there is no evidence that fusion or the actor adapter is
  inventing the sustained backward angle or reporting it as level.

These checks do not independently calibrate the physical mounting angle or
encoder zeros. Gyro-only integration shares the initial attitude estimate and
may drift; accelerometer tilt includes linear acceleration during motion.

## Measured posture and simulation context

Negative pitch means backward. Medians in degrees:

| Run | Zero command, 10–20 s | Forward command >0.15 m/s, 20–50 s |
| --- | ---: | ---: |
| 15:00:03 | -7.89 | -5.96 |
| 15:02:13 | -7.94 | -5.98 |
| 15:07:28 | -7.50 | -3.50 |
| 15:09:41 | -9.21 | -4.67 |
| Saved same-checkpoint simulation | -5.03 | +1.65 at 0.25 m/s |

The simulated zero-command pose is already backward. Hardware adds a further
discrepancy, and the simulated forward posture does not transfer. Simulation
cases have different starts and command schedules; these differences cannot
be subtracted as a causal decomposition. Real moving segments include pushes.

The robot has waist roll and yaw, but no waist pitch joint. Reconstructing the
torso up-axis in the pelvis heading frame shows the fore/aft lean follows
pelvis pitch; it is not an isolated waist-pitch target. There is no reason to
change the waist gains based solely on this symptom.

## A verified training objective gap

The source hash of `opensprite_tracker/supermimic/joint_interfaces.py` exactly
matches the archived continuation-training source manifest:
`b359b2fe0f4aba374ab2ccb457e570f78eae1bba6f3c55259c301ea880c702e3`.

`VelocityMDP.reward_for_command` penalizes torso roll through
`4 * up_y**2 / (up_y**2 + up_z**2 + 1e-8)` and torso angular velocity, but
has no analogous static sagittal-pitch term. A stationary pure-pitch deviation
has zero contribution to that roll penalty. Motion tracking and latent
alignment still influence pose; it would be incorrect to say the whole
algorithm has no posture information. However, the velocity objective does
not explicitly distinguish a level zero-speed pose from a modestly backward
zero-speed pose by pitch alone. The saved simulation demonstrates that this
particular policy actually uses such a pose.

This is an embodiment/task weakness of the current trained objective and
policy, not evidence that SuperMimic's world-model method inherently fails.

## Remaining hardware discrepancy

Ankle pitch deserves follow-up. In the last trial's stationary window, the
left/right raw policy targets minus measured joint positions have medians
of +9.37/+6.74 degrees. In the saved zero-command simulation they are
-2.66/+0.76 degrees. None of the ankle motors recorded torque saturation in
the four powered runs. This rules out the recorded saturation mechanism as
an explanation for those ankle errors, but does not rule out load-dependent
tracking, motor-side behavior, mapping/zero offsets, or contact geometry.
PD tracking error under load is not itself a bug or a reason to raise gains.

Assuming both soles are flat, recorded joint angles imply pelvis pitch around
-4.1 to -5.6 degrees in the stationary windows, compared with IMU -7.5 to
-9.2 degrees. The approximately 3-degree mismatch is a useful cross-check,
not proof of IMU mounting error: actual sole edge contact, sole geometry,
joint zero and parallel-ankle mapping also affect it. No contact sensor or
independent optical pose was recorded.

## Timing and next discriminating checks

Median inference is 2.85–2.93 ms, P99 3.00–3.26 ms. Maximum inference times
are 5.59, 3.46, 3.46 and 8.56 ms. The last spike occurs at 26.45 seconds;
the sustained zero-command backward posture exists before it. The largest
feedback-to-target-accept age is 14.80 ms, below the unchanged 20 ms limit.
CPU-frequency stabilization improves typical timing but does not eliminate
every tail or fix posture by itself.

Prioritize:

1. Compare stationary support geometry and ankle pitch load/tracking against
   a matched simulation; independently distinguish sole/encoder/mounting
   offsets before changing any sensor transform or zero.
2. Evaluate an explicit bounded sagittal posture objective for zero and low
   speed, preserving whole-body motion tracking and allowing appropriate
   dynamic lean. Validate in simulation before retraining or deployment.
3. Separate undisturbed command tracking from push recovery. A backward step
   under a forward command can be balance recovery, but these logs lack ground
   truth translation and push timing, so they do not establish that mechanism.

Do not compensate by falsifying projected gravity, forcing a waist offset,
raising gains, or relaxing protections. Current evidence narrows the cause
but does not uniquely identify every hardware contributor.
