# Independent PM01 seeds: completed results

Both fresh runs reached update 9000. All six milestone audits completed at
15:36 China time on November 14; local evidence backup finished at 15:44.
The GPU was idle after this bounded queue ended until the 17:40 review.
The new diagnostic queue described below was started during that review.

Independent training can reproduce walking, but the recipe still has substantial
checkpoint and seed variability. No replication candidate replaces parent 7629.
All values below are simulation measurements, not hardware validation.

| Candidate | Forward vx, command 0.25 m/s | Swing medians L/R | 48-case survivors, seeds 101/202/303 | Matched recovery, 40 forces per seed |
| --- | ---: | --- | --- | --- |
| Seed 44, 7500 | 0.183 m/s | 18.4 / 15.3 mm | 48/48/48 | 38/37/38 |
| Seed 44, 9000 | 0.158 m/s | 14.5 / 16.0 mm | 48/48/48 | 40/40/40 |
| Seed 45, 7500 | 0.191 m/s | 19.5 / 21.0 mm | 48/48/48 | 40/40/40 |
| Seed 45, 9000 | 0.209 m/s | 19.2 / 15.3 mm | 48/48/47 | 36/40/38 |

Forward/clearance figures use the seed-101 standard 20-second audit. Matched
recovery is the supplemental velocity/yaw-rate/tilt criterion; it does not
require restoring world heading or position. Survival is not the same as
recovery. Preserve original instantaneous metrics alongside this supplement.

## Retain seed 45 at 7500 as an independent research candidate

This checkpoint combines bilateral clearance with quiet stationary behavior.
All five 60-second nominal cases survived, including turns and start/stop.
Zero-command mean horizontal speed was 0.0000845 m/s. Forward-run mean speed
was 0.1875 m/s, and left/right curve yaw rates were +0.3229/-0.2382 rad/s for
commands +/-0.30 rad/s.

It has an unresolved straight-walking bias: with commanded yaw rate zero, the
60-second forward run averaged +0.0472 rad/s, about 2.70 degrees/s. This is
sustained uncommanded rotation without a disturbance, not an acceptable settled
heading change after a push. Therefore, preserve the checkpoint for comparison
but do not promote it over the existing baseline.

Seed 44 at 6000 failed many walking cases (22/48 survivors per seed); its large
pre-failure foot-height values must not be interpreted as improved foot lift.
Both runs improved by 7500, and neither final checkpoint dominated its earlier
candidate. Further blind increases in update count are not justified by these
results alone.

## Next diagnostic queue

Started 24 simulation-only comparisons across parent 7629, quarter-LR 8500,
quarter-LR 10000 and seed-45 7500. For each candidate, evaluate nominal physics,
fixed 10/20 ms command delay, 5/10 ms torque-response constants, and a nominal
controller-default starting pose. Each uses the existing 48-case command/force
suite for 20 seconds with seed 101. Two bounded evaluation lanes run concurrently.

These actuator perturbations are diagnostic hypotheses, not measurements of
the installed motors. Kp/Kd, nominal effort caps, policy history and runtime
protections remain unchanged. The wrapper verifies the compiled physical
parameters and published gains/caps. Initial-pose and latency comparisons must
be labeled separately from nominal checkpoint scores.

This queue is evaluation, not another training run. It determines whether a
candidate's improvement survives plausible execution mismatch before choosing
the next training modification. It does not establish accurate real-robot
actuator parameters or replace later real-data alignment.

Replication evidence is in `outputs/retrain-20261114/seed-replication` locally.
The new remote queue is
`/root/gpufree-data/supermimic_velocity_v2/outputs/candidate-transfer-audit-20261114`;
its local backup is under
`outputs/retrain-20261114/candidate-transfer-audit-20261114`.
