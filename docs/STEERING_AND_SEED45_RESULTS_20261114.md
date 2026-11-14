# Steering correction and seed-45 continuation results

All nine steering rollouts and all eight continuation milestone audits have
completed. The continuation evaluator finished at 20:16 China time on November
14. Final incremental continuation backup completed locally at 20:22. The GPU
was idle when checked at 20:19 because the bounded jobs had finished.

## Explicit commands correct the seed45-7500 yaw bias

The result is supported by simulation, not yet hardware testing. At a forward
command of 0.25 m/s, the last-five-second yaw rate with zero turn command was
approximately +0.047 rad/s. With explicit yaw command -0.08 rad/s, the rates
were -0.00524/-0.00542/-0.00553 rad/s for seeds 101/202/303: about -0.30 to
-0.32 degrees/s instead of +2.7 degrees/s. Actual forward speed remained about
0.187 m/s, and swing medians remained about 19.2/20.6 mm.

All 34 cases per seed survived for seed45-7500: 33 small-command combinations
and one staged sign-change case, for 102/102 runs. At forward commands 0.15,
0.25 and 0.35 m/s, the yaw-response curves crossed zero between commands -0.08
and -0.06 rad/s. The precise offset depends on speed and policy; it is not a
universal robot calibration constant. No offset was inserted into runtime.

The staged experiment confirms live response rather than merely selecting a
different initial state. Under 0.25 m/s, changing yaw command from 0 to -0.06
reduced late-phase yaw rate to approximately +0.005 to +0.007 rad/s; restoring
zero command restored approximately +0.048 rad/s. Command +0.06 produced
approximately +0.088 rad/s, then returning to zero restored the original bias.
Use raw trajectories for settling transients, rather than interpreting these
late-phase means as instantaneous response times.

Quarter-LR 10000 from the earlier seed-43 continuation also admits small-command
correction, but its required correction changes sign at the highest tested
forward command. This reinforces the need to distinguish command responsiveness
from a single fixed trim. The data remain available in
`outputs/retrain-20261114/steering-response-20261114-v2`.

## Seed-45 continuation shortlist

| Candidate | Forward vx at 0.25 m/s command | Swing medians L/R | Mean forward yaw rate | 60 s zero-command horizontal speed | Matched force recovery |
| --- | ---: | --- | ---: | ---: | --- |
| Parent seed45-7500 | 0.191 m/s | 19.5 / 21.0 mm | +0.0469 rad/s | 0.0000845 m/s | 40/40 in each of 3 seeds |
| Quarter actor LR, 9000 | 0.197 m/s | 18.1 / 18.9 mm | +0.0227 rad/s | 0.0000452 m/s | 40/40 in each of 3 seeds |
| Quarter actor LR, 10000 | 0.204 m/s | 16.1 / 18.4 mm | -0.00538 rad/s | 0.0000676 m/s | 40/40 in each of 3 seeds |
| Original actor LR, 10000 | 0.287 m/s | 9.7 / 15.4 mm | +0.0197 rad/s | 0.000299 m/s | 34/36/35 of 40 |

Forward, yaw and foot values above use the standard seed-101 20-second audit.
The original-LR final checkpoint's mean forward speed dropped to 0.210 m/s in
the separate 60-second case, so its short-run speed must not be presented as
sustained performance. All three final/new shortlisted checkpoints survived
48/48 cases in each standard seed, and all five long nominal cases; survival
alone does not erase the original-LR clearance and recovery regression.

Retain quarter-LR 9000 for its clearance/stopping/recovery balance. Retain
quarter-LR 10000 for its reduced uncommanded yaw, with lower left-foot clearance
explicitly recorded. The seed45-7500 parent remains a useful candidate now that
small-command correction is demonstrated. No single policy has been silently
promoted to hardware, and the seed-43 parent 7629 remains preserved.

These outcomes support reduced actor step size as a useful refinement strategy
on this second seed, while showing that it does not monotonically improve all
gait measures. The exact source of residual yaw bias is still unresolved.

## Next validation

The two new quarter-LR finalists enter the same six-condition plant/start-pose
audit used for earlier candidates: nominal, 10/20 ms command delay, 5/10 ms
torque response and controller-default initialization. This is 12 further
simulation evaluations, not another training run. Hypothesized actuator changes
remain clearly labeled and do not modify hardware gains or protections.

The new local evidence directory is
`outputs/retrain-20261114/candidate-transfer-finalists-20261114`.
Continuation checkpoints, videos and metrics are in
`outputs/retrain-20261114/seed45-lr-continuation`. Important artifacts are backed
up locally; hardware qualification remains false.
