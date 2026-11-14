# PM01 continuation from audited checkpoint 7629

## Objective and retained baseline

Prioritize reliable forward steps, usable turns, quiet stops and disturbance
recovery. Reverse and lateral tracking remain secondary. The PM01-only 7629
checkpoint remains the research baseline until new physical evaluations justify
a change. It is not hardware-qualified. No Jetson or motor operation is part
of this experiment.

Parent SHA-256:
`eaaba88997aa870dd95df92e12cbe684f877bbcf11790f86d70283d38290f8dc`.
The previous 9000 continuation regressed, including sustained movement under
zero command. It is preserved as negative evidence and is not the new parent.
See [the continuation record](PM01_CONTINUATION_20261113.md).

## Controlled comparison

Both arms restore learned networks, optimizer moments, critics and reference
sampler state from 7629. Replay, live simulation and RNG restart; this is not
bitwise continuation. Both arms use the same restart seed and conditions.

| Arm | Actor learning-rate multiplier | World-model and critic multiplier |
| --- | ---: | ---: |
| `control` | 1.0 | 1.0 |
| `actor-quarter` | 0.25 | 1.0 |

The existing cosine schedule remains in force. The multiplier applies to both
disjoint actor optimizers: motion encoder/decoder and velocity encoder. It is
applied on every optimizer update **after** checkpoint restoration. Merely
changing a loaded optimizer once would be overwritten by the original loop.
Actual parameter-group learning rates are logged every reporting interval.

At update 7700 the control actor, critic and world-model rates were approximately
`1.9954691e-4`; the treatment actors were `4.9886728e-5` while its critic and
world-model rates remained `1.9954691e-4`. A focused test exercised all parameter
groups and repeated schedule application after simulated optimizer restoration.

The following remain unchanged: corrected PM01 reference, robot assets,
published gains, 500 Hz PD, 50 Hz policy, action scaling and history semantics,
actuator assumptions, command distribution, recovery curriculum, network,
world-model validation split, velocity reward, and the existing experimental
motion sole-height term (`std_m=0.015`, `weight=0.25`). This is a continuation
study, not a claim that the sole-height term is an official paper default.

Reference SHA-256:
`7c6dccdc87756956f58ed9c5c6c7432882172abc62f116b803f719ecc16e6870`.

Each arm targets update 10000, with retained checkpoints at 7630, 8000, 8500,
9000 and 10000. The additional wall-clock limit is eight hours per arm.
Checkpoint 7630 verifies effective optimizer state; the other four milestones
receive physical audits. Training is not automatically promoted by reward or
by reaching the final update.

## Execution and evidence

The control started at 2026-11-14 08:19:26 China time (PID 360056); treatment
started at 08:20:38 (PID 360443). Both produced finite optimizer updates.
Observed concurrent GPU usage was approximately 37.6 GB and 98–99% utilization,
with each arm processing approximately 0.44–0.46 updates/s at this early stage.
These are observed samples, not a guaranteed completion time. Evaluations share
the GPU and can reduce throughput.

Frozen server root:
`/root/gpufree-data/supermimic_pm01_lr_20261114`.
Run evidence is under `outputs/actor-lr-comparison` within that root.
Local evidence is copied to
`outputs/retrain-20261114/actor-lr-comparison` in this checkout.

The run stores parent/reference hashes, changed configuration fields, source
snapshot and manifest, separate evaluation overlay, effective learning rates,
world-model holdout diagnostics, checkpoints, physical rollouts and videos.
The local backup worker copies new evidence every 30 minutes and after the
bounded evaluator finishes. Reusable upstream raw datasets are excluded.

## Physical selection protocol

A single evaluator serializes both arms' milestone audits, avoiding competing
evaluation processes. Each candidate receives the existing 48-case suite under
seeds 101, 202 and 303, physical sole geometry, and a 60-second suite covering
zero command, sustained forward motion, left/right curves and start/stop.
Recorded-pose videos show standing, forward motion and both turn directions.
Original instantaneous recovery metrics remain available.

An additional command-matched report compares one-second averaged velocity and
yaw rate with an unforced case at the same command. It retains raw tilt,
failure timing, signed command error, accumulated heading and displacement.
It stops at the first failure of either case, so later reset frames cannot
produce a false recovery. Three tests cover normal stride oscillation,
persistent velocity bias and failure before recovery. Same-command cases do
not imply identical initial states.

For the parent 7629 at seed 101, all 40 force events resumed nominal-like
velocity/yaw-rate behavior under this supplementary criterion, with confirmation
within 4.74 seconds. This **does not mean full state restoration**: the largest
accumulated heading difference from the corresponding unforced run was about
34.65 degrees. The original instantaneous criterion reported only 26/40,
partly because undisturbed strides themselves crossed its velocity threshold.
Both reports must remain visible; neither alone establishes robust navigation.

New candidates must be compared on alternating foot lift, loaded slip, foot and
torso alignment, sustained forward motion, usable curves, quiet zero-command
behavior and disturbance response. Survival and aggregate reward alone are
insufficient. Hardware protection thresholds and runtime parameters are not
changed by this research.

## New retained result: 60-second baseline audit

The identical long suite for parent 7629 completed on November 14 and was copied
locally with its actual rollouts and geometry report. All five cases survived
60 seconds without resets. These are nominal simulation results, not a hardware
walking claim.

| Case | Measured forward speed (m/s) | Measured yaw rate (rad/s) | Foot event peak medians L/R (mm) |
| --- | ---: | ---: | --- |
| Zero command | -0.00014 | 0.00003 | No stepping events |
| Forward, command 0.25 m/s | 0.1926 | -0.00553 | 20.20 / 18.93 |
| Left curve, command 0.20 m/s and +0.30 rad/s | 0.1840 | +0.2384 | 18.55 / 17.52 |
| Right curve, command 0.20 m/s and -0.30 rad/s | 0.1644 | -0.2365 | 21.49 / 18.26 |

Zero-command mean horizontal speed was 0.000534 m/s. The start/stop case ended
with mean forward velocity -0.000922 m/s over its final five seconds. The
forward case's minimum detected swing peaks were 20.11/17.96 mm, so the
reported median was not hiding an occasional nearly grounded swing in this
particular nominal rollout. Foot headings relative to the pelvis were
approximately +3.94/-1.62 degrees; torso heading was -0.23 degrees. Modest
speed undershoot and a small forward-run yaw-rate bias remain visible.

This strengthens the evidence for retaining 7629 while the comparison trains.
The source files are `parent-7629-long-command/summary.json`, `rollout.npz`
inside that directory, and `parent-7629-long-geometry.json` under the local
comparison evidence directory. New candidates use the same suite.

The complete 60-second four-panel recording is
`parent-7629-long-command-review.mp4`, with SHA-256
`edc8cd54c8f90416e3b2f27038add00ff22d242afc7e4a18b8ebde7114c37d14`.
It shows forward motion, left curve, right curve and stop/restart from actual
recorded poses. A contact sheet at 3, 20, 40 and 58 seconds was inspected.

## Queued independent-seed replication

After **both** current trainers finish normally at update 10000, a bounded
queue launches the same PM01 B recipe from scratch under seeds 44 and 45.
The original seed was 43. This tests reproducibility rather than assuming a
single successful checkpoint is representative. The queue is prepared and
running as a waiter; these two replications have not started at the time of
this record.

Both replications retain the original actor learning-rate schedule (multiplier
1.0), full world-model warmup and all physical/reward/reference settings.
No checkpoint is loaded. Each targets update 9000 with an eight-hour runtime
limit. Checkpoints at 2000, 4000, 6000, 7500 and 9000 are retained; 6000, 7500
and 9000 receive the three-seed physical and long-command audits. Intermediate
checkpoints remain candidates even if the last checkpoint regresses.

The queue checks normal predecessor completion and frozen source hashes before
launching. A predecessor crash, numerical failure or source mismatch blocks
the queue instead of silently starting another job. The replication evaluator
waits for the earlier evaluator to exit, keeping one physical evaluation
process active at a time. Trainers may run alongside that evaluator.

Server evidence is under `outputs/seed-replication` in the same frozen root;
local evidence is under `outputs/retrain-20261114/seed-replication`.
`replication-contract.json`, `queue-preflight.json`, `queue-job.json`,
`queue-source.tar.gz` and `queue-status.json` preserve the preparation and
actual transition into training. A separate local backup worker is active.
Queued work is not reported as completed training or as a better policy.
