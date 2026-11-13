# Mixed-reference comparison and actuator sensitivity

## Scope and frozen training run

The rental is renewed through 2026-11-16 at noon, Asia/Shanghai. The new run
started at 2026-11-13 12:49:14 Asia/Shanghai on the L40S. Its isolated source
root is `/root/gpufree-data/supermimic_mixed_v3_20261113`; output is
`outputs/pm01-neutral-data-comparison`. The training PID at launch is 261930.
The source manifest SHA-256 is
`14f24d069d108761454b7072c029ceea546fd638fc21965c9fea4a74a41a1fc2`.

This is a fresh seed-43, 4096-environment, full joint SuperMimic run. It keeps
the PM01 reference-only comparison A's physics, rewards, published Kp/Kd,
action scale, optimizer, 500 Hz PD and 50 Hz policy. It does not include
comparison B's extra sole-height reward. The initial budget is 6030 updates,
with preserved checkpoints at 3000 and 6030 and a four-hour training wall cap.
Extension to 12000--18000 updates depends on physical gait evaluation, not
reward alone. The previous A/B runs remain preserved.

The production log has actual optimizer/world-model updates; this is not just
a queued command. Early world-model-only warmup throughput is not a valid
estimate of whole-run throughput. Prior PM01 runs took about 2--3 hours for
6030 updates; mixed references and concurrent audits may change that.

## Reference composition and review

The bank contains the corrected PM01 ordinary-walking clip and five reviewed
100STYLE Neutral clips: standing, three forward/path clips, and one rightward
lateral clip. It has 3236 frames, approximately 64.6 seconds of within-clip
time. Source samples are preserved exactly during packing; clips retain their
own boundaries. Its SHA-256 is
`19a1a2b0656934b81578a90540c44527972dd7c2a2bd5c94651c30f10f16c60c`.

Three composition tests check sample equality, stale-hash rejection and
validation-split rejection. A full-method three-update smoke run passed.
All six composition contact sheets were visually inspected, with 12 sampled
frames per clip. Existing source-level visual reviews are retained. This is
not a claim that a new full-length video was reviewed. The hash-bound QA and
activation review record this scope.

The reference sampler initially gives each clip equal probability, then uses
return-based weights inside each clip. Therefore raw frame counts do not
describe reset sampling probabilities. The measured initial reset distribution
has about 65.0% forward, 19.0% quiet, 13.7% rightward, 1.8% leftward, 37.1%
positive-yaw and 11.5% negative-yaw frames under the audit thresholds. Categories
can overlap. There is no accepted backward reference. These are initial reset
probabilities, not measured trained occupancy or velocity-command sampling.
This six-clip exploratory library is not the paper's dataset and is not a
balanced all-direction library.

Six additional reviewed Neutral clips remain held out with source-interval
separation. The held-out bank is never substituted into training.

## Delay semantics and independent plant diagnostics

Training has a stochastic zero-to-one-physics-tick command delay: 0--2 ms,
with the existing delay refresh/hold settings. The optional first-order torque
response is disabled (`torque_lag_enabled=False`). This is neither a measured
motor transfer function nor a model of the full feedback/inference/CAN path.

`scripts/evaluate_actuator_sensitivity.py` uses a separate evaluator process
and explicitly labeled plant overrides. It does not edit the frozen run or
the hardware runtime. Twelve profiles include nominal physics, fixed 10/20 ms
command delay, two 4340 friction levels, physical viscous friction, armature
at 0.7/1.3 times nominal, 5/10 ms torque response, and two illustrative
4340 torque-speed envelopes. Each profile changes one factor.

Physical viscous friction is not actuator Kd. Compiled model parameters are
checked after construction, while all published Kp/Kd and constant caps must
remain unchanged. Torque-speed diagnostics preserve constant-cap braking and
reduce motoring torque with speed; applied-torque logs record the actual
post-envelope effort. The external older 4340 bench prior is not asserted to
identify the installed 40 Nm revision. Scaling no-load speed by 38/48 is an
explicit voltage hypothesis, not a measured voltage model. Parallel ankle and
head motor mappings and unverified 6248 voltage ratings are excluded from
these direct-4340 parameter changes.

The three profile/math unit tests and nominal, friction, speed-envelope and
20 ms delay simulation smoke checks passed. In a four-second stationary
comparison, diagnostic-baseline differences were within 1.1 times the native
evaluator's own repeat differences plus 1e-7 for every floating output field;
non-floating arrays matched exactly. The largest qpos difference was 3.53e-5,
and the native repeat differed by 3.51e-5. GPU physics is not bitwise repeatable;
no exact-trajectory claim is made.

The sweep launched with all twelve profiles on A and four additional checks
(nominal, 20 ms delay, friction 0.339, 38 V speed hypothesis) on B. Each evaluates
twelve command/perturbation cases for 20 seconds at seed 101, followed by
per-step geometry and recovery analysis. Additional seed replication is needed
before interpreting small differences as robust. Local results are preserved
under `outputs/retrain-20261113/actuator-sensitivity` as they complete.

### First completed delay comparisons

At 13:03 China time the new mixed-reference training had reached update 810.
The independent A sensitivity tests had completed baseline, 10 ms delay,
20 ms delay and friction 0.1, without harness errors. The delay subset gives:

| A profile | Surviving cases | Mean forward speed at 0.25 m/s command | Left/right median swing peak |
| --- | --- | --- | --- |
| Nominal 0--2 ms delay | 12/12 | 0.2101 m/s | 12.97 / 6.81 mm |
| Fixed 10 ms | 12/12 | 0.2125 m/s | 12.18 / 7.03 mm |
| Fixed 20 ms | 12/12 | 0.2015 m/s | 10.58 / 6.64 mm |

These results do not establish satisfactory gait across all twelve cases.
For example, backward clearance deteriorates under some profiles. They test
the newly trained simulation candidate A, not the older hardware policy;
they cannot diagnose the prior robot's failure to start by themselves.
The remaining profiles are still running as of this entry.

## Evaluation and local preservation

The standard milestone watcher evaluates 48 command/disturbance cases with
three evaluation seeds, motion tracking, sole tracking and rendered videos.
The supplementary watcher adds all-case bilateral clearance events, loaded
slip, foot and torso heading, 60-second stationary/start-stop evaluations and
held-out motion tracking at 3000 and 6030 updates. Evaluation-seed repeats are
not independent training seeds. All results remain simulation-only.

Local backups are stored under
`outputs/retrain-20261113/mixed-reference-training`. The first backup completed
after launch. A bounded hourly worker preserves checkpoints, derived motion
data, provenance and audits, without downloading generic raw BVH/FBX datasets.
The source archive and manifest are also stored locally under `.local`.
Dataset licensing and third-party notices remain separate from project AGPL-3.0.
