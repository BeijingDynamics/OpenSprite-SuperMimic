# Steering response and seed-45 continuation

## User acceptance criterion

Steady yaw bias is acceptable if explicit control commands can reliably
correct it while preserving balance and walking. Continue investigating its
origin. This is separate from returning to an original world heading after
a push, which is not required. Existing reference, robot gains, action scaling,
history and hardware protections remain unchanged.

## Completed execution-mismatch diagnostics

All 24 tests finished without evaluator errors. Four candidates each received
six conditions: baseline, fixed 10/20 ms command delay, 5/10 ms torque response,
and the controller-default starting pose. Each test used 48 command/force cases
for 20 seconds, evaluation seed 101. These are diagnostic simulation models,
not identified real-actuator parameters.

Seed45 at 7500 survived all 48 cases under every condition. Its uncommanded
forward yaw rate remained positive:

| Condition | Mean yaw rate (rad/s) |
| --- | ---: |
| Baseline | +0.04687 |
| Controller-default initial pose | +0.04665 |
| Fixed command delay 10 ms | +0.05363 |
| Fixed command delay 20 ms | +0.05319 |
| Torque response 5 ms | +0.04845 |
| Torque response 10 ms | +0.05129 |

Other policies under the same baseline plant differed: parent 7629 was about
-0.00331 rad/s, quarter-LR 8500 about -0.02078, and quarter-LR 10000 about
+0.01548. This weakens an explanation based solely on initial pose or one
specific actuator delay. A learned gait/controller bias is a working hypothesis,
not an established root cause. Reference asymmetry, objective sensitivity and
world-model errors remain possible contributing mechanisms.

Quarter-LR 10000 also survived all six 48-case tests. Parent 7629 had one
failure under 20 ms command delay; quarter-LR 8500 had one under 20 ms delay
and one under 5 ms torque response. Retain case-level results rather than
equating survival with disturbance recovery.

## Explicit steering-response experiment

Compare seed45-7500, quarter-LR-10000 and parent-7629 under evaluation seeds
101, 202 and 303. Each rollout contains 33 fixed command combinations:
forward speed 0.15/0.25/0.35 m/s and yaw rate
-0.12/-0.08/-0.06/-0.05/-0.04/-0.02/0/+0.02/+0.04/+0.08/+0.12 rad/s.
An additional staged case changes yaw command from zero to -0.06, back to
zero, then +0.06 and zero, at fixed 0.25 m/s forward command.

Measure steady response, the command at which yaw changes sign, response
consistency across speed/seeds, transient response, foot clearance and failures.
The command values are explicit test inputs; no runtime bias compensation is
installed. A usable small-command response must be demonstrated, not inferred
from the existing +/-0.30 rad/s turns.

The initial test setup omitted the evaluator's mandatory command stage at
2 seconds and was rejected during validation. Its failed setup evidence is
retained and invalid for performance conclusions. The corrected `v2` queue
includes that stage; only its completed results may be used.

Server evidence:
`/root/gpufree-data/supermimic_velocity_v2/outputs/steering-response-20261114-v2`.
Local backup:
`outputs/retrain-20261114/steering-response-20261114-v2`.

## Training resumed at 18:22 China time

Two continuations resume the independently reproduced seed45-7500 checkpoint
`f74a2ba5d8d6465ccfbffaba1a09ca9cf48023b5f8a36bb55aa3f29cbebf32f4`.
The control retains original actor LR; treatment uses a 0.25 actor multiplier.
World-model/critic LR, reference, physical model, rewards and curriculum are
unchanged. Both rebuild replay, live simulator and RNG identically; neither
claims a bitwise resume. This tests whether the actor-step-size effect observed
under seed 43 also occurs on the seed-45 good-clearance controller.

Both targets are update 10000, with physical audits at 8000, 8500, 9000 and
10000. At update 7510, actual actor rates were approximately 1.99569e-4 and
4.98922e-5; both world-model/critic rates remained 1.99569e-4. GPU utilization
was observed at 100%. Actual optimizer updates, not process existence alone,
confirmed that training resumed.

The earlier queue had already finished when checked at 18:19; no training was
active at that time. These new continuations started subsequently, at 18:22.
Important evidence is backed up locally by dedicated workers.

Server evidence is under
`/root/gpufree-data/supermimic_pm01_lr_20261114/outputs/seed45-lr-continuation`;
local evidence is under `outputs/retrain-20261114/seed45-lr-continuation`.
