# Simulated PD torque and actuator-friction budget, November 14, 2026

No training or hardware control was performed. The user requested quantitative
review before approving a new actuator model. Existing policies, gains and caps
were unchanged; friction overrides were confined to diagnostic simulations.

## Method

Both seed45 quarter-actor-rate finalists (9000 and 10000 updates) were evaluated
at 50 Hz policy / 500 Hz PD, five command cases, 20 seconds each, seed101. Cases:
zero; forward 0.25 and 0.35 m/s; curves at 0.2 m/s with yaw +/-0.3 rad/s.
An observer recorded q, qd, delayed position target, signed P/D contributions,
raw PD torque and saturated applied torque at all 10,000 physics substeps.
Both audits verified the records against the existing per-policy-step torque
peaks and the published clipped-PD formula. No observer dynamics were added.

Statistics exclude the first three seconds and any post-failure samples.
Swing proxy: own foot force <5 N and opposite >5 N, stable across adjacent
50 Hz frames; transition frames are excluded. It is an unloaded-foot proxy,
not a 500 Hz geometric toe-off detector. Dynamic friction statistics exclude
|qd|<0.05 rad/s, where static friction depends on the applied load. Ratios are
sums of absolute friction divided by sums of absolute motor torque on the
same samples, avoiding division by individual near-zero torques.

## Friction prior and scope

[MABEL J4340P source](https://github.com/robotmabel/open_mabel/blob/main/calibration/motor_calibration/results/DM-J4340P/params.json)
reports Coulomb friction 0.333081/0.345377 Nm by direction and viscous friction
0.183973 Nm s/rad. The torque-budget calculation uses those directional values.
Closed-loop comparisons use its symmetric MuJoCo approximation 0.339229 Nm
plus passive damping 0.183973, without the Stribeck term. A smaller 0.1 Nm
Coulomb-only diagnostic bounds a less severe case; it is not an identified
parameter for our installed hardware.

All changes are limited to the 13 directly mapped J4340P joints. J4310P, J3507,
J6248P and parallel transmissions are not assigned borrowed friction values.
MABEL's 27 Nm revision is not established as matching our 40 Nm revision.
These are explicit external-prior tests, not measured hardware torque losses.

## Latest 10000-update policy, forward command 0.25 m/s

Moving unloaded-leg samples; units Nm. Coulomb percentage omits viscous loss.

| Joint | Mean absolute PD torque | Estimated friction | Coulomb/PD | Total friction/PD |
| --- | ---: | ---: | ---: | ---: |
| left_hip_pitch_joint | 1.388 | 0.576 | 24.8% | 41.5% |
| left_hip_roll_joint | 0.512 | 0.385 | 66.4% | 75.1% |
| left_hip_yaw_joint | 0.390 | 0.425 | 85.7% | 109.0% |
| left_knee_joint | 1.344 | 0.657 | 25.1% | 48.8% |
| right_hip_pitch_joint | 2.314 | 0.610 | 14.5% | 26.3% |
| right_hip_roll_joint | 0.636 | 0.416 | 53.1% | 65.5% |
| right_hip_yaw_joint | 0.433 | 0.440 | 79.5% | 101.5% |
| right_knee_joint | 2.034 | 0.767 | 16.8% | 37.7% |

High ratios do not mean the policy loses that percentage of walking speed.
P and D terms cancel substantially during motion. At t=3.266 s, left knee:
action=0.1161096; q_target=0.3+0.25*action=0.3290274 rad;
q=0.2435632 rad; qd=1.9348631 rad/s; Kp=130, Kd=5.
P=11.11035 Nm; D=-9.67432 Nm; applied=1.43604 Nm (below the 40 Nm cap).
Estimated friction is 0.68905 Nm, about 48% of that sample's net PD torque.

With Kp=130, 0.339 Nm requires only 0.1495 degrees of extra position error;
0.66 Nm requires 0.291 degrees. The passive viscous coefficient is about 3.68%
of the configured controller Kd=5. These comparisons explain how a high
instantaneous net-torque ratio can coexist with a modest closed-loop change.
They do not prove the phase response or static breakaway is always harmless.

## Closed-loop results without retraining

Forward metrics below use the 0.25 m/s case. Swing height is the median of
per-event sole-clearance peaks, not mean clearance and not the single maximum.
Survival covers all five command cases for the given policy and plant.

| Iteration | Plant | Forward m/s | Yaw rad/s | L/R swing peaks mm | Survival |
| --- | --- | ---: | ---: | --- | --- |
| 9000 | baseline | 0.1969 | 0.0232 | 18.05 / 18.86 | 5/5 |
| 9000 | Coulomb 0.10 | 0.1958 | 0.0220 | 17.99 / 18.77 | 5/5 |
| 9000 | Coulomb 0.339 | 0.1931 | 0.0191 | 17.72 / 18.48 | 5/5 |
| 9000 | Coulomb + viscous | 0.1903 | 0.0193 | 17.54 / 18.35 | 5/5 |
| 10000 | baseline | 0.2039 | -0.0051 | 16.06 / 18.41 | 5/5 |
| 10000 | Coulomb 0.10 | 0.2028 | -0.0053 | 15.91 / 18.39 | 5/5 |
| 10000 | Coulomb 0.339 | 0.2008 | -0.0054 | 15.63 / 18.26 | 5/5 |
| 10000 | Coulomb + viscous | 0.1981 | -0.0056 | 15.44 / 18.06 | 5/5 |

All 30 friction-perturbed trials survived (2 policies x 3 plants x 5 cases).
Each trial lasted 20 seconds, with one evaluation seed and no external pushes.
This establishes limited nominal-command tolerance, not a hardware guarantee
or full robustness validation. The 10000 policy loses about 2.9% of forward
speed under the combined prior, with submillimeter median clearance changes.
The 9000 policy shows the same qualitative result. The model still has its
preexisting speed-tracking deficit before friction is introduced.

## What current domain randomization actually covers

- Ground friction: 0.35--1.2. This is foot/ground friction, not gearbox friction.
- Joint armature: 0.7--1.3 times nominal. Acceleration-dependent uncertainty.
- Encoder bias: +/-0.01 rad. With Kp=130, a 0.01 rad error has a 1.3 Nm PD scale;
  this is an order-of-magnitude comparison, not equivalent friction coverage.
- Torso center of mass offsets: +/-[0.00588, 0.01225, 0.01102] m.
- Command delay: 0--1 physics tick (0--2 ms).
- Joint frictionloss: multiplied by 0.5--1.5, but nominal is zero, so the result
  is still exactly zero. No nonzero motor friction is sampled by this term.
- Gain and effort-limit randomization: disabled by the approved profile.
- Torque-response lag: disabled in the completed recipe.

The policy/PD loop tolerated the tested friction priors despite this coverage
gap. Attributing a particular fraction of that tolerance to domain
randomization would require a matched training ablation without it. It would
be incorrect to say randomization explicitly covers or cancels 97% of friction.

## Assessment

For current slow forward walking at full training gains, this external
friction model appears to be a refinement, not the leading explanation for
earlier failure to step or severe dragging. It is not negligible relative to
low-load hip yaw/roll torque, but the current controller compensates well in
the tested simulations. A small explicit nonzero friction randomization may
be worth comparing after review; an elaborate learned actuator network is
not justified by these results alone.

Preserve attention to measured control/feedback delay, effective embedded-PD
semantics, parallel-linkage motor/joint mapping, contact/sole geometry and the
policy's existing clearance and symmetry. These may matter more than fitting
another decimal place of an unmatched friction prior. Independent finalist
audits already show fixed 20 ms command delay reduces the 10000 policy's
forward speed from about 0.204 to 0.188 m/s, a larger change than this friction
prior, although delay location/distribution must match the real system before
it becomes a training model. No causal claim about old hardware runs follows.

## Local evidence

`outputs/retrain-20261114/torque-budget-20261114/` contains both full-rate torque
traces, standard rollouts, contract checks, both torque summaries, all friction
rollouts and geometry metrics, closed-loop comparison JSON and local hashes.
Original finalists remain unchanged. New training remains pending user review.
