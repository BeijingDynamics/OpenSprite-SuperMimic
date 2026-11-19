# Targeted upright and moving-foot clearance comparison

The user renewed the rental on November 18 and requested a defensible solution
to backward lean and asymmetric foot clearance. This experiment is separate
from the completed posture/width continuation. No runtime, motor gain, action
scale, robot asset, motion reference or protection setting changes.

## Evidence before training

An offline MuJoCo geometry audit found reference torso pitch P05/median/P95
of -0.305/-0.076/+0.075 degrees, and left/right sole-height P95 of
23.67/23.87 mm. Thus a consistently backward or grossly asymmetric reference
is not supported by these measurements. This does not exclude all reference
limitations, especially the absence of explicit turning clips.

The update-5000 zero-speed rollout incurs a mean existing pitch cost of only
0.00937 before the shared 0.02-second reward multiplier, against up to 4 units
of velocity/yaw tracking reward. The proposed pitch setting raises that same
recorded-state cost to 0.08545. This is an objective-scale mismatch hypothesis,
not proof that a single coefficient explains the learned equilibrium.

Motion tracking has an explicit sole-height term; the velocity interface has
none. It rewards command tracking and smooth actions without directly requiring
the moving sole to clear the floor. At update 10000, forward action-change
cost falls from 0.0408 at update 5000 to 0.0266, alongside much lower foot lift.
This is consistent with a low-clearance compromise, but does not establish
whether reward weights, the shared decoder, world-model errors or the recovery
curriculum caused the deterioration. The comparison is intended to narrow it.

## Controlled arms

All arms transfer from update 5000, SHA256
`8351af3dfe72236de5592a6ecd97592d001beab331bf223cc153500ba04068ac`,
using seed 47, the unchanged full joint SuperMimic algorithm, 4096 environments,
the same reference and physical contract. Velocity critics and optimizers are
reset consistently. Actors are frozen for the first 100 updates while the
new velocity critic learns; world-model recollection warmup is 20 updates.
Recovery impulses remain at the parent's update-5000 strength: maximum linear
increment 0.20 m/s and angular increment 2/7 rad/s, with the existing nominal
environment fraction and intervals. This fixes curriculum strength across
arms; it does not test robustness at the former update-8000 maximum.

| Arm | Pitch term | Additional velocity clearance term |
| --- | --- | --- |
| control | weight 8, 2-degree deadband | none |
| upright | weight 32, 1-degree deadband | none |
| upright-clearance | weight 32, 1-degree deadband | weight 0.15 |

The clearance term is the mean of identical left/right costs. It penalizes
sole-bottom height below 20 mm, weighted continuously by world horizontal
sole-center speed squared divided by speed squared plus 0.08 squared. A
command-speed gate makes it exactly zero at zero command; turn commands also
activate it. It has no phase clock, fixed left/right timing, instantaneous
symmetry target, joint-target correction or torque feedforward. Stationary
stance feet receive zero cost. Turning can retain different inner/outer step
lengths. This is an explicit embodiment-specific reward experiment, not a claim
that this term appears in the original paper.

## Execution and review

The queue trains arms serially, evaluating at 600, 1600 and 3000 fresh updates.
Every stage saves physical rollouts, posture/spacing metrics, clearance metrics,
the unchanged quality screen and a four-panel video. A bounded research gate
stops an arm for falls, severe lean, loss of forward motion, markedly reduced
bilateral lift, narrowed spacing or loss of turn direction. It is separate
from the stricter deployment-candidate screen. The two best surviving research
entries receive paired initial-tilt and finite-force evaluations.

Twenty-eight CPU tests passed, including reward gradient direction, foot-swap
symmetry, heading invariance, zero-command behavior, unchanged observations and
termination behavior, and exact zero-weight legacy reward compatibility.
A three-update small GPU transfer smoke completed with finite telemetry. The
smoke checks integration; its actor-freeze window does not prove successful
policy optimization. Full-size training has started with the clearance arm.

The coordinator stops by November 19 at 14:00 China time, leaving an hour
before the assumed renewed 15:00 lease expiry. A local evidence copier runs
through 14:55 and saves completed checkpoints with SHA verification, metrics,
configs, videos and validation rollouts. Source and diagnosis are also kept
locally. No model is qualified for hardware by this queue alone.

## First 600-update result

The upright-clearance arm completed its first physical evaluation. The upright
arm was training and the control arm remained queued at the time of review;
this is not yet a completed three-arm causal comparison.

| Nominal metric | Transfer parent, update 5000 | Upright-clearance, fresh update 600 |
| --- | ---: | ---: |
| Zero-command median torso pitch | -3.99 degrees | -1.93 degrees |
| Forward 0.25 m/s measured speed | 0.199 m/s | 0.225 m/s |
| Forward 0.25 m/s swing peaks L/R | 17.58 / 20.78 mm | 16.58 / 16.82 mm |
| Forward 0.25 m/s P05 center spacing | 115.8 mm | 115.7 mm |
| Left-turn swing peaks L/R | 12.23 / 19.74 mm | 15.23 / 16.28 mm |
| Right-turn swing peaks L/R | 13.56 / 15.83 mm | 13.95 / 9.87 mm |
| Left/right turn yaw responses | +0.148 / -0.246 rad/s | +0.195 / -0.203 rad/s |

All five 20-second nominal cases survived. The zero-speed posture and forward/
left-turn clearance symmetry improved, but right-turn right-foot clearance
regressed. At the 0.15 m/s command, forward speed was 0.125 m/s, with only seven
left and zero right clearance events above the existing 5 mm threshold. The
right sole never exceeded 4.82 mm and its P95 height was 2.52 mm, so this is
not merely an unexplained missing-event field. Low-speed alternating clearance
has not been retained. The candidate fails the stricter quality screen and is
not selected for hardware.

Checkpoint SHA256 is
`f29fc064da0e81d6ebfce0740258ca194369ec05064c4c7fdadc0e7301dd3a8b`.
Its local copy was verified against the evaluation report. The evaluation,
training metrics, configuration and four-panel video are preserved under
`outputs/targeted-upright-clearance-20261118/evidence/`. The standard video shows
zero command, forward 0.25 m/s, left turn and right turn; the low-speed case
is present in the saved rollout and reports but is not one of those panels.

## Per-foot regression audit and preservation screen

The user requested completion of the three-arm comparison without losing the
improved standing posture or spacing. The training objectives and schedules
remain unchanged. A separate bounded read-only monitor reconstructs each
completed rollout's arrived-at state from logged qpos, the next observation's
joint velocities and gyro, and measured root velocity. It checks joint-state
alignment and raw previous-action alignment before applying the same analytic
sole-center velocity calculation used in the new reward. Contact forces are
the logged substep averages; the loaded threshold is the existing 5 N diagnostic
convention, not a new controller condition.

For the parent5000 and upright-clearance600 rollouts, respectively:

| Diagnostic | Parent5000 | Upright-clearance600 |
| --- | ---: | ---: |
| Low-speed right-foot unloaded median horizontal speed | 0.331 m/s | 0.375 m/s |
| Right-turn right-foot unloaded median horizontal speed | 0.415 m/s | 0.452 m/s |
| Low-speed left-foot loaded fraction | 56.5% | 84.1% |
| Low-speed right-foot loaded fraction | 60.9% | 69.1% |
| Low-speed right-foot mean proposed clearance cost | 0.01275 | 0.02301 |
| Right-turn right-foot mean proposed clearance cost | 0.01176 | 0.01565 |

The proposed clearance cost is evaluated on both recorded trajectories; it
was not part of the parent's original objective. These results do not support
the simple explanation that the low foot was missed by the new cost or that
the new policy merely slowed its unloaded swing to evade that cost. The low
foot is penalized more, but remains low. Changed support/swing allocation is
observed, and competing reward gradients, latent-controller adaptation and
world-model error remain possible explanations pending controlled results.
Reward-input-only counterfactuals for +5 mm height and half foot speed are saved
with explicit warnings that they are not physically realized interventions.

The additional selection screen retains the clearance600 standing pitch within
0.3 degree and each case's P05 center spacing within 5 mm, with no crossing.
It requires 95% of clearance600 bilateral forward/left-turn swing peaks and
95% of parent5000 bilateral low-speed/right-turn peaks, plus 85% of parent step
counts. These are candidate-selection tolerances, not changes to protection,
training gates or the three reward arms. Passing also requires the original
quality screen; neither screen alone qualifies a model for hardware. Synthetic
regressions confirmed that worse standing pitch and narrowed spacing are
rejected, and the current low-speed/right-turn failures remain visible.

The local job-bound monitor writes `progress-comparison.json`, per-checkpoint
retention reports and foot-reward audits into the same evidence directory.
It runs until the comparison reaches a terminal state and all completed
evaluations are audited, or the rental backup deadline. Checkpoints and videos
continue to be transferred by the existing independent evidence copier.

## Complete three-arm 600-update comparison

All three arms completed 600 fresh updates and survived all five nominal
20-second cases. None passes the combined quality and retention criteria.

| Metric | Original-reward control | Upright only | Upright plus clearance |
| --- | ---: | ---: | ---: |
| Zero-command backward pitch magnitude, degrees | 3.73 | 2.86 | 1.93 |
| Right-turn right-foot median swing peak, mm | 12.21 | 10.37 | 9.87 |
| Low-speed L/R clearance events above 5 mm | 22 / 21 | 20 / 1 | 7 / 0 |
| Low-speed measured forward speed, m/s | 0.146 | 0.104 | 0.125 |
| Forward 0.25 m/s L/R swing peaks, mm | 17.34 / 18.58 | 11.04 / 13.60 | 16.58 / 16.82 |
| Left-turn L/R swing peaks, mm | 13.82 / 15.81 | 8.73 / 10.83 | 15.23 / 16.28 |

Clearance events use the unchanged 5 mm threshold and a minimum duration of
two 50 Hz samples, over the scored interval after three seconds. An event
count of zero means no qualifying lift event, not absence of joint movement.
All three retain widened feet; moving-case P05 center spacing stays at or above
113.1 mm. The unchanged-reward control also loses some low-speed and right-turn
clearance relative to parent5000, so continued transfer training itself is an
important comparison. The single-seed results suggest that strengthening the
upright objective contributes to low-speed gait deterioration, while the added
clearance term recovers forward and left-turn clearance but not the two target
failures. Longer stages and the controlled follow-up remain necessary before
generalizing the effect.

A new local nine-panel video, `three-arms-600-focus.mp4`, shows columns in the
order control / upright / upright-clearance and rows zero / forward0.15 /
right-turn. It renders the original physical qpos, sampling every second frame
at 25 fps for real-time playback. No new dynamics rollout, motion smoothing,
reference substitution or training GPU use is involved. The local XML hash was
checked against the training host before rendering. The paired source hashes
and panel mapping are recorded in the video manifest. Original four-panel
videos and checkpoints remain unchanged.

## Complete three-arm 1600-update comparison

All three checkpoints and their five nominal 20-second rollouts are available
locally with verified checkpoint SHA-256 values. All nominal cases survived,
but none passes the complete quality and retention criteria.

| Metric | Original-reward control | Upright only | Upright plus clearance |
| --- | ---: | ---: | ---: |
| Zero-command backward pitch magnitude, degrees | 3.58 | 1.69 | 2.13 |
| Zero-command median knee flexion magnitudes L/R, degrees | 32.3 / 14.7 | 37.8 / 14.0 | 38.6 / 15.3 |
| Low-speed L/R clearance events above 5 mm | 19 / 20 | 16 / 13 | 26 / 26 |
| Low-speed L/R median swing peak, mm | 12.1 / 9.5 | 8.4 / 6.0 | 9.9 / 8.9 |
| Low-speed measured forward speed, m/s | 0.151 | 0.152 | 0.170 |
| Right-turn L/R median swing peak, mm | 12.5 / 16.9 | 11.7 / 13.7 | 13.0 / 14.3 |
| Right-turn P05 foot-center spacing, mm | 100.5 | 127.0 | 123.6 |

Knee values are magnitudes of signed model joint readings; the right knee has
the opposite axis sign. These values are not a separate anatomical-angle
measurement. Equal event counts alone do not establish phase symmetry or a
fully satisfactory alternating gait.

The upright-clearance arm recovered bilateral low-speed lift events from 7/0
at update 600 to 26/26. Right-turn right-foot swing peak increased from 9.87 to
14.3 mm, with measured yaw rate -0.192 rad/s for a -0.2 rad/s command. Low-speed
P05 foot-center spacing remained 124.2 mm. However, zero-command knees remained
crouched and asymmetric. Forward 0.25 m/s clearance regressed to 11.4/20.4 mm,
with measured speed 0.185 m/s. It is a promising diagnostic candidate, not a
combined-success or hardware-qualified model.

Actual motion-interface rollouts of the same three checkpoints produced
approximately 24-25 mm bilateral swing peaks. This supports the feasibility
of the corrected reference height, while the velocity-interface shortfall
remains. Different initial conditions and objectives prevent attributing the
gap to a single cause from this comparison alone.

The recorded-qpos nine-panel video is
`outputs/targeted-upright-clearance-20261118/evidence/three-arms-1600-focus.mp4`.
Columns are control / upright / upright-clearance; rows are zero / forward0.15 /
right-turn. It contains 500 frames at 25 fps, 1440x1080, with 20 seconds of
real-time playback. Rendering does not recompute physics. Its manifest records
checkpoint hashes and panel mapping. The standing joint audit is saved as
`standing-knee-arm-audit-1600.json` in the same evidence directory.

The original three arms continue unchanged to update 3000. The queued follow-up
will select its parent using all completed results. If bilateral low-speed
stepping is retained but standing knee flexion remains excessive, it compares
a matched continuation against a soft knee-flexion cost gated to near-zero
commands. Its 20-degree deadband avoids rewarding locked straight knees, and
the command gate is negligible at 0.15 m/s and during the tested turn. Indirect
walking effects must still pass the original gait, posture and spacing checks.
No model, gain or hardware protection setting is changed on the robot.

## Complete three-arm 3000-update comparison

All three arms completed, and all five nominal cases survived. None passes
combined quality and preservation checks. More updates did not yield a
uniform improvement. Measurements below use seconds 3-20 of physical rollouts.

| Metric | Original reward | Upright only | Upright plus clearance |
| --- | ---: | ---: | ---: |
| Standing knee flexion magnitude L/R, degrees | 32.5 / 14.0 | 35.3 / 13.5 | 35.8 / 13.8 |
| Standing backward torso pitch, degrees | 3.99 | 1.39 | 1.37 |
| Low-speed effective lift events L/R | 21 / 8 | 20 / 19 | 0 / 0 |
| Low-speed swing peaks L/R, mm | 12.82 / 8.04 | 9.46 / 5.49 | No qualifying events |
| Low-speed mean forward velocity, m/s | 0.138 | 0.097 | 0.012 |
| Forward 0.25 m/s swing peaks L/R, mm | 12.27 / 20.17 | 16.05 / 17.76 | 18.24 / 16.86 |
| Forward 0.25 m/s measured velocity, m/s | 0.182 | 0.185 | 0.208 |
| Right-turn swing peaks L/R, mm | 13.90 / 12.83 | 13.99 / 11.74 | 14.42 / 13.26 |
| Right-turn mean yaw rate, rad/s (command -0.2) | -0.187 | -0.173 | -0.219 |
| Right-turn P05 foot-center spacing, mm | 127.0 | 121.6 | 117.3 |

The upright-clearance arm recovered forward0.25 symmetry from 11.4/20.4 mm
at update 1600, but lost low-speed walking. Its low-speed last-five-second
forward velocity is approximately 0.00009 m/s. Standing left knee flexion
decreased by only 2.8 degrees and remains excessive. Right turning remains
functional, but right-foot swing peak decreased from 14.3 to 13.3 mm and
P05 spacing decreased from 123.6 to 117.3 mm. Standing P05 spacing decreased
from 130.4 to 122.0 mm and fails the existing preservation screen. There are
no crossed-foot samples in these nominal cases; this does not establish that
spacing was fully retained. Keep the update-1600 checkpoint rather than
replacing it with update 3000.

The saved moving-sole audit provides a new diagnostic lead for the low-speed
collapse. Upright-clearance loaded fractions increase from approximately
65%/60% at update 1600 to 99%/98% at update 3000. Mean per-foot added costs
fall from 0.0139/0.0189 to 0.00115/0.00288 as motion disappears. By definition,
the moving-sole cost vanishes at zero foot velocity. Also, the implemented
linear-velocity tracking term is `2 * exp(-squared_error / 0.5**2)`; a stopped
robot commanded to 0.15 m/s still receives about 91.4% of that term's maximum.
Together these identify an objective tradeoff worth testing. They do not prove
that the added sole cost caused the collapse: policy/decoder drift, model
prediction error and other penalties have not yet been separated by a full
reward-component and imagined-versus-physical audit. Do not increase the sole
cost blindly in response to this stopped trajectory.

The twelve-panel original-speed video is
`outputs/targeted-upright-clearance-20261118/evidence/three-arms-3000-four-cases.mp4`.
Columns are original reward / upright / upright-clearance; rows are zero /
forward0.15 / forward0.25 / right-turn. It renders recorded physical qpos.
The accompanying manifest identifies the checkpoints, and
`three-arms-3000-and-standing600-audit.json` includes verified checkpoint
hashes and numerical summaries of these three results plus the two follow-up
600-update results.
