# R1 extended Xbox trials: operator feedback and initial checks

The operator completed two further trials and reported acceptable overall
dynamics, mild residual torso lean, slow initiation, and more varied responses
to backward pulling than the previous AMP policy. Earlier feedback described
light foot contact. These are qualitative observations, not controlled comparisons.

The completed evidence was copied locally to
`outputs/deploy-sm-walk-r1-xbox120-20261119/powered-through-144201.tar.gz`.
Its verified SHA-256 is
`2598500e6bebdb5f7a2de6dd71446ee166f98ea0ddcd26838edc162bd0d33036`.
Collection began only after no device-control process remained.

## Session outcomes

Both `143828-4975` and `144201-5126` completed 7,300 policy action groups and
confirmed all 31 motors disabled. The latter passed all commissioning checks.
The former completed physical control but failed its post-run TX audit:
sequence 1526 had 19 first-send completion receipts at or above 20 ms from
the oldest feedback, with maximum 20.377288 ms. Its maximum driver handoff age
was 19.560669 ms. Receipt time is an upper bound on physical transmission,
not a motor execution timestamp. This failure remains recorded; physical
completion must not be conflated with passing the timing audit.

## Waist roll

Current joint gains match training: Kp 60 Nm/rad and Kd 3 Nm*s/rad. No gain
change was made during this review. For samples with forward command above
0.08 m/s and absolute yaw command below 0.03 rad/s:

| Measurement | 143828 | 144201 |
| --- | --- | --- |
| Selected duration | 18.34 s | 29.46 s |
| Mean waist target minus measured angle | +0.075 deg | +0.230 deg |
| 5th to 95th percentile target error | -3.406 to +3.383 deg | -3.114 to +3.542 deg |
| Mean reconstructed torso left lean | -1.320 deg | -0.538 deg |
| Maximum whole-session waist feedback torque | 17.377 Nm | 15.736 Nm |
| Waist command saturation count | 0 | 0 |
| Waist command and feedback cap | 87.3 Nm | 87.3 Nm |

These samples do not show a large persistent signed waist tracking error.
Errors of several degrees during motion still exist and can cancel in an
average; mean error alone cannot rule out insufficient dynamic stiffness.
Torso lean also contains pelvis lean and the policy's desired waist motion.
The measurements do not justify treating gain increase as an established fix.
Disturbance and sling-load timestamps are not available, so the selected
segments are not matched undisturbed trials.

## Initiation

The unchanged command acceleration is 0.05 m/s^2. A full stick step from rest
takes approximately five seconds to request 0.25 m/s and eight seconds to
request 0.40 m/s. This is a command ramp, not a gain ramp or inference delay.
It can contribute substantially to perceived slow initiation, but does not
measure actual walking onset. A future comparison should first separate
command rise from leg motion and change one factor at a time. No acceleration,
model, gain, saturation or safety setting was changed in this review.

More varied perturbation responses alone do not establish better or worse
robustness than AMP. Controlled comparisons require recovery time, falls,
foot clearance and posture after disturbance, together with matching commands
and support conditions. Preserve the reported light foot contact and acceptable
dynamics when assessing subsequent changes.
