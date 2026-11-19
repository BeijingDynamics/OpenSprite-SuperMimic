# Actuator candidate review: November 14, 2026

Status: review only. The user explicitly requested inspection before training.
No new actuator training has started and no actuator parameters were changed.
Do not launch this experiment until the user confirms the reviewed model.

## Preserved policy candidates

Local files were hashed again on November 14:

- `outputs/retrain-20261114/seed45-lr-continuation/actor-quarter/training/recovery-audit-009000/checkpoint.pt`
  SHA256: `50a6aa050b9b3f6c78fff4e0dcd60e5cbc87a3cfb5912a6a0f6517c49e1d9492`.
- `outputs/retrain-20261114/seed45-lr-continuation/actor-quarter/training/recovery-audit-010000/checkpoint.pt`
  SHA256: `aab860df068d5ccc67aa94ea315f5517dd333373c0e7be52fe6344bb5a98ee8a`.

These match the reviewed shortlist. They are training checkpoints, with local
evaluation/configuration/video evidence, not a newly qualified hardware bundle.

## Current plant versus candidate components

The completed training uses 500 Hz PD, a 50 Hz policy, original per-joint
Kp/Kd and constant effort caps, command delay of zero or one physics tick
(0--2 ms), no active torque-response filter, and zero joint frictionloss.
Armature comes from the pinned asset and is not a measured identification.

For direct joints, retain the policy position target and PD controller:

```
q_target = q_default + 0.25 * action
u = clip(Kp * (delayed_q_target - q) - Kd * qd, -effort_cap, effort_cap)
motor_torque_next = motor_torque + (1 - exp(-dt / tau)) * (u - motor_torque)
joint_drive = motor_torque_next - physical_friction(qd)
```

The filter is optional. A 5--10 ms time constant is a sensitivity hypothesis,
not a measured DAMIAO response. It must be isolated from friction in ablations.
Physical friction belongs in the plant, not in policy Kd and not as runtime
feedforward. Static friction near zero speed requires a proper friction
constraint/model; a naive sign function alone does not model stiction.

## Concrete external evidence

[MABEL J4340P identification output](https://github.com/robotmabel/open_mabel/blob/main/calibration/motor_calibration/results/DM-J4340P/params.json)
was re-read from the primary repository for this review. It reports:

| Quantity | Author-reported value | Limitation |
| --- | ---: | --- |
| Positive/negative Coulomb friction | 0.333081 / 0.345377 Nm | Different or unpinned hardware revision |
| Positive/negative static friction | 0.349885 / 0.382287 Nm | Requires near-zero-speed validation |
| Viscous coefficient | 0.183973 Nm s/rad | Plant damping, not controller Kd |
| Stribeck velocity | 0.087989 rad/s | Bench fit, not our installed hardware |
| Reflected inertia | 0.0259001 kg m^2 | Inertia fit residual 0.4235 Nm RMS |

The source also supplies a simpler MuJoCo approximation: frictionloss
0.339229 Nm and passive damping 0.183973 Nm s/rad. This approximation omits
direction asymmetry and the Stribeck curve. Its 27 Nm datasheet peak differs
from the 40 Nm revision in our hardware map. Neither the old peak nor its
reported no-load speed should replace our installed-revision values.

The inspected direct J4340P mapping has 13 joints: bilateral hip pitch/roll/yaw,
knees, shoulder pitch/roll, and waist yaw. It does not justify applying these
friction numbers to J4310P, J3507, J6248P or parallel mechanisms. MABEL's other
identified J4310 is not J4310P. No revision-matched identified model for all
four installed families has been established in this search.

## Proposed review decision

First consider a narrowly scoped friction experiment using the published
MuJoCo approximation on those 13 direct joints, with the existing armature,
gains, effort caps and command delay retained. Treat friction as an uncertain
external prior; agree its range before implementing it. Keep any optional
torque-response experiment separate so its effect can be identified.

Do not yet combine borrowed inertia, unmeasured time constants and guessed
voltage-dependent speed curves into a claimed calibrated actuator model.
The 38 V system supply versus 48 V datasheets and physical parallel ankle/head
transformations remain explicit gaps. Actual motor identification can later
replace the external priors.

After approval, preserve the final PM01 recipe, seed, reference hash, rewards,
network, observations and learning-rate stages. Train the world model and
policy afresh under the changed plant, and compare both nominal and changed
dynamics. The existing shortlisted checkpoints remain untouched.
