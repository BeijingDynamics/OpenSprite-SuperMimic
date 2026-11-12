# PM01 reference-only clearance comparison

The original PM01 experiment learns very low physical foot clearance despite a
high motion reward. This experiment changes only the reference trajectory,
while retaining full joint SuperMimic, seed 43, all optimization settings,
published gains, action scale, 500 Hz PD and 50 Hz policy updates. Added world
model sole diagnostics are read-only. No hardware parameters are changed.

## Qualified reference

The source is the previously reviewed PM01 G71 Sprite motion. Frames 20 through
591 form an 11.42 s segment with 14 swings per foot and double-support endpoints.
Support phases are inferred from source kinematics, not measured contact forces.
The correction preserves source cadence, all non-leg joint angles and root
orientation exactly. It adjusts twelve leg joints and bounded root translation.
Sparse trajectory optimization enforces joint, velocity, planted-foot and
root-acceleration constraints. A natural knee-flexion branch avoids IK
hyperextension; this is a reference constraint, not a runtime safeguard change.

The independently reproducible candidate has SHA-256
`7c6dccdc87756956f58ed9c5c6c7432882172abc62f116b803f719ecc16e6870`.
It passes all eight numerical gates, independent FK and contact checks:

| Measurement | Left | Right |
| --- | ---: | ---: |
| Peak lowest-sole height | 24.79 mm | 25.31 mm |
| Inferred-support material-point slip RMS | 2.36 mm/s | 2.57 mm/s |
| Maximum foot tilt | 4.10 degrees | 5.17 degrees |

Maximum support residual is 0.318 mm. Maximum adjacent-frame joint speed is
70.04% of its published bound. Root acceleration is bounded at 3 m/s² per
component, with less than 0.001 m/s² numerical tolerance. Independent FK position
agreement is within 0.72 micrometres. There is no detected ground or self
penetration, and no frame with both soles above 5 mm.

Front and side rendered frames were reviewed across the first gait cycle and
at 0.48, 2.08, 4.32, 6.48, 8.68 and 10.88 seconds. The upright torso and arms beside
the body are retained, and alternating swing clearance is visible. This review
qualifies a kinematic research reference, not a dynamically stable learned gait.
The original numeric audit remains immutable; a separate hash-bound visual
review and training QA record admit this exact candidate.

```bash
python scripts/prepare_pm01_clearance_candidate.py \
  --source outputs/pm01-baseline-20261112-064405/reference/pm01_g71_v5_50hz.npz \
  --output outputs/pm01-clearance-reproduced \
  --trajectory --root-acceleration-bound 3.0
```

This is an engineered PM01-derived reference, not unmodified motion capture or
the paper's locomotion library. Preserve the PM01 provenance and BSD notices;
the repository's AGPL license does not replace the data license.

## Controlled training and evaluation

`configs/velocity-v2-pm01-clearance-reference-only.json` starts fresh and stops
at 6,030 updates or four hours, whichever occurs first. Compare against the
preserved original-reference update-6,030 checkpoint, rather than a checkpoint
trained for longer. The original run and its final checkpoint are retained.

The evaluator checks motion tracking and command control separately, physical
sole geometry, contact slip, speed and yaw response, and finite-force recovery
over three seeds. Inspect bilateral repeated steps, not just peak height or
survival. World-model validation reports sole-height errors and geometric
missed/false swings at multiple horizons; these are not contact-force estimates.

100STYLE Neutral is a separate data candidate. It is not mixed into this
reference-only comparison. If the corrected reference alone is insufficient,
any reward or dataset expansion requires a separately identified experiment.
