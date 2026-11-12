# Foot clearance is a blocking locomotion issue

The operator reports the same low-clearance problem in the previous deployed
SuperMimic policy and requires the new training to resolve it. Forward speed,
survival, tracking reward and world-model prediction loss cannot individually
qualify walking. The current PM01 research baseline is not gait-qualified.

## Frozen evidence

Run: `pm01-baseline-20261112-064405`, iteration 7306, source commit
`818792eefe3148141dee13546a9e1a23d352551a`.
The motion interface follows the 11.98-second reference. The velocity interface
receives zero for two seconds and then 0.25 m/s. The scored intervals start at
three seconds and stop before any failure. Neither rollout fell.

| Quantity | Reference | Motion interface | Velocity interface |
| --- | --- | --- | --- |
| Absolute maximum sole height, left/right | 14.99 / 13.91 mm | 1.26 / 2.05 mm | 1.25 / 0.17 mm |
| Maximum height above the lower foot, left/right | 5.76 / 6.00 mm | 1.87 / 2.79 mm | 2.12 / 1.09 mm |
| Loaded near-floor material-point RMS planar speed, left/right | No measured forces | 0.138 / 0.219 m/s | 0.145 / 0.234 m/s |

The reference's minimum absolute sole heights are 3.64 and 4.63 mm. Therefore
its approximately 15 mm absolute peak is **not** a 15 mm stance-to-swing
clearance. This corrects the earlier interpretation of that number. Relative
height removes common vertical translation but is not a force-derived support
label. Both feet being low does not by itself prove a slip; the physical force
and near-floor material-point motion provide additional evidence here.

Loaded intervals require measured mean vertical force above 5 N at both
endpoints. Material points are the same rigid-box corners at both endpoints,
selected by height <=2 mm, never by a low-speed cutoff. This reduces the
foot-centre rolling ambiguity but remains a 50 Hz interval estimate, not a
solver contact-point measurement. The 5 mm event counter is diagnostic and
must not become the sole gait-acceptance criterion.

## Why the current reward does not reject this behavior

The physical motion-interface rollout earns a mean normalized tracking reward
of **0.98896** despite the low-clearance result. Mean body-position RMS error
is 11.70 mm. The motion reward averages body-position error over tracked
bodies and uses a 0.30 m scale. Analytically removing only the two foot-origin
Z errors changes its raw body-position reward by about **0.0000782** on
average. This reward-only counterfactual is not a physically achievable pose
or a complete causal ablation. It does demonstrate weak discrimination of
these errors by that term.

The velocity decoder receives its weight updates from the motion interface;
velocity learning cannot repair those decoder weights directly. Since the
motion interface itself exhibits the failure, changing Xbox input, runtime
send frequency or deployment action scale is not an adequate explanation or
a demonstrated repair. This does not establish that all historical dragging
had the same cause.

The reference was used as an unconditional AMP style prior in the old Xbox
recipe, together with task/contact objectives. Its success there does not
establish suitability for exact time-indexed whole-body tracking. See the
[pinned engineering handoff](https://github.com/BeijingDynamics/open_sprite_rl/blob/63a82f744c78d48ee42f557fb472d7499702ee94/docs/SPRITE0825_STABLE_WALK_HANDOFF.en.md).

## Repair sequence and acceptance

1. Preserve the approved PM01 upper-body style, source data and baseline.
   Audit support phase and rigid-sole geometry before constructing a separate
   contact-consistent reference candidate. A whole-clip height translation
   cannot repair alternating contact or create swing clearance. Do not equate
   a velocity-filtered contact proxy with measured low slip.
2. Check and render any reference correction before long training. Recompute
   derivatives and FK, enforce the same joint/velocity/collision constraints,
   preserve provenance, and check both foot headings and upper-body posture.
3. Compare reference-only correction with an explicit motion-interface
   foot-tracking refinement in isolated runs. Any new term is an embodiment
   extension, not a verbatim paper reproduction. Do not reward absolute
   airborne height against the currently floating support targets, introduce
   fixed-clock stepping into the velocity actor, or add a runtime lift offset.
4. Require repeated alternating physical swings, bilateral per-step minimum
   toe/heel clearance, bounded loaded material-point sliding and stable
   touchdown. Evaluate these with command tracking, stops, turns, asymmetric
   dragging, pushes and pulls. Reject hopping and persistent single-foot
   shuffling even if an average clearance or speed improves.
5. Validate world-model foot/contact transition errors as well as joint/root
   averages before attributing every failure to reward design. The current
   reward audit alone does not exclude model exploitation or dataset gaps.

Full joint SuperMimic, 500 Hz PD, 50 Hz policy, full published gains, action
scale and hardware protections remain unchanged. No repaired gait is claimed
by this diagnosis. The original training continues as a preserved baseline;
no new reference or reward has been silently substituted into it.

## Reproduction and local evidence

`scripts/audit_sole_tracking.py` accepts the prepared reference and immutable
physical motion/velocity rollouts. It checks next-observation joint feedback
and raw previous-action alignment before reconstructing the motion reward.
`scripts/test_sole_audit.py` covers stationary contact, sliding, alternate
swings, single-frame spikes, unloaded translation, elevated corners and a
common vertical offset. The checkpoint watcher has an opt-in `--sole-audit`
mode for the same checks on subsequent checkpoints.

The periodic evaluator was restarted with this option at 2026-11-12 18:07:58
Asia/Shanghai. The trainer was not restarted or changed. The evaluator retains
the original deadline and completed-audit history; its restart manifest records
the diagnostic source hashes. All seven synthetic checks and the recorded
feedback/action alignment checks passed on the rented host.

Local evidence is under
`outputs/retrain-20261112/progress-1748/sole-tracking-audit-v2.json` and
`standing-forward-turns.mp4`. Data and rollouts remain outside Git.
The frozen checkpoint and both rollouts are also retained under
`outputs/retrain-20261112/progress-1748/frozen-7306`, together with the final
alignment-checked `sole-tracking-audit-v3.json` and local SHA-256 manifest.
Reference SHA-256:
`94f90f396360c3c019475b6f9be47d0430ab0f8306c8b75b17710b14130faf06`.
