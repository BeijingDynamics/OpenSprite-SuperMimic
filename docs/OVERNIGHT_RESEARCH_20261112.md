# November 12 controlled locomotion investigation

This record separates completed evidence from running work. All experiments
use the pinned Sprite v5 model, full published joint gains, 500 Hz PD, 50 Hz
policy, and the verified pelvis IMU convention. No Jetson or motor operation is
part of this overnight investigation.

| Experiment | Reference | Motion reward | State |
| --- | --- | --- | --- |
| Original PM01 baseline | Original G71 | Existing v2 reward | Preserved through update 10,209; update 6,030 is the equal-budget comparator |
| Reference-only comparison | Reviewed 25 mm PM01 clearance correction | Existing v2 reward | Fresh seed 43, running to update 6,030 |
| Sole-height comparison | Same exact corrected PM01 bank | Bounded opt-in sole term | Separate worktree; tests and three-update smoke passed; not launched |
| Neutral library | Source-preserving screened 100STYLE clips | Not selected yet | Retargeting and independent source/geometry review; excluded from active runs |

The original and reference-only jobs live on the rented host under
`/root/gpufree-data/supermimic_velocity_v2/outputs/`. The separate reward source
is `/root/gpufree-data/supermimic_sole_experiment`. Its local worktree is
`E:/supermimic_run/OpenSprite-SuperMimic-sole-experiment`, branch
`supermimic_sole_tracking`, commit `3f3667e`.

The first reference-only physical review at update 1,101 was not a successful
repair: motion tracking reached 11.3/5.0 mm peak left/right sole heights, but
the 0.25 m/s velocity case reached only 2.1/1.3 mm, with no sustained 5 mm events.
Its mean forward speed was 0.153 m/s and zero-command drift was 0.131 m/s.
All 48 cases survived under each of three seeds; this does not establish good
walking. Do not confuse lower sliding caused by slower travel with better gait.

At update 3,769, the reference-only run shows a meaningful forward improvement:
the 0.25 m/s case averages 0.230 m/s, reaches 10.5/10.3 mm peak sole heights and
has 23/24 sustained events above 5 mm. Loaded material-point slip is approximately
0.022/0.012 m/s. Motion tracking reaches 15.0/13.3 mm. However, zero command drifts
forward at 0.279 m/s and reverse also moves forward. Forward gait improvement
does not qualify command control. Final equal-budget review must decide whether
residual clearance or missing standing/backward/lateral reference coverage is
the next experimental priority.

At update 3,319, held-out world-model replay diagnostics improved to roughly
0.9--1.6 mm sole-height RMSE at 32 steps, depending on side/interface. These
are same-recorded-action open-loop predictions, not proof of accurate arbitrary
policy imagination or contact-force prediction. Physical gait still decides.

The next reward experiment is only activated after the completed reference-only
rollouts are reviewed. It must start fresh and keep the same reference and
ordinary configuration. The smoke checkpoints have identical physical
contracts; their configuration differences are limited to experiment labels,
the activation note and the explicit reward extension.

Local hourly backups preserve checkpoints, source snapshots, qualified
references, physical rollouts, videos and reports under
`outputs/retrain-20261112/pm01-clearance-training`. The stopped baseline is in
`outputs/retrain-20261112/pm01-recovery-training`. Neutral derived artifacts
and provenance are retained separately; generic downloadable BVH data is not
part of the local backup or Git publication.

Before accepting a later run, inspect bilateral repeated clearance, loaded
material-point sliding, zero-command drift, forward/reverse/lateral/yaw
response, starts/stops, foot headings and finite-force recovery. Inspect source
arm/head fidelity as well as feet. No reference or simulated checkpoint in this
record is automatically qualified for hardware deployment.
