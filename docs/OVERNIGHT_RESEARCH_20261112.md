# November 12 controlled locomotion investigation

This record separates completed evidence from running work. All experiments
use the pinned Sprite v5 model, full published joint gains, 500 Hz PD, 50 Hz
policy, and the verified pelvis IMU convention. No Jetson or motor operation is
part of this overnight investigation.

| Experiment | Reference | Motion reward | State |
| --- | --- | --- | --- |
| Original PM01 baseline | Original G71 | Existing v2 reward | Preserved through update 10,209; update 6,030 is the equal-budget comparator |
| Reference-only comparison | Reviewed 25 mm PM01 clearance correction | Existing v2 reward | Completed update 6,030; final physical audit and local backup complete |
| Sole-height comparison | Same exact corrected PM01 bank | Bounded opt-in sole term | Started November 12 at 21:13 China time after final reference-only review |
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

The final update-6,030 result improves substantially over the earlier snapshot.
At the same update count and evaluation seed 101:

| Physical measurement | Original PM01 | Corrected reference only |
| --- | ---: | ---: |
| Forward command / measured speed | 0.25 / 0.216 m/s | 0.25 / 0.217 m/s |
| Forward left/right sole maximum | 0.20 / 2.11 mm | 14.91 / 8.08 mm |
| Sustained bilateral events above 5 mm | 0 / 0 | 21 / 19 |
| Loaded near-floor material-point slip | 0.248 / 0.186 m/s | 0.012 / 0.032 m/s |
| Zero-command mean forward drift | 0.067 m/s | 0.0048 m/s |
| Reverse command / measured forward speed | -0.15 / +0.061 m/s | -0.15 / -0.185 m/s |

Both survive all 48 seed-101 cases, illustrating why survival alone misses
dragging. The final corrected-reference run stops almost completely in the last
five seconds of the zero-command case. Reverse has residual yaw error, right
turns and lateral movement remain imperfect, and the forward right-foot event
median is only 7.1 mm. The corrected data helps; it does not yet meet the desired
gait quality. This is one training seed, not a replicated statistical result.

The isolated sole-height experiment starts fresh from the same seed and exact
reference bytes, targeting 6,030 updates. Its activation review binds the final
physical metrics and sampled command-video frames by hash. Source commit is
`3f3667e51ce2a5f19406b3930f415b00bac219fa`; source snapshot SHA-256 is
`8e33d6cbb415e84cd6306587763228e5544b6789f13c0d7a16e1fc9a47277fd9`.
The run is under `supermimic_sole_experiment/outputs/pm01-clearance-sole-height-20261112-125806`
on the rented host. Bounded hourly local backups use
`outputs/retrain-20261112/pm01-sole-height-training`.

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

Longer follow-up experiments can request exact `checkpoint_iterations` in their
configuration. These snapshots survive ordinary retention; the evaluation
watcher accepts `--required-iterations` and prioritizes them before regular
latest-checkpoint reviews. This preserves an exact update-6,030 comparison
while allowing training to continue. It changes checkpoint scheduling only,
not optimization or the active isolated sole-height experiment. Eight focused
tests cover retention, delayed polling, final draining and invalid inputs.

Recorded rollout rendering optionally uses a brighter floor and closer camera.
The physical rollout and telemetry remain unchanged; these are playback views,
not newly simulated trajectories or reference animation substituted for policy.

The audit path now supports packed multi-clip libraries. It reads the named
pelvis instead of requiring an unpacked `qpos` field, measures each reference
without crossing clip boundaries, and scores each motion environment only
until its own clip ends. A quiet first clip can no longer stand in for the
entire walking library. Two regression tests and real two-clip software
rollouts exercise these cases; software-smoke gait quality is not reported as
a training result.

Held-out motion evaluation requires an explicit original training-bank path,
checkpoint/reference identity, both numeric and rendered qualifications, and
hash-bound source manifests with a one-second cross-split gap. Ordinary
evaluation still rejects changed references. Final validation banks are
evaluation-only, and training explicitly rejects the validation split even
for a software smoke. This changes research evaluation, not actor inputs,
gains, dynamics or active PM01 runs.

The final corrected-reference checkpoint also survives all 48 seed-101 cases
from both the controller default pose and the URDF zero pose, with physical
velocities zeroed and no failure reset. Forward 0.25 m/s cases measure
0.215 and 0.214 m/s respectively. This is broader simulated startup evidence,
not a live motor-space or supported-handoff test.

Its 993-input ONNX candidate passes replay parity on 512 recorded inputs
covering all 48 command/disturbance environments. Reconstructed ten-frame
histories include the previous raw clipped action; both PyTorch and ONNX
match the recorded actions within 1.91e-6 maximum absolute action error.
The exported contract remains hardware-unqualified. A one-thread CPU test
on the rented host's Xeon Platinum 8480+ measures approximately 0.67 ms median;
this is explicitly not Jetson latency, IPC latency or CAN timing. The replay
verifier is reusable for the eventual selected checkpoint.
