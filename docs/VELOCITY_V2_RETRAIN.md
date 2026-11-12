# Commanded locomotion retraining, November 2026

This isolated experiment follows the SuperMimic paper's joint motion-tracking
and reinforcement-learning velocity interfaces (Appendices B/G). It is not the
official release's tracking-reward velocity distillation configuration.

## Preserved baseline

- Branch: `supermimic_velocity_v2`, starting at `027fd2e`.
- Robot: `sprite0825_v5_4340_shoulders_waist20261105`.
- Control profile SHA-256:
  `184ec989fd24b4bc6ea5034e6fdae796b5b9ce83ef2c884dfe4fc61d8d6987dc`.
- Training PD: 500 Hz; policy: 50 Hz; published full joint gains, effort caps,
  default positions and action scale 0.25 remain together.
- The previous trained model and powered-test evidence remain unchanged.
- The deployed runtime baseline remains mixed-rate transmission, not the
  experimental all-joint 500 Hz branch. This experiment does not operate motors.

## Pinned algorithm sources

- C. Zhang: `zita-ch/supermimic_vibed`,
  `2a26b2f8c4d616e16361a616fe659c7d4acdf317` (MIT).
- Official: `HybridRobotics/supermimic`,
  `abc124c1bd5976341034668af3d12c087f2f1f55` (Apache-2.0).

The repository remains AGPL-3.0. Existing upstream notices are retained.
New algorithm features are opt-in, so legacy checkpoint interpretation does
not change. A v2 checkpoint has a distinct schema even though the velocity
input remains 993-dimensional.

V2 predicts local root velocity increments and joint velocity increments,
then integrates position and orientation over the 20 ms control step. The
semi-implicit ten-substep coefficient is 0.55. The world-model loss explicitly
supervises root position, rotation, linear/angular velocity, joint position
and velocity with nominal physical tolerances. Sprite's 66-dimensional
actuator memory is retained and predicted as an increment; its loss tolerance
is explicitly 0.1 in the existing normalized memory units. This is an
embodiment-specific extension, not an upstream claim.

Actor and critic hidden layers use LayerNorm/GELU. Motion goals use ten
future samples at offsets 1, 6, ..., 46; orientation is relative to the current
robot state. The motion critic additionally receives relative future root
positions. Clip-end padding never borrows the next clip's velocities.
Velocity latent alignment evaluates a detached teacher at the current
imagined state; a frame-only latent cache is invalid for these goals.

Both interfaces retain separate critics, a shared decoder and world model,
equal environment/imagined-batch allocation, ten history frames and
reference-command-only latent alignment. Velocity reward cannot update the
decoder or motion encoder weights, but gradients still pass through frozen
decoder inputs and dynamics. Critic values are bounded above by the task's
maximum discounted reward; negative values are not clamped to zero.
The first 500 training iterations fit dynamics only. This is simulator
training and must not be confused with zero-gain policy operation on hardware.

## Reference selection

Update after the November 12 video review: the operator approved PM01/G71 as
the primary ordinary-walk reference style and rejected the displayed LAFAN/UMR
upper-body posture. The LAFAN candidate batch is stopped; production training
has not started. See [the reference style review](REFERENCE_STYLE_REVIEW_20261112.md)
for the measured postprocessing artifact, pending PM01 numerical qualification,
and the distinction between optional dance references and world-model data.

The prior five-clip bank contains 40 seconds with overlap (approximately
31 seconds of unique source). It is a software-test baseline, not the final
v2 dataset. Long v2 training requires a new numerical QA report and rendered
review tied to the exact motion-bank hash.

Prefer natural locomotion from BONES-SEED and carefully reviewed LAFAN1.
Do not select solely because a filename contains `walk`. Inspect neutral
stance, clean starts/stops, steady walking, both turn directions, speed
coverage, foot heading, per-step clearance, stance slip, contact transitions,
joint limits and dynamic feasibility after retargeting to Sprite.

Maintain source/performer identity, crop intervals, mirror relationships,
file hashes and rejection reasons. Split source sequences/performers before
cropping; overlapping or mirrored excerpts must not leak into evaluation.
Do not manufacture straight walking by deleting real turn trajectories.
Human SOMA/Geno data require their own correspondence validation; G1 joint
trajectories are not Sprite references.

BONES-SEED access is gated by the provider. The account holder must accept
the terms and configure authenticated access before restricted files are
downloaded. Raw data, retargeted motion and credentials are not published in
this repository. Use metadata-first, selective retrieval rather than a full
dataset clone on the rented machine.

## Observation and release gates

Physical IMU mounting must be verified: pelvis/root or a torso-mounted site
moving with the waist. V1 used root-local gyro/gravity; changing only the
runtime frame or only the training frame is not acceptable. V2 supports an
explicit `root` or `imu_site` observation and refuses long training until the
configuration records verification. Software smoke tests may exercise an
explicit unverified assumption, but are not production training.

On November 12 the operator confirmed that the physical ICM-45686 is mounted
on the pelvis. The production configuration now records verified `root`
observations: body angular velocity in rad/s and dimensionless unit projected
gravity. Preserve historical smoke/benchmark metadata as originally recorded.
This resolves the mounting question; it does not approve the reference bank or
qualify a policy for hardware. See `IMU_OBSERVATION_AUDIT_20261112.md`.

Joint position and velocity are measured simulator/robot feedback. Previous
action is the previous clamped policy output before PD target saturation,
not a measured position or reconstructed saturated target. Export records
these semantics and keeps hardware qualification false.

Required sequence: targeted unit tests, short physical simulation training,
checkpoint evaluation/export parity, source and retarget review, then bounded
long training with retained checkpoints, command-sequence evaluations and
videos. Important artifacts are copied locally before rental expiry; generic
download caches and reproducible raw datasets are excluded.

## Initial implementation evidence, November 12

- 27 targeted integration, gradient, critic and replay tests passed on the
  preserved L40S environment (PyTorch 2.8.0+cu128, MuJoCo 3.11.0).
- Both eager and GPU-compiled three-update software smoke tests completed.
  They used the old reference bank and an explicitly unverified root-IMU
  assumption solely to exercise software paths. Their first update fitted
  dynamics only; the other two also updated both policy interfaces/critics.
  The original smoke config recorded the production 500-iteration warmup
  setting, while the CLI reduced it to one; subsequent configs now record
  `effective_wm_warmup_iterations` and `software_smoke_test` explicitly.
- Eager smoke ONNX parity maximum absolute error: `1.862645149230957e-08`.
  The resulting checkpoint also completed a short physical evaluation. Three
  updates are not a locomotion-quality result or a throughput estimate.
- No long training or physical motor operation has started in this experiment.

### Recovered Xbox reference

The user identified `BeijingDynamics/open_sprite_rl`; inspection pinned commit
`63a82f744c78d48ee42f557fb472d7499702ee94`. Its G74 model3000 uses the
G71 V20 contact-cleaned PM01-derived reference, not LAFAN1. The full-body input
is `assets/references/g71/sprite0825_pm01_g55_ik_contactclean_wbt_v1.npz`,
SHA-256 `97ec26f45460316e55845137b2adeb83958bd2676967af166085df085b509256`.
Retain `ENGINEAI_BSD_3_CLAUSE.txt` and the source notices. The three AMP walk
files are byte-identical sampling weights, not independent demonstrations.

The initial current-v5 audit keeps poses/root trajectories unchanged and
selects every second original sample (100 Hz to 50 Hz). It recomputes velocities
and FK. Common-body position disagreement is below 0.8 micrometres, joint
hard-limit excess is zero, minimum active-collider height is 3.64 mm, and no
self penetration was measured. Mean local forward speed is 0.19646 m/s.
Mean foot headings relative to pelvis heading are +3.38 and -2.66 degrees.

The reference remains a candidate: each foot's maximum *lowest sole point*
height is only 14.99/13.91 mm, below the existing 20 mm lift-event criterion.
A height-only near-ground foot-centre mask produces 0.10594 m/s RMS planar
motion; bottom corners produce 0.09873 m/s. These masks are not measured
contact, and the historical V20 report used a different height-and-velocity
support detector. That detector cannot independently attest to low slip
because it conditions on speed. Preserve both definitions rather than
reporting the historical and new metrics as identical.

A current-model reference video and eight-frame gait sheet have been inspected.
They show a compact, largely forward-facing gait with small clearance. The
old AMP policy's success does not establish that direct SuperMimic tracking
of this reference will fix foot dragging. Use it as a valuable geometry/style
comparison while resolving contact interpretation and building wider coverage.
The mass/COM update still changes dynamics despite nearly identical FK.

## Data and L40S execution plan, November 12

The main experiment remains the paper's Appendix B/G joint motion and
reinforcement-learning velocity interfaces. BONES-SEED is currently unavailable;
this is an explicit dataset deviation, not a reason to substitute AMP or declare
an exact reproduction. G71 is a comparison and possible reviewed supplement,
not a replacement for a varied locomotion library.

C. Zhang's pinned `tools/download_data.sh` retrieves 40 G1-retargeted LAFAN1
CSVs and the matching G1 description from
`lvhaidong/LAFAN1_Retargeting_Dataset`. These are 29-DoF G1 trajectories, not
Sprite trajectories or BONES-SEED data. The README reports 2.45 hours of motion.
Use the acquisition/provenance pattern, but retarget the human source to Sprite
and verify named joints, FK, contacts and temporal alignment. Do not reinterpret
G1 joint columns as Sprite joints.

The public Geno archive contains 12 walking and 4 running source sequences.
All 16 have been queued for selective acquisition; six are already available.
The six-source screening proposed seven non-overlapping crops totaling 42
seconds: three straight, three left-turn and one stop segment. These are human
source candidates, not accepted robot references. Coverage remains insufficient
for the final bank. Continue reviewing the remaining sequences, including held-out
subject 5, rather than treating this first filter's output as a complete dataset.
Source-level filters are research heuristics and need visual validation; their
rejections are not proof that every rejected sequence is unusable.

The world model is fitted to actual simulator transitions `(s, a, s_next)`.
References select tracking tasks and influence the visited state distribution;
they are not the dynamics model's direct replacement for simulator transitions.
Preserve 500 dynamics-only warmup iterations followed by joint online updates.
Assess prediction on withheld simulator episodes, separately reporting 1-step,
8-step and policy-horizon 32-step open-loop errors, with actions and randomization
context held equal to the recorded physical rollout. Break down errors by root
velocity/rotation, joint velocity, standing, gait transitions and turning; do not
accept a falling average training loss as sufficient evidence. This held-out
assessment remains required work, not a completed validation claim.

### Hardware-aware comparison

| Item | Published C. Zhang configuration | This Sprite experiment |
| --- | --- | --- |
| GPU | RTX PRO 6000 Blackwell | L40S, 46,068 MiB reported by driver |
| Neural matrix precision | BF16 autocast; state/FK/reward FP32 | Current baseline FP32, TF32 disabled |
| Physics/control | 200 Hz / 50 Hz | Confirmed 500 Hz / 50 Hz |
| Task | G1 motion tracking, 40 clips | Sprite joint motion and RL velocity interfaces |
| Body/collisions | G1, self-collisions disabled | 31-DoF v5, full-body collision enabled |

NVIDIA specifies 864 GB/s for L40S versus 3.35 TB/s for H100 SXM. This is a
memory-bandwidth comparison, not a training-time multiplier. The paper's H100
results, C. Zhang's roughly one-hour tracker and the earlier Sprite tracker run
have different workloads and success criteria. None provides a direct ETA.

A bounded production-shape benchmark uses 4,096 environments, the configured
replay capacity, world-model window 8, imagined horizon 32 and both interfaces.
It measures collection, world-model optimization, joint policy/critic updates,
allocated memory and reserved memory separately. Exclude first compilation and
the first dynamics-only update from steady-state estimates. The benchmark uses
the legacy bank solely to exercise compute, records an unverified root-IMU
assumption, and is marked `software_smoke_test`; its checkpoints cannot resume
as production training. The run is limited to 30 updates and a 1,800-second
training-loop budget, checked at update boundaries.

Tune microbatch size before changing scientific batch sizes or rollout horizons.
Compare TF32 and BF16 only with numerical and short closed-loop checks; do not
silently enable them or reduce the confirmed PD rate. Retain FP32 integration,
kinematics and reward calculations if mixed precision is qualified. Use measured
post-compilation throughput and early learning curves to allocate the three-day
rental, reserving time for evaluation and local artifact transfer. A wall-clock
cap is not a convergence criterion.

Sources:
- https://github.com/zita-ch/supermimic_vibed/blob/2a26b2f8c4d616e16361a616fe659c7d4acdf317/tools/download_data.sh
- https://github.com/zita-ch/supermimic_vibed/blob/2a26b2f8c4d616e16361a616fe659c7d4acdf317/README.md
- https://arxiv.org/pdf/2610.28480
- https://www.nvidia.com/en-in/data-center/l40s/
- https://resources.nvidia.com/en-us-gpu/h100-datasheet-24306
