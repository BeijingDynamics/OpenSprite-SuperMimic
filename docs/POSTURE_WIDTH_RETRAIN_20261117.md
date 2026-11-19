# Posture and foot-spacing transfer experiment

## Machine recovery and isolation

The new rented endpoint is `120.209.70.195:30179`, with an NVIDIA L40S
(46,068 MiB reported by the driver). Its cloned system disk retained package
caches, but its 98 GB data volume was empty. The operator reactivated the
original machine on port 31067 without a GPU. Validated assets, reference
motion and the parent checkpoint were recovered from that data disk.

Remote experiment root:
`/root/gpufree-data/supermimic_posture_width_20261117`.
Local source and evidence:
`outputs/retrain-posture-width-20261117/`.
The old experiment, deployed robot policy and runtime are unchanged.

Parent checkpoint SHA256:
`aab860df068d5ccc67aa94ea315f5517dd333373c0e7be52fe6344bb5a98ee8a`.
PM01 reference SHA256:
`7c6dccdc87756956f58ed9c5c6c7432882172abc62f116b803f719ecc16e6870`.
Both were verified after recovery. PyTorch 2.8.0+cu128, MuJoCo/MuJoCo-Warp
3.11.0 and Warp 1.17.0 are retained. Transitive dependencies were reconciled
against the original `pip freeze`; the only intentional extra tool is `uv`.

## Changes under test

This remains joint SuperMimic: shared world model and decoder, separate
motion/velocity encoders and critics, 50/50 real environments, 10-frame
history, 50 Hz policy and 500 Hz simulation PD. Full published gains,
action scales, actuator/physical contract, reference data, motion tracking,
sole-height tracking extension and recovery curriculum are unchanged.

The opt-in `sprite_velocity_posture_width_v1` velocity reward adds:

- A soft sagittal torso-angle cost with a 2-degree deadband and weight 8 on
  squared excess sine. It does not command a fixed torso angle or constrain
  world heading. Dynamic recovery remains free to use body lean.
- A soft shortfall cost below 120 mm signed foot-center spacing or 40 mm
  lateral sole-edge gap, weight 0.4 with a 40 mm normalization scale.
  Sole orientation and dimensions are included, so turning toes outward
  cannot hide a smaller edge gap. Signed separation penalizes crossed feet.
- A lighter excessive-width cost above 180 mm, weight 0.1, to discourage
  solving the narrow-foot issue by keeping a very wide stance.

These are experimental coefficients, not proven hardware targets. Lateral
projection is conservative when feet are staggered fore/aft; evaluation must
also retain physical contact and foot-clearance metrics.

The transfer inherits controller/normalizers, world model and motion critic.
The velocity critic, all optimizers, replay, samplers, live simulator, RNG and
wall/iteration budgets restart. It is explicitly not an ordinary resume across
changed rewards. The base algorithm retains its 500-iteration fresh-training
world-model warmup contract. A separately recorded pretrained-transfer
override uses 20 recollection/world-model updates; actor optimizer steps stay
frozen through iteration 100 while critics adapt. Actor learning-rate scale
remains 0.25. Later unchanged-reward stages use ordinary checkpoint resume.

## Completed checks and baseline

23 reward, geometry, legacy parity and interface tests passed. A three-update
GPU smoke run exercised both imagined interfaces and critic/world-model
updates with finite losses. The new full-size pilot uses 4,096 environments.

Re-evaluation of the unchanged parent on the recovered machine reproduced the
known posture and gait measurements, with all five cases surviving 20 seconds:

| Case | Median torso pitch | Median foot-center distance | P05 center distance | Mean forward speed |
| --- | ---: | ---: | ---: | ---: |
| Zero | -5.03 deg | 141.4 mm | 141.2 mm | approximately zero |
| Forward 0.25 | +1.65 deg | 89.3 mm | 84.6 mm | 0.204 m/s |
| Forward 0.15 | -1.61 deg | 100.1 mm | 92.9 mm | 0.153 m/s |
| Left curve | -0.27 deg | 91.6 mm | 83.5 mm | 0.188 m/s |
| Right curve | +0.48 deg | 91.4 mm | 88.4 mm | 0.182 m/s |

Negative pitch means backward. This distinguishes zero-speed backward posture
from narrow moving steps: the parent is not narrow in the zero-speed case.

## Execution and acceptance

`scripts/run_posture_width_experiment.py` serializes tests, parent evaluation,
a 200-update transfer pilot, physical evaluation and recorded-qpos video.
It continues to updates 500, 1000, 2000 and 3000 only after regression checks.
Each stage produces a checkpoint, physical rollout, posture/spacing metrics,
clearance metrics and a video. Training has a cumulative six-hour budget.
GPU evaluations do not compete with training on the same card.

The local `scripts/backup_posture_width_experiment.py` process copies completed
evaluation directories, videos and their checkpoints back to this workspace.
Checkpoint SHA256 must match its physical-evaluation report before the local
copy is accepted. The copier stops at a terminal job state or after seven
hours; it is tied to this experiment, not a recurring automation.

The automatic gate only authorizes continued research: all nominal cases
survive, zero-speed pitch does not regress by more than 1.5 degrees, forward
0.25 m/s produces at least 0.12 m/s, foot crossing does not increase materially,
and P05 forward foot spacing does not shrink by more than 5 mm. Passing this
gate does not mean the user's gait-quality goals have been reached.

Before selecting a new hardware candidate, require materially improved
zero/low-speed upright posture and bilateral spacing without trading away
forward motion, foot clearance, step alternation or turning. Follow with
multiple seeds, longer starts/stops, and separate undisturbed and finite-force
recovery evaluations. Do not demand recovery of pre-push world heading.
Rendered review and subsequent no-enable runtime parity remain separate gates.

No robot communication, motor enabling, IMU offset, gain increase or protection
relaxation is part of this experiment. Keep source manifests, selected metrics,
checkpoints and videos locally; do not publish externally licensed motion data.

## First pilot result

The 200-update pilot passed the five nominal physical continuation checks.
Its checkpoint SHA256 is
`55752528a949f07b42cacf475b3979e8ee0dd176570f1b6eaf0ce54cbff5cd94`.

| Metric | Parent | Pilot |
| --- | ---: | ---: |
| Zero-command median pitch | -5.03 deg | -3.95 deg |
| Forward 0.25 median pitch | +1.65 deg | -0.40 deg |
| Forward 0.25 median center spacing | 89.3 mm | 131.2 mm |
| Forward 0.25 P05 center spacing | 84.6 mm | 120.4 mm |
| Forward 0.25 mean speed | 0.204 m/s | 0.187 m/s |
| Forward 0.25 median swing peaks, left/right | 16.1 / 18.4 mm | 15.5 / 15.9 mm |

All five 20-second cases survived, and there were no crossed-foot samples in
the scored pilot cases. The low-speed case and both turning cases also became
wider. The right-turn yaw response weakened to about -0.089 rad/s for a
-0.20 rad/s command. Static backward lean remains and swing clearance has not
improved; the early result is not a solved or hardware-qualified controller.

A video-only argument error interrupted the coordinator after successful
pilot evaluation: the archived renderer requires exactly four panel indices.
The launcher was corrected to use cases 0, 1, 3 and 4 and gained an explicit
`--continue-job` path. Completed stages were reused; the pilot was not repeated.
The failure status was preserved. Video generation then passed and training
resumed from update 200 toward update 500, with subsequent stages gated as
described above. The local evidence copier was restarted after that repair.

## Continued training and gait-quality selection

The user requested further upright improvement while retaining foot clearance
and turning. Update 500 restored right-turn response (-0.205 rad/s for a
-0.20 rad/s command), but did not meet the combined objective. At 0.25 m/s,
median center spacing was 124.1 mm and forward speed was 0.236 m/s. At the
0.15 m/s command, median pitch deteriorated to -5.05 degrees and bilateral
swing peaks diverged to 22.84 / 7.01 mm. The 500-update checkpoint is retained
as research evidence, not selected as an improved deployment controller.

The isolated experiment now runs `scripts/assess_gait_quality.py` after every
nominal geometry evaluation. It requires all five cases to survive, median
absolute pitch within 3 degrees, and bounded P95 pitch. Moving cases must have
P05 center spacing of at least 100 mm, P05 lateral sole-edge gap of at least
20 mm, no crossed samples, each foot retaining 95% of its baseline median
swing peak and 85% of its swing count, and bounded left/right peak asymmetry.
Forward speed errors and signed responses in both turns must also retain
baseline performance within explicit tolerances. Five synthetic regression
tests cover pass, low clearance, reversed turning, speed overshoot and lean.

These are provisional nominal screening criteria, not modifications to reward,
actuator limits, gains or robot protections. They do not interrupt the research
continuation gate. Passing nominates a checkpoint for multi-seed and recovery
validation; it is not permission to deploy. The existing pitch/spacing reward
continues to train, with unchanged motion clearance tracking. Future reward
changes must use a separately identified transfer experiment, not silently
change a resumed checkpoint's objective.

## One-day rental continuation

The 1000-update result improved zero-command median pitch to -3.34 degrees.
Forward 0.25 m/s P05 center spacing was 123.4 mm, measured speed 0.195 m/s,
and left/right swing peaks 13.56 / 21.38 mm. Left/right turn responses were
+0.200 / -0.170 rad/s. It passes the research continuation gate, but not the
combined quality screen: standing lean, asymmetric clearance and slow-command
tracking remain unresolved. Its checkpoint and video were copied locally.

The user explicitly authorized continued use of the one-day rental. A separate
bounded coordinator waits for the existing 3000-update job to finish. It only
extends a normally completed job, through updates 5000, 8000 and 10000, with
the same model, reference, objective, gains, action scale and physics contract.
Each stage retains the existing nominal survival/regression gate and creates
geometry, clearance, quality-selection and video evidence. Cumulative training
wall time is capped at 12 hours; the follow-on coordinator has an 18-hour
deadline including its wait. These are execution bounds, not a verified cloud
lease expiry. The separate evidence copier is bounded to 19 hours. No paid
renewal or cloud account change is performed.

Recovery impulse training starts at update 1000 and reaches full configured
strength at update 8000. The longer continuation therefore exercises the
existing curriculum instead of changing rewards prematurely. Failed research
gates stop further training; they are not overridden to keep GPU utilization
high. Completed stage configurations are preserved when the coordinator
reuses earlier results.

After training, up to two surviving research checkpoints are ranked by failed
quality checks, then aggregate absolute median pitch. Their screen-pass flag
is retained: ranking does not turn an unsuccessful screen into a pass. These
checkpoints and the original parent receive paired 30-second tests with seeds
202 and 303, using independently sampled local root roll/pitch within +/-2
degrees and restoring minimum collision clearance to 1 mm. The exact sampled
angles and initial root states are saved. This explicit perturbation matters:
the original nominal evaluator disables physical randomization, so changing
its seed alone would not establish starting-pose robustness. Default tilt is
zero and preserves existing nominal tests. No actuator or model randomization
is claimed by these probes.

Each shortlisted checkpoint and parent also receives the existing 48-case,
20-second finite-force push/pull suite. No return to the pre-disturbance world
heading is required. Full reports remain for review rather than an automatic
hardware-release decision. The follow-on backup saves completed evaluations,
verified checkpoints, stage logs/configs and videos locally as they become
available. Nine targeted selection/rotation tests passed locally and on the
training host; the new perturbation path still awaits its physical rollout.
