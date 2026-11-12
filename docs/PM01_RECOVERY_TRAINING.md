# PM01 walking and recovery experiment

This simulator experiment targets Xbox-style commands `(vx, vy, yaw_rate)`
with natural walking and recovery from disturbances. It retains full joint
SuperMimic: motion and velocity interfaces, shared decoder and world model,
separate critics, equal environment allocation and the current v2 algorithm.
It does not introduce AMP or a discriminator.

## Launched run

The first phase started at **2026-11-12 14:53 Asia/Shanghai**, from source
commit `818792eefe3148141dee13546a9e1a23d352551a`. The rented-server run is
`outputs/pm01-baseline-20261112-064405/training`. The first checkpoint was
saved during dynamics warmup. Training startup is not a walking-quality result.
The bounded evaluator is running alongside training. A hidden local backup job
copies this run's essential artifacts hourly, with its first copy completed;
it requires this Windows host to remain running and connected.

Local destination: `outputs/retrain-20261112/pm01-recovery-training`.
It includes the prepared reference, preserved audits, exact source snapshot,
run configuration, checkpoint and subsequent evaluation evidence. Generic raw
source datasets and dependency caches are excluded.

## Reference and qualification

Use the operator-reviewed PM01/G71 whole-body walk adapted by name to the
current v5 FK at 50 Hz. The root and joint poses remain unchanged. This is one
11.98-second steady-walk demonstration, not the paper's larger locomotion
library and not an independent-source holdout. No dance is mixed into this
first baseline. Repeated copies would change sampling weight, not diversity.

`prepare_pm01_baseline.py` writes a separate simulator-research qualification
bound to the source, model and rendered-review hashes. It preserves the original
failed heuristic audit. Low sole clearance and ambiguous contact-phase labels
remain explicit evaluation risks. The 20 mm clearance heuristic is not a joint,
motor or robot-body safety limit; no runtime protection has been changed.

## Training

Configuration: `configs/velocity-v2-pm01-recovery.json`.

- 4,096 environments; 500 Hz PD and 50 Hz policy; full published 31-joint
  gains, effort caps, default pose and action scale 0.25.
- Pelvis gyro in rad/s and unit projected gravity. Previous raw policy action
  enters history; measured joint states remain measured states.
- First 500 iterations fit dynamics only. Normal joint training then proceeds.
- Recovery impulses start after iteration 1,000 and reach their bounded
  amplitude at iteration 8,000. They occur every 3-6 simulated seconds in
  eligible episodes. One quarter of environments receive no extra impulses.
- Maximum planar velocity increment is 0.35 m/s; roll/pitch angular increment
  norm is at most 0.5 rad/s. These are simulator disturbance settings, not
  qualified physical push limits.
- An impulse changes physical root velocity before observation and before
  collecting the next transition. It does not reset history, previous action,
  episode age, joint pose or actuator memory. Replay marks a world-model-only
  discontinuity: a prediction starting before the impulse cannot supervise
  across it; predictions starting after it use valid physical transitions.
- This curriculum trains recovery after impulsive disturbances. It is not an
  implemented sustained-force training curriculum. Actual finite-duration
  forces test that generalization separately; do not claim equivalence.

Existing initial-state, friction, armature, encoder-bias and COM randomization
remain. Gains and effort caps stay fixed to the confirmed profile. The actor
receives neither disturbance labels nor force sensors. The 993-dimensional
deployment interface is unchanged; new model deployment still needs replay and
hardware qualification.

MLPs use BF16, while integration, kinematics and rewards stay FP32. Global TF32
remains disabled. Measured production-shape throughput before this curriculum
was approximately 0.73 updates/s with a peak allocation near 15.1 GiB on L40S.
This is an estimate, not a convergence guarantee. The first phase has a 12-hour
wall limit and a 100,000-update ceiling; saving and evaluation occur throughout.

## Evaluation

`configs/velocity-recovery-evaluation.json` contains nominal standing/walking,
start/stop, turns, reverse and lateral commands, plus four-direction torso
forces during standing and forward walking. The simulator applies world-frame
forces at the torso body COM and records every applied force sample.

The force ladder includes 5/10/20 N for 0.2 seconds and 5/10 N for 1 second.
These values define tests, not promised capabilities. The policy gets no cue
about the force timing. There is no failure reset. Report failures before force
application separately from failure to recover.

Recovery means velocity error below 0.10 m/s, yaw-rate error below 0.25 rad/s,
and pelvis tilt below 12 degrees for one continuous second after force release.
Also report subsequent survival, actual foot clearance and heading, and
foot-centre velocity during measured loaded contact. Rolling motion means
foot-centre speed is not identical to material-point slip. Never classify a
low-speed gait solely by whether clearance exceeds 20 mm.

`watch_recovery_training.py` evaluates complete frozen checkpoints with three
seeds, periodically and at job exit. Seeds vary simulator stochastic timing;
they do not constitute independent training seeds. Videos show physical
rollouts, commands, measured velocity, failures and external forces.
World-model holdout reports separately cover 1/8/32-step prediction. Holdout
environments are excluded from WM fitting and its normalization; policy
collection still visits them, so this is not full policy generalization.

## Verified implementation before the long run

- 34 relevant replay, world-model, v2-interface and retarget contract tests passed.
- The real MuJoCo Warp disturbance check preserved policy history and episode
  state, matched injected velocities, kept nominal control environments intact,
  and confirmed force release. A 10 N lateral force for 0.2 s produced 11.6 mm
  differential lateral displacement in its pose-hold plumbing test. This is
  not trained-policy robustness evidence.
- The PM01 three-update software smoke completed both interfaces and WM holdout
  reporting. Its 32-case physical evaluation exercised the pipeline; all cases
  failed, as expected for this untrained checkpoint. Do not publish that as a
  trained walking result.
- ONNX export maximum absolute parity error was approximately `1.86e-8` on the
  smoke checkpoint. This verifies export plumbing, not hardware readiness.

Inspect early physical results before choosing a continuation or any dance
ablation. If sustained pulls expose a gap, add and validate a separate force
curriculum rather than weakening survival or tracking criteria.
