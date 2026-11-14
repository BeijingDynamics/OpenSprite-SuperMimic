# Research decisions and real-data alignment roadmap

## Task definition

The primary task is reliable teleoperated forward locomotion, usable turns,
quiet stops and balance recovery. Disturbances may change world heading;
returning to the original heading or path is not required. Track commands in
the current pelvis heading frame and track commanded yaw rate. Continue to
inspect foot and torso alignment relative to the pelvis.

The retained research baseline is PM01 checkpoint 7629. Preserve complete
SuperMimic, the published robot/control profile and action-history semantics.
Current actor-learning-rate comparisons and independent-seed replications
continue. A newer checkpoint is not automatically a better checkpoint.

## How to resolve a failure

Each proposed experiment should identify the symptom, a falsifiable cause,
one controlled change, and a physical evaluation that could reject it.

| Evidence | Diagnostic question | Next experiment |
| --- | --- | --- |
| Reference itself has poor feet or asymmetry | Is retargeted contact geometry executable? | Inspect source/retargeted motion, clearance, joint velocities and physical tracking before admitting data. |
| Imagined motion is good but physical simulation is bad | Does the world model predict the relevant transition incorrectly? | Replay the same actions from the same states; inspect 1/8/32-step state and sole errors, with held-out failures and recovery states. |
| Physical simulation satisfies reward while gait is poor | Is the objective rewarding an unwanted solution? | Inspect reward components alongside swing alternation, slip and alignment; test one explicitly documented objective/reference change. |
| Simulation is good but hardware is poor | Is the observation/action contract wrong, or are dynamics different? | Audit ordering, units, histories, timestamps and command transformations first; then identify mismatch from real trajectories. |
| Longer training regresses | Is this unstable optimization, data-distribution drift, or model exploitation? | Keep the good checkpoint; compare actor step size, world-model validation and physical behavior at saved milestones. |

Training return, world-model loss and simulation survival measure different
things. No one number substitutes for the other two. Current world-model
validation samples also change with the collecting policy; they cannot alone
prove improvement on an identical test set. If a validation batch contains no
physical failures, it cannot validate failure detection even when its error is
small. A frozen transition benchmark is the next diagnostic extension if the
current comparison does not explain regression.

Use several initial conditions and evaluation seeds. Include command changes,
stop/restart, alternating support, pushes at different gait phases and sustained
pulls. Increase difficulty in simulation after nominal walking is reliable.
Keep absolute world-heading offsets descriptive. Hold out some perturbations
from experiment selection for a later generalization check.

Record successful and failed experiments with hashes, exact configurations,
raw trajectories, physical metrics, videos and a decision. Preserve the known
good baseline and back up important evidence locally before rental expiry.

## Verified ASAP mechanism

ASAP learns a state/action-dependent residual from recorded real trajectories,
uses it to correct simulated actions, freezes that residual during policy
fine-tuning, and deploys the resulting policy without the residual model.
Its paper uses onboard measurements plus motion capture for full-state data.
This differs from fitting a few actuator parameters. See
[ASAP, sections III-A through III-D](https://arxiv.org/html/2502.01143v2#S3)
and the [official project method](https://agile.human2humanoid.com/).

The official repository provides separate delta-model training and policy
fine-tuning configurations, and a real-data logger that subscribes to joint,
command, IMU and odometry streams:

- [Delta training configuration](https://github.com/LeCAR-Lab/ASAP/blob/main/humanoidverse/config/exp/train_delta_a_open_loop.yaml)
- [Policy fine-tuning configuration](https://github.com/LeCAR-Lab/ASAP/blob/main/humanoidverse/config/exp/train_delta_a_closed_loop.yaml)
- [Collection example](https://github.com/LeCAR-Lab/ASAP/blob/main/sim2real/rl_policy/listener_deltaa.py)

This roadmap proposes an adaptation to Open Sprite and SuperMimic; that combined
method has not been implemented or validated here. Do not replace the current
joint SuperMimic controller with the upstream phase-conditioned PPO controller.

## Proposed sequence after the new policy walks on hardware

1. **Freeze and record the actual execution contract.** Save model and runtime
   hashes, joint/motor mapping, defaults, action scales, Kp/Kd, saturation logic,
   parallel-joint transformations and observation/history construction. Log
   policy action and nominal position target separately from protected targets
   and encoded motor commands. Save action IDs, state sample times, inference
   start/end, IPC arrival, per-motor TX and feedback timestamps. A bus TX timestamp
   is not proof of the actuator's internal application time.
2. **Collect informative, labeled sessions.** Include starts, steady forward
   motion, turns, stops and later disturbances over the intended operating range.
   Mark support by the sling, external contact, saturation, dropouts, battery
   state and temperature when observable. Supported and freely walking data must
   not be treated as the same unforced dynamics. Motor-reported torque is an
   estimate unless independently calibrated.
3. **Fit an interpretable baseline first.** Estimate identifiable delay,
   response bandwidth, friction, effective effort/speed behavior and mechanical
   effects from the appropriate data. Keep uncertainty ranges and evaluate on
   entirely held-out sessions. A better fit on one trajectory is insufficient;
   correlated parameters can hide timing or state-estimation errors. This step
   is our system-identification baseline, not a claim to reproduce ASAP itself.
4. **Learn the residual mismatch in simulation.** Compare nominal simulation,
   identified physics and identified physics plus an ASAP-style residual on the
   same held-out real recordings. Regularize residual magnitude and reject
   models that only memorize a particular policy, command or support condition.
   Split by whole recording session, not neighboring samples in one rollout.
5. **Re-align SuperMimic before policy refinement.** Freeze the residual,
   regenerate transitions under the corrected simulator, and revalidate the
   shared world model before substantial actor updates. Version or rebuild
   replay so unlabelled transitions from incompatible dynamics do not mix.
   Then refine both interfaces with the shared decoder, latent alignment and
   separate critics intact. Judge the result with physical simulation and
   subsequent hardware trials, not only imagined reward.

Conceptually, choose and document one action boundary:

`a = policy(history, command)`

`a_sim = a + delta(simulated_state, a)`

`s_next = simulator(s, execution_pipeline(a_sim))`

The policy history still stores **a**, while joint/IMU observations come from
the resulting state. The world model learns transitions conditioned on **a**
through the corrected environment. It must not learn one action convention
and deploy with another. Scale, defaults, saturation and parallel-joint mapping
must be represented at the same documented boundaries during replay and
fine-tuning. Avoid double compensation if identified physics already explains
part of the mismatch.

The final hardware actor runs through the established PD/protection pipeline.
This plan does not add a manually chosen feedforward torque or an ASAP residual
to the deployed runtime. Policy refinements are learned in the corrected
simulation and then tested as complete, versioned controllers.

## Measurement boundary and current status

Joint feedback and the pelvis IMU can support useful actuator and local-response
analysis. They do not by themselves provide independent, drift-free base world
position and linear velocity. Full-state ASAP alignment requires a validated
external pose/velocity measurement or a separately validated estimator, with
time synchronization and uncertainty recorded. Do not invent those labels by
assigning zero velocity, integrating IMU indefinitely, or treating a policy
prediction as ground truth. Partial-observation fitting is a separate experiment
with explicitly narrower claims.

At this stage this is a documented future work package. Current GPU training is
unchanged. No new robot data collection, motor enable, runtime residual,
actuator parameter change or hardware protection change has been performed.
