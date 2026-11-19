# Actor architecture comparison: 5,000 iterations per arm

## Frozen hardware-test candidate

The existing candidate is **Sprite SM-Walk R1**, release ID
`sprite-sm-walk-r1-20261119`. It is an alias of `walk-stand80-6000`, not a new
model or a hardware qualification. Its checkpoint SHA256 is
`70cf3e70354c8b574777709f56f24f83864329d589c256cfa97d6a1236afdedd`;
ONNX SHA256 is `5695b6980746534a11d075befc29bf5f7fe0ef2a7d81e3dadf3147c50f4d6930`.
The original delivery files, contract, recorded evidence, and hashes remain
unchanged. Experiments cannot overwrite this release.

## Question and four arms

Can a smaller actor retain useful standing, forward locomotion, turning and
recovery, while reducing batch-one CPU latency? This is a training experiment,
not an inference-only weight truncation or a gain adjustment.

| Arm | Motion encoder / shared decoder hidden widths | Velocity path | Purpose |
| --- | --- | --- | --- |
| full | 2048, 1024, 512, 256 | 256, 256 encoder, 32 latent, shared decoder | Current architecture control |
| medium | 1024, 512, 256, 128 | 128, 128 encoder, 32 latent, shared decoder | Approximately half-width shared architecture |
| small | 512, 256, 128 | 128, 64 encoder, 32 latent, shared decoder | More aggressive shared architecture reduction |
| amp_mlp | 512, 256, 128 for the motion branch | Direct 993 -> 512 -> 256 -> 128 -> 31, ELU | AMP-shaped deployment actor ablation |

The first three preserve LayerNorm/GELU, both interfaces, the shared decoder,
and reference-only latent alignment. The fourth reproduces the locally
available AMP G60 actor's hidden widths and ELU activations, **not its weights,
795-dimensional observation contract, or AMP training algorithm**. All four
receive the current 993-dimensional observation, including ten history frames.
The exact October Xbox artifact has not been verified; G60 is the audited
local architecture reference. No old project chat context is imported.

An exact direct MLP cannot simultaneously be the original encoder/shared-decoder
graph. Therefore `amp_mlp` is explicitly a **model-based direct-actor ablation**:
it uses the same learned dynamics, physical collection, motion task, velocity
objective and two critics. A training-only 32-dimensional auxiliary head retains
reference latent supervision. The velocity output bypasses the shared motion
decoder. This removes the decoder bottleneck and its motion prior, so this arm
is not a pure capacity comparison or an unchanged SuperMimic reproduction.
No discriminator or PPO is introduced. The other three provide the controlled
SuperMimic capacity comparison. The user's requested AMP-shaped experiment is
the specific exception to the inherited shared-decoder project convention.

### Activation smoothness and gradient interpretation

The audited AMP-style implementation uses ELU with alpha=1. This function is
continuous and continuously differentiable at zero; its second derivative has
different one-sided limits (1 from below, 0 from above). It is not a ReLU-style
first-derivative discontinuity. GELU, x times the standard normal CDF, is smooth
to all orders. Definitions: [PyTorch ELU](https://docs.pytorch.org/docs/stable/generated/torch.nn.ELU.html)
and [PyTorch GELU](https://docs.pytorch.org/docs/stable/generated/torch.nn.GELU.html).

The implemented actor update backpropagates through an unrolled learned model
using first-order `loss.backward()`; it does not differentiate an optimizer or
request Hessians / `create_graph=True`. ELU's lack of a continuous second
derivative therefore does not make this actor update undefined. Neither does
GELU alone guarantee a well-conditioned rollout gradient: network weights,
normalization, learned-model error and horizon matter. GELU can have negative
local slope and slope above one, whereas alpha=1 ELU has positive slope at most
one; this is a scalar activation property, not a network stability bound.

Record a same-weight, same-input ELU/GELU probe of a direct MLP, including
directional action sensitivity, directional gradient changes and finite values.
This is a numerical diagnostic, not a trained-policy substitution or an isolated
causal result about gait. Do not swap activation functions in a trained actor
and present that as the corresponding trained model. The four 5,000-iteration
arms proceed as defined; a trained activation-only pair can be a later follow-up
if evidence makes it useful. Alpha, GELU approximation mode, LayerNorm and
precision belong in the saved experiment metadata.

## Controlled initialization and budget

All four start with freshly initialized actors and both critics, using seed 61.
All inherit only the same R1 world-model weights and observation/goal/command
normalizers. Actor weights, optimizer state, replay and return samplers are not
inherited. Thus the full architecture does not receive an unfair trained-actor
head start. This tests learning under a fixed budget with a common pretrained
dynamics prior, not training entirely from scratch or distillation of R1.
Different shapes consume random numbers differently: the seed is shared, but
identical trajectories/minibatches are not claimed. One seed is a screening
experiment, not a statistical ranking of architectures.

Each arm runs **5,000 trainer iterations**, including 500 world-model-only
warmup iterations and 4,500 joint actor/critic iterations. Each actor iteration
updates both interfaces; each trainer iteration includes ten world-model
minibatches and physical collection at the existing update/data ratio.
This is not 5,000 simulator steps. Keep 4096 environments, 50/50 interfaces,
10-frame history, 10 future reference frames, horizon 32, MLP BF16 and FP32
physics. Keep world-model/critic sizes, optimizer settings, reward terms,
domain randomization, perturbations and command ranges fixed across arms.
Use the same reviewed 80% PM01 / 20% standing clip sampling and bank hash
`edd4ddd6f54e8a8a710481d11340ac6e38d6b944e75613c48e318fc3b32d296f`.
PD remains 500 Hz, policy 50 Hz, with unchanged gains, action scales, assets
and protections. Final iteration count is equal across arms; do not silently
stop a poor performer early and call it a completed comparison.

## Execution checklist

1. Freeze/name R1; create an isolated source snapshot and immutable configs.
2. Test old-graph parity, architecture serialization, gradient ownership,
   shared-decoder freezing, direct-actor export pruning and ONNX parity.
3. Run short physical training smoke tests of all four arms before long jobs.
4. Run the four arms serially on L40S/48 GB to avoid memory contention; retain
   checkpoints at 1,000, 3,000 and 5,000. Evaluate and render each milestone.
5. Export every final actor, including poor performers, with parity evidence.
   Preserve failures in reports rather than selecting only attractive videos.
6. Evaluate common nominal commands (zero, 0.15, 0.25, 0.5 m/s, left/right
   curves, start/stop), initial tilt probes, impulses/pulls, and 10/20 ms delay.
   A 1.0 m/s probe is out of the existing training range and separately labeled.
7. Compare torso pitch, standing knee flexion, foot spacing/crossing, bilateral
   clearance/contact duty, forward/yaw tracking, survival and recovery. Do not
   require recovery to the original heading. Retain the existing acceptance
   thresholds; low-speed stepping is diagnostic rather than a selection veto.
8. Save checkpoints, ONNX, configs, logs, videos, source provenance and analysis
   to the local workstation with verified checksums during the campaign.
9. Benchmark each final actor on identical saved observations. Desktop CPU
   measurements are preliminary. A valid Jetson comparison requires the same
   ORT build, 1.344 GHz CPU 4/5, thread configuration, warmup and 50 Hz pacing;
   report p50/p95/p99/max and deadline misses. No motor access is required.
10. Deliver an evidence table and videos for all arms; distinguish architecture
    cost, learning quality, and measured latency. Hardware admission is separate.

## Timing and failure handling

Initial estimate: 2-4 hours to the first arm's final checkpoint and roughly
12-18 hours for all training, milestone videos and validation. Replace this
estimate with measured throughput after compilation/warmup. Reserve time before
the renewed rental ends for exports and local backups. The coordinator must
record errors and continue independent arms; numerical/contract failures are
not repaired by changing gains, physics or protection limits. Any extension
beyond 5,000 must be a separately labeled follow-up, not overwrite this comparison.

## Implementation status and network-change pitfalls

On November 19, all four file-only architecture checks and all four physical
three-iteration training smoke tests passed. The full-size formal run started
after the common R1 baseline evaluation. Checkpoints are evaluated in rounds:
all four at 1,000, then all four at 3,000, then all four at 5,000. This gives
early evidence for every arm rather than finishing one entire arm before
starting the next. Consequently the first final 5,000-iteration model arrives
later than the initial 2-4 hour serial-arm estimate; plan to inspect the first
four-arm milestone in approximately 2-3 hours and final results in 12-18 hours,
subject to measured post-compilation throughput.

The isolation directory is `outputs/actor-architecture-20261119/source` locally
and `/root/gpufree-data/supermimic_architecture_20261119` on the rental machine.
The local backup worker retains milestone checkpoints with SHA256 verification,
including checkpoints whose subsequent evaluation fails. It also copies videos,
configs, metrics, world-model validation and final analysis. It is independent
of the SSH session that started training.

Specific pitfalls checked or explicitly retained as limitations:

- State-dict shape coincidence is not semantic compatibility. Architecture IDs,
  widths and initialization mode are explicit and checked on resume.
- No partial actor-weight loading, optimizer-state reuse or silent truncation.
  Output layers retain the existing small-output initialization.
- Motion and velocity optimizer parameter sets are disjoint and their union
  covers all trainable parameters. Velocity backward cannot accumulate gradients
  into motion-encoder or shared-decoder weights. It still differentiates through
  the frozen decoder inputs for the three shared-interface arms.
- The direct actor's auxiliary alignment head must be absent from deployed ONNX;
  its deployed graph has exactly four dense layers and three ELU operators.
- Runtime term-major history and training frame-major history are explicitly
  cross-checked. Previous action remains the raw clamped policy output, never
  the saturated target or measured joint position.
- Training, physical evaluation and export use the same controller factory.
  Raw checkpoint loading and graph parity are checked before claiming a model
  can be deployed. Smoke artifacts are not deployment candidates.
- The reported inference parameter/MAC count excludes the motion encoder,
  world model, critics and the direct actor's training-only auxiliary head.
- Same seed does not mean identical stochastic training trajectories. Learning
  rate, global gradient clipping and fixed horizon are held constant; smaller
  models may have different optimization dynamics and convergence rates.
- No activation-function replacement inside a trained candidate. Record ELU
  alpha=1, exact GELU, LayerNorm, precision and observation normalization when
  interpreting accuracy and latency.
- Width experiments cannot resolve every sim-to-real failure. Simulation
  comparison, Jetson inference replay and powered hardware tests stay separate.
