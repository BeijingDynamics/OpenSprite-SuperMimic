# Inference cost and tail diagnosis

Follow-up: the [completed Jetson CPU replay diagnosis](JETSON_CPU_REPLAY_DIAGNOSIS_20261117.md)
reproduces the slow mode and validates a temporary CPU-floor intervention,
including CPU/EMC telemetry and restoration. The plan below records the
initial architecture audit and hypotheses, before that target experiment.

## Verified deployment graph

The November 16 live bundle and selected update-10000 model have identical
SHA256 `ec638542ea681da6fcd56a9888f3f09ef13eb92a60ab1d75b9aae9131600abf6`.
`scripts/audit_deployed_policy_cost.py` validates the ONNX graphs and records
layer counts, initializer counts and dense MAC estimates in
`outputs/inference-audit-20261117/model-cost.json`.

The deployed forward path is:

1. Reorder 993 runtime inputs into ten 99-element history frames and a
   three-element velocity command; normalize them.
2. Velocity encoder: `993 -> 256 -> 256 -> 32`, followed by tanh.
3. Concatenate the 32-element latent and 990-element normalized history.
4. Shared decoder: `1022 -> 2048 -> 1024 -> 512 -> 256 -> 31`.
5. Clip the policy output using the existing export contract.

There are six LayerNormalization operations and six GELU activations exported
as Erf-based expressions. There is no deployed world-model rollout, motion
encoder, critic, online optimizer, Transformer or recurrent neural network in
this graph. Ten-frame history is an explicit input stack, not ten independent
forward passes. The C++ actor invokes ONNX Runtime CPU inference.

The encoder/decoder widths match Table A.4 of the
[SuperMimic paper](https://arxiv.org/pdf/2610.28480), with input and output
dimensions adapted to Open Sprite's 31 joints.

| Quantity | Current SuperMimic | Locally available AMP G60 baseline |
| --- | ---: | ---: |
| Input / output | 993 / 31 | 795 / 31 |
| Dense layers | 8 | 4 |
| Dense MACs per batch-one inference | 5,181,440 | 574,848 |
| Initializer elements, including stored statistics | 5,194,763 | 577,365 |
| ONNX file bytes | 20,797,822 | 2,310,883 |

The dense MAC ratio is 9.0136. The decoder accounts for 4,853,504 MACs
(93.67%); its first two matrices account for 80.87% of the full actor.
At 50 Hz the dense workload is about 259 million MAC/s, or 518 million
floating-point operations/s when multiply and add are counted separately.
This excludes activations, normalization, memory movement, and scheduling.
It is not a latency prediction: batch-one dense matrix-vector inference has
different utilization and memory behavior from a large GPU training batch.

The G60 graph SHA256 is
`43f213e4c5b9079e13b7f6f3635f224766227757417a0940ca49d585231016e3`.
It is **not** the exact October 19 Xbox model, whose saved manifest identifies
SHA256 `05f7ea02b6020ec44f4d5fa71e579294a73dc8c87d4b54f11198a7e83cb96d5d`.
The approximately ninefold cost comparison is therefore explicitly a local
AMP baseline comparison, not a verified architecture comparison to that
particular historical Xbox artifact.

## What the existing timing can establish

The current two-thread live actor has approximately 3 ms median inference and
7-8 ms tails. A 3 ms inference consumes 15% of a 20 ms policy period; an 8 ms
inference consumes 40%. This is compatible with the nominal rate in these
recordings but is neither a measured lower bound nor a stability guarantee.
Complete sensor-to-TX timing and its variability still matter.

Earlier one-thread, 50 Hz, CPU-only replay of the same update-10000 model
reported a 10.029743 ms median in `env-00-handoff-off`, ORT 1.23.2, 50 warmup
calls and 1,000 recorded frames. This differs from the live two-thread setup
and lacks a matched CPU operating-state trace. It does not prove a threefold
benefit from adding one thread or establish that single threading is better.

The observed 25.9 ms action intervals are already decomposed in
[the action interval audit](ACTION_INTERVAL_JITTER_20261116.md). That audit
identifies increased inference wall time, not its underlying OS/hardware
cause. A November 17 read-only SSH connection attempt to 253 timed out.
No new Jetson benchmark or powered run was completed for this report.

## Target profiling experiment

Use isolated file-only CPU replay with no controller, CAN, IMU or gamepad
device access. Confirm no live control process before applying CPU load.
Preserve the existing deployed binary and settings. Reuse recorded 993-element
observations so every configuration sees identical inputs; verify the 31
outputs against the current actor, separately from performance measurement.

Measure baseline runs without instrumentation first, then diagnostic runs with:

- Monotonic inference start/end, main-thread CPU time, process CPU time,
  and voluntary/involuntary context switches.
- Main and ORT worker thread IDs, effective CPU affinities, migrations,
  and per-thread scheduler run/wait times where available.
- CPU frequency/governor, temperature, power mode and background workloads.
  Sampling current frequency alone cannot establish per-call causation.
- Both 50 Hz paced and unpaced replay. A hot unpaced loop is not a valid
  replacement for measuring the actual 20 ms duty cycle.

Compare repeated interleaved trials of the current two-thread CPU 4/5 setup,
one thread, explicit distinct per-thread placement, and supported spinning
settings. Keep the model, ORT version and power state fixed within each
comparison. Do not assign inference a priority above CAN or change system
power/CPU policies as an unrecorded side effect.

Interpretation requires multiple counters: wall time rising with run-queue
wait indicates preemption/competition; increased executing time with frequency
changes suggests compute-rate effects; worker wake/wait changes implicate
thread-pool behavior. Process CPU time sums parallel execution and may exceed
wall time, while spinning consumes CPU without useful computation. Neither
counter alone establishes the cause.

[ONNX Runtime's threading documentation](https://onnxruntime.ai/docs/performance/tune-performance/threading.html)
documents the thread-count, affinity and spinning controls. In particular,
explicit intra-op thread counts do not automatically give each worker a
distinct core affinity. The actor's CPU 4/5 process mask restricts eligible
cores but is not the same as pinning one worker to each core. Available keys
must be checked against the deployed ORT 1.23.2, not assumed from newer docs.

## Optimization order

1. Diagnose and reduce tails with unchanged weights and numerical precision.
2. Benchmark batch-one static shapes, existing graph optimizations/fusion,
   allocation reuse and thread configuration; establish numerical parity.
3. Consider a GPU/TensorRT backend only with measured end-to-end transfer,
   launch, synchronization and tail latency. More GPU FLOPS do not by
   themselves establish lower latency for this small batch.
4. Treat FP16/INT8 or a distilled smaller decoder as a separate model change
   requiring output, closed-loop and robustness validation. A smaller model
   is not yet justified merely by a 3 ms median, and its achievable speed or
   behavior cannot be promised in advance.

Do not shrink the architecture or change gains to hide the observed torso
lean. That mechanical/closed-loop discrepancy remains a separate unresolved
diagnostic alongside inference timing.
