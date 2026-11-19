# Recorded action interval jitter and torso lean

## Result

The approximately 25.9 ms left hip pitch action interval is real: same-adapter
hardware TX timestamps independently confirm it. Both maximum events were
common to the entire action, not a left-hip-only transport delay. Most of the
extra interval comes from increased actor inference wall time; the remainder
mainly comes from waiting for the coherent 2 ms send boundary.

This localizes the delay, but does not establish the operating-system cause
of the inference tail or prove that timing caused the observed side lean.
No controller settings, gains, protections, or hardware state were changed.

## Evidence and reproducibility

Run `python scripts/diagnose_action_interval_jitter.py` from the repository root.
It reads the immutable two powered-run logs under
`outputs/deploy-20261116/live-evidence/supermimic-pm01-xbox30-1116/` and writes
derived per-transition CSVs, summary JSON, and a plot under
`outputs/deploy-20261116/action-interval-jitter/`.

Each run contains 2,700 formal actions and 2,699 adjacent transitions. The
analysis checks 31 unique motors for every joined action and reconstructs
every left hip driver interval from publication and stage duration differences
to within 1e-8 ms. Hardware intervals subtract timestamps only for the same
motor on the same adapter; adapter clocks are never subtracted from host clocks.

| Measurement | Run 152917, sequence 1783 | Run 153129, sequence 2504 |
| --- | ---: | ---: |
| Policy time (s) | 35.64 | 50.06 |
| State publication interval (ms) | 19.980214 | 20.021602 |
| Inference, previous -> current (ms) | 2.973249 -> 8.190964 | 3.130187 -> 7.626775 |
| Target -> left hip driver, previous -> current (ms) | 0.590510 -> 1.260123 | 0.289574 -> 1.792427 |
| Left hip driver interval (ms) | 25.873238 | 25.882320 |
| Left hip hardware TX interval (ms) | 25.896 | 25.897 |
| Right hip hardware TX interval (ms) | 25.845 | 25.866 |
| Min/max driver interval across all 31 motors (ms) | 25.782230 / 25.873238 | 25.821838 / 25.882736 |

The interval identity is:

`TX interval = publication interval + change in state IPC + change in preprocessing + change in inference + change in postprocessing + change in target-to-TX latency`.

For the first maximum, inference adds 5.218 ms and target-to-driver latency
adds 0.670 ms relative to the previous action. For the second, these changes
are 4.497 ms and 1.503 ms. The complete identity includes the small remaining
stage changes; none of these contributions is an independent extra full cycle.

These are intervals between first sends of consecutive new actions. They are
not the motor's internal PD period or measured encoder-sampling-to-torque
latency. Motor-internal target latching is not instrumented.

## Scheduling mechanism

`runtime/native/src/sprite_can_shadow.cpp` uses 0.5 ms transport slots.
`coherent_commit_due` in `runtime/native/src/action_tx_trace.hpp` permits a new
action commit at `slot % 4 == 0`, giving a 2 ms coherent boundary. A slower
inference changes when the action reaches this boundary. The CAN process
prepares the whole action before issuing its motor sends; the peak events
therefore affect both legs and the other joints together.

The saved `scheduler-actual.json` verifies CAN affinity CPU 2 and SCHED_FIFO 50
before actor connection. This is not evidence of a missing CAN realtime grant.
The actor runs on the CPU 4/5 mask with ORT intra-op threads=2, inter-op=1 and
sequential graph execution. The inference timestamps surround the actual
inference call, excluding the subsequent IPC and transport waits.

Existing logs contain wall time, not per-thread CPU time, scheduler switches,
worker wakeups, or CPU frequency residency. They cannot distinguish preemption,
frequency changes, worker synchronization, and compute/cache effects. These
remain hypotheses. A read-only SSH attempt on 2026-11-16 timed out, so no fresh
Jetson profiling was possible in this investigation.

## Relationship to side lean

Compare equally sized 500-action windows: early control at 22-32 s and late
control at 40-50 s. Torso roll is reconstructed from logged joint positions and
IMU gravity using the existing geometry audit; it is not external motion capture.

| Measurement | Run 1 early -> late | Run 2 early -> late |
| --- | ---: | ---: |
| Driver intervals above 22 ms, out of 500 | 12 -> 15 | 12 -> 4 |
| Inference P99 (ms) | 6.843 -> 6.814 | 6.750 -> 6.732 |
| Median torso roll in model coordinates (degrees) | -0.932 -> -4.172 | -1.419 -> -6.192 |

The second run's largest interval occurs at 50.06 s, just after the nominal
Xbox window, whereas its torso lean is already substantial in the preceding
40-50 s window. Long intervals also occur before walking. There is no observed
progressive worsening of inference timing to match the progressive lean, and
the maximum events do not show a selective left-leg delay. These facts weaken
a simple left-motor-lag explanation; they do not rule out accumulated or
gait-phase-dependent sensitivity to common latency and jitter.

## Next diagnostic steps

1. On an idle Jetson, replay recorded observations with no CAN or IMU access.
   Measure inference wall time and per-thread CPU time/context switches,
   together with CPU frequency. Compare one and two ORT threads with identical
   model, inputs, CPU placement, and 50 Hz pacing; verify output parity.
2. If tails occur only in the complete pipeline, profile the no-enable pipeline
   under its real concurrent IMU, IPC, and CAN workload. An unpaced file replay
   alone cannot establish live timing behavior.
3. In closed-loop simulation, separately model measured observation age and
   action delivery timing, then compare matched seeds with and without measured
   jitter. The earlier fixed-target-delay experiment does not cover aged
   observations or this event timing distribution.

Do not hide the jitter by simply holding every result for a longer fixed delay.
Do not remove coherent action delivery or lower CAN priority without a separate
validated change. No powered test is part of this diagnostic.
