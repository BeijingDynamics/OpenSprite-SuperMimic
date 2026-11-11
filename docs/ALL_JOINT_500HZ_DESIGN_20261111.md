# All-joint 500 Hz transport proposal

Status: analysis only; no controller implementation or deployment in this branch.
Branch: `supermimic_500hz`, based on `32d6dfb`. This repository's runtime snapshot
is the source of the deployed coherent-send executable; the separate runtime
checkout is older. Existing working trees and Jetson deployment are unchanged.

## Contract

Keep the policy at 50 Hz, with the existing 993-input, ten-frame observation
history and raw previous-action history. Hold each accepted raw action between
policy updates. Every 2 ms, use fresh measured motor state to recompute the
existing protected motor targets and transmit all 31 MIT commands. Preserve
training Kp/Kd, action scale, mapping and limits. Do not introduce action
interpolation, feedforward torque, gain ramps or extra policy evaluations.

Sending MIT commands at 500 Hz does not change the motor firmware's internal PD
frequency. The useful change is a 500 Hz command/feedback exchange and 500 Hz
delivery of dynamic torque saturation and parallel-linkage mapping. This reduces
one deployment mismatch; it does not make the physical system identical to the
serial-actuator training simulation or establish that transport causes the gait.

## Current implementation findings

- Four ankle motors have a nominal 500 Hz schedule; 27 others have 50 Hz slots.
  Coherent sending additionally transmits the whole group for every new action,
  merging a coincident nominal transmission. Thus non-ankle motors are not
  strictly receiving only 50 frames/s in the current candidate.
- Motor-space preview already runs at 500 Hz. Delivery of recalculated protected
  targets to the other joints is less frequent.
- The loader rejects more than two 500 Hz endpoints per bus. The poll-phase
  allocator, strict feedback collection and fixed 6.5--8.5 ms publication window
  encode assumptions from the mixed-rate schedule.
- Temperature-filter integration and RMS windows depend on nominal poll rate.
  Raising samples without preserving physical time would change protection
  semantics even if numeric thresholds stayed identical.
- TX evidence reserves 200,000 records per socket. Eight motors at 500 Hz need
  198,000 records for the 49.5 s active phase alone, before preparation and final
  disable verification. Capacity must be calculated for the full lifecycle.

## Proposed scheduling

Retain one CAN owner with the verified CPU 2 / SCHED_FIFO 50 profile. The policy
remains on CPUs 4/5 and the IMU on CPU 3. The existing 0.5 ms service tick can
continue draining feedback and timestamp completions, with full motor output
only at each 2 ms boundary. Avoid an additional send whenever a policy action
arrives: latch it at the next boundary, so the new mode is 500 Hz, not 550 Hz.

At a boundary, first drain available feedback and IPC. Atomically select the
complete accepted action, snapshot all motor positions/velocities and compute
the entire mapped/protected payload array. Validate it before the first write.
Dispatch the 31 commands without waiting synchronously for each reply; interleave
the four interfaces by motor rank so one interface is not filled before the next
starts. All commands in a group share one action identity and control-tick ID.
Bus serialization is unavoidable; measure it instead of claiming simultaneous
physical actuation across four unsynchronized adapter clocks.

Consume replies between groups. Publish exactly one fresh observation per 20 ms
at a deterministic policy phase, retaining the existing policy phase initially
where practical. Replace assumptions about collecting slow endpoints with
timestamp-based checks for complete recent feedback. Do not append 500 samples/s
to a history trained at 50 samples/s. Motor replies do not echo policy sequence
numbers: keep real receive timestamps and never invent reply/action association.

Preserve the original 20 ms source-feedback-to-new-action deadline, pending
action timeout, target watchdog and 2 ms wake guard. Repeating a held action
must not refresh its source timestamp or make a stalled policy appear alive.
Keep action identity, motor feedback freshness and repeated-control-tick identity
separate. Repeated sends alone are not grounds to restart a policy deadline.
Keep final 31-motor disable acknowledgement, including early exits.

## Transport budget

Read-only inspection on November 11 found all four interfaces at 1 Mbit/s nominal
and 5 Mbit/s data rate, using eight-byte CAN-FD/BRS command and feedback frames.
All four channels share the same USB parent, so per-bus CAN capacity does not
prove USB/adapter throughput. Nominal arbitration, framing, stuffing, ACK and
replies must be included; the 5 Mbit/s rate does not apply to the whole frame.
See the [SocketCAN documentation](https://docs.kernel.org/networking/can.html).

| Interface | Motors | Commands/s at 500 Hz | Frames/s including one reply/command |
|---|---:|---:|---:|
| kcan1 | 8 | 4000 | 8000 |
| kcan2 | 8 | 4000 | 8000 |
| kcan3 | 8 | 4000 | 8000 |
| kcan4 | 7 | 3500 | 7000 |
| Total | 31 | 15500 | 31000 |

This is approximately 3.2 times the current schedule's upper bound of 4900
commands/s (3350 nominal plus at most 1550 action-commit sends, with some merged).
The measured 0.703 ms maximum per-bus group span in the latest disabled trial is
encouraging but was obtained at 50 whole-action groups/s, not sustained 500 Hz.
It does not establish that every 2 ms request/reply cycle will complete.

Use actual adapter TX stamps, per-motor RX times, socket queues and loss/error
counters to qualify sustained throughput. A host loop at 500 Hz is not proof
that the motors receive frames at 500 Hz. Reject accumulating queues and missed
cycles; do not enqueue bursts to catch up after a scheduling overrun.

## Implementation and validation sequence

1. Add an explicit all-joint schedule profile with separate validation-cache
   identity. Preserve the current mixed-rate/coherent mode for comparison.
2. Replace the two-fast-motors allocator, full-lifecycle scheduling and feedback
   collection assumptions. Generate a dedicated 500 Hz transport configuration;
   verify that every non-rate motor field and all gain/mapping hashes remain
   unchanged. Keep 1 s/10 s RMS windows and the 2 s temperature time constant in
   physical time, with one monitor update per actual admitted sample.
3. Preallocate TX/RX evidence for the complete experiment outside realtime
   control. Record policy action ID, 500 Hz tick ID, per-motor dispatch/completion
   times, feedback age and saturation. Extend audits beyond new-action groups.
4. Test deterministic scheduling, full-group action consistency, no stale packet
   catch-up, unchanged PD limits and all error/disable paths. Frozen 50 Hz logs
   can check mapping but cannot invent the missing 500 Hz physical feedback.
5. Run authorized disabled hardware qualification for the full experiment,
   including rate/jitter/queue checks and deliberate policy timeout. Only after
   success, compare supervised walking with the same gains, speed command and
   startup procedure. Compare foot clearance/drag, heading and stop/start behavior,
   not only whether the robot survives.

Expected benefit is fresher all-body feedback and prompt delivery of state-based
PD protection. It cannot by itself repair outward foot placement already present
in simulation, and does not accelerate ONNX inference. The first November 11
validation's 13.97 ms inference tail remains a separately recorded issue.
