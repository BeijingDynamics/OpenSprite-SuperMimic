# Sprite SM-Walk R1: Jetson deployment

This deployment is separate from the four-architecture rental-GPU experiment.
It uses the frozen `sprite-sm-walk-r1-20261119` candidate, not an experimental
small actor. Stage: `/home/tony/sprite-sm-walk-r1-20261119` on Jetson 253.

## Identity and unchanged execution

- Checkpoint SHA256: `70cf3e70354c8b574777709f56f24f83864329d589c256cfa97d6a1236afdedd`.
- ONNX SHA256: `5695b6980746534a11d075befc29bf5f7fe0ef2a7d81e3dadf3147c50f4d6930`.
- Native CAN and policy binaries are reused from the validated mixed-rate
  Xbox runtime. No recompilation or new realtime capability grant is needed.
- Direct joints retain 50 Hz target transmission; parallel joints retain
  500 Hz. Policy inference remains 50 Hz, two ORT threads on CPUs 4/5.
- The existing frequency service holds CPUs 4/5 at 1.344 GHz during inference
  and restores the prior settings afterward. CAN remains CPU 2 / FIFO 50.
- All 31 joints retain training Kp/Kd, action scale and offsets. Native history,
  raw previous-action semantics, IMU processing, dynamic torque saturation,
  differential mapping and every existing protection limit remain unchanged.

The runtime identity allowlist adds only this exact training-run/checkpoint/ONNX
triple; it does not accept arbitrary new models. Five binding tests pass,
including rejection of changed gains, caps, offsets and unreviewed R1 hashes.
The original bundle/contract and old runtime stages remain unchanged. Staging
compares 25 observation, physical and control fields with the old live contract.

## CPU replay

Seven recorded simulation cases, 1,000 frames each, run through the existing
C++ replay binary at 50 Hz with two CPU threads and the frequency lease.
Native/Python recurrence and recorded-simulation action parity must both pass.
Prewarm uses separate state; formal history starts fresh. CPU replay opens no
CAN/IMU devices and sends no motor frames.

## Live no-enable gate

`run-disabled.py` exposes only disabled operation and intentional deadline
injection. It preserves the existing 54-second Xbox profile to test the same
runtime used for the subsequent bounded operator trial. With no controller
connected, all commands remain zero. This is not a new nonzero joystick
mapping qualification. IMU stationary calibration lasts approximately 15 seconds
and estimates bias; it does not redefine the robot as level.

Both the full disabled run and unchanged 20 ms timeout-injection run must pass,
record zero enable attempts, and individually read back all 31 motors disabled.
No archive compression or bulk transfers run on the Jetson during these tests.
Successful timing is transport evidence, not proof that R1 stands on hardware.

## After a battery-change reboot

Run the existing combined setup in WindTerm:

```bash
sudo bash /home/tony/setup_supermimic_can_on_253.sh
```

This restores the qualified temporary CAN driver, all four buses and TX/RX
timestamps. The SPI and frequency services persist across boots. The rental
GPU campaign is independent and is not stopped for Jetson work.

`operator-xbox.sh` is generated only after matching R1 disabled/fault evidence
passes. Its `--check` path is read-only. Powered operation is not started by
deployment or validation: it requires the operator's current onsite conditions
and explicit `READY` in WindTerm. The existing full-gain direct takeover,
20-second supported initial phase, 30-second Xbox window, command caps,
shutdown behavior and 31-motor disable confirmation are retained.

Local evidence directory: `outputs/deploy-sm-walk-r1-20261119`.

## Completed validation

Seven CPU cases / 7,000 frames passed. Native/Python recurrence matched exactly;
maximum action error against recorded simulation was 4.292e-6. CPU-frequency
leases restored the original 729600 kHz floor after every completed session.

`fixed-disabled-01` completed the full 54 seconds, with 2,701 recorded input
frames and 2,700 formal actions / 83,700 first motor transmissions. All commands
were zero. The unchanged timing/TX audit passed. `fixed-fault-01` passed the
intentional actor-pause test of the original 20 ms deadline. Both recorded zero
enable attempts and individually confirmed all 31 motors disabled. No control
process remained at the final audit. No powered test was performed.

| Live measurement | Median | P99 | Maximum |
| --- | ---: | ---: | ---: |
| Inference | 2.864 ms | 3.012 ms | 6.923 ms |
| Oldest feedback to all driver handoffs | 9.577 ms | 9.778 ms | 13.737 ms |
| Oldest feedback to all TX completion receipts | 10.387 ms | 10.794 ms | 14.370 ms |
| Left-hip new-action hardware interval | 19.992 ms | 20.126 ms | 24.108 ms |

The single long inference and corresponding update interval remain recorded;
fixed CPU frequency does not eliminate every tail. All transmitted groups met
the existing checks. TX receipt is a host upper bound, not motor-internal torque
execution time; a target interval above 20 ms is distinct from the protected
age of feedback used by each newly accepted target.

The Xbox was initially absent but appeared at 13:25 during CPU validation.
The live trace subsequently records it connected in all 2,701 frames, with no
permission granted and all commands zero. This qualifies zero-command policy
transport, not a new nonzero Xbox direction test. The final v2 evidence summary
derives controller connection from its CSV rather than initial device presence;
the first summary's disconnected-controller statement was corrected. Raw logs
are unchanged. The generated R1 operator entry passed its read-only preflight.
