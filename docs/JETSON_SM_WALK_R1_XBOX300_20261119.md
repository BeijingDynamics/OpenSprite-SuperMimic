# Sprite SM-Walk R1: five-minute Xbox window

The operator requested a bounded five-minute Xbox window after the first
successful faster-start 120-second run. This is an isolated duration extension,
not an indefinite-control mode.

## Deployment

New stage: `/home/tony/sprite-sm-walk-r1-xbox300-20261119`.
The previous `/home/tony/sprite-sm-walk-r1-fast-20261119` stage remains intact.

The intended operator entry is:

```sh
bash /home/tony/sprite-sm-walk-r1-xbox300-20261119/operator-xbox.sh
```

The script requires same-build no-enable and deadline-fault evidence before it
can prompt for powered operation. Qualification status is recorded below; the
existence of this document alone does not qualify the deployment.

## Scope

- Xbox commands are admitted for 300 seconds. The initial supported standing
  checkpoint remains 20 seconds, and braking/zero-command shutdown remains at
  the end. Total native control duration is 326 seconds.
- Forward limit remains 0.40 m/s, reverse 0.05 m/s, yaw +/-0.20 rad/s and lateral
  command zero. Acceleration and braking remain 0.10 m/s².
- R1 ONNX, full training Kp/Kd for all 31 joints, action history, direct policy
  takeover, PD command saturation, motor fault checks and live timing guards
  are unchanged.
- Ordinary-joint 50 Hz / parallel-joint 500 Hz scheduling and coherent action
  commits remain unchanged. CAN stays on CPU 2 / FIFO 50. Inference main stays
  on CPU 4, ORT worker on CPU 5, receive threads on CPU 1, with idle spinning
  disabled. The CPU-frequency lease still restores the previous settings.
- Instantaneous feedback torque remains observation-only, as explicitly
  requested for the preceding profile. Other protections remain in force.

Longer recording requires 16,301 policy frames, a bounded fixture capacity of
16,301, and capacity for 700,000 TX events per socket. The IMU recording bound
is 360 seconds. The post-control TX audit has a 90-second process timeout;
this does not change any live control deadline. Native allocation occurs
before control begins.

LT release brakes the velocity command; it does not disable motors. The dormant
LB+RB disable-combo implementation is not enabled by this production entry.
Terminal Ctrl+C shutdown is a separate no-enable qualification case. Retain fall
protection and the motor power switch throughout powered testing; do not infer
unsupported balance from the body-sensor admission message alone.

## Evidence and issues

The previous 120-second powered run
`xbox-powered-20261119-165413-12501` passed, with 31 enable attempts and all 31
motors individually confirmed disabled at the end.

The new native build passed all 10 tests. A startup-only attempt exposed the
old Python/native fixture limit of 10,000 frames. Both readers now support the
bounded 16,301-frame profile. Python rejects 0 and 16,302 frames, and the native
reader accepts the complete larger fixture. A 1,000-frame CPU recurrence check
matches the original output byte for byte:

`0bc4612dab90c5e78b8cb59a82110b07cb885bf5b4e3d7cec71e6a26dd8ac076`.

The first hardware no-enable attempt (`disabled-02`) stopped at state 11 under
the original 20 ms oldest-feedback protection. CAN published the state about
5.83 ms after its oldest feedback. Actor receipt followed publication about
11.57 ms later, and inference took about 3.63 ms. The delay therefore occurred
before inference, rather than being a large model inference. This evidence
does not establish the underlying scheduler cause. All 31 motors were
individually confirmed disabled, with zero enable attempts. The failed audit
and raw evidence are retained; no live timing limit was relaxed.

The unchanged-build repeat (`disabled-03`) completed all 326 seconds of native
control. Its TX audit passed for 16,300 groups / 505,300 motor dispatches and
all 31 motors were confirmed disabled. The overall run correctly failed because
the actor timed out waiting for controller IPC closure: the controller closed
IPC only after serializing the longer TX log. This was a post-disable ordering
defect, not a live protection failure.

The controller now closes IPC after final disable verification and the operator
status event, before serializing traces. The supervisor still requires the CAN
report and all 31 readbacks. The 2-second actor completion wait and 20 ms live
deadline remain unchanged. A native socket-pair regression verifies immediate
EOF, retained evidence counters and idempotent cleanup. All 10 tests passed.

Final-build `disabled-04` passed the full 326-second control schedule and
post-control audits. There were zero enable attempts, all 31 motors were
individually confirmed disabled, and the CPU minimum frequency was restored
to 729,600 kHz. Maximum measured inference was 5.789444 ms and maximum
oldest-feedback-to-target-accept time was 12.820813 ms. These are finite-run
measurements, not hard real-time guarantees.

Final-build CAN SHA256:
`d94fd40af0d27801fcb15ffb50b4081d3f735b7a612e142d5f52af52e3b59826`.

Final-build actor SHA256:
`0fac1b2c893637448791e708980d01b08e461a5368212edfa0c48d24623d2c3f`.

The first planned late-window fault trial (`fault-01`) stopped naturally at
state 2,593, before the planned injection. Its actor state receipt lagged CAN
publication by approximately 11.54 ms, then inference took approximately
3.13 ms; the original 20 ms guard stopped control. All 31 disabled readbacks
were obtained. This was correctly marked failed, not counted as successful
fault injection. Together with `disabled-02`, it demonstrates an intermittent
pre-inference delay whose underlying scheduler cause remains unresolved.
Passing later finite runs does not erase these failures or guarantee five
minutes of uninterrupted powered operation.

The final-build repeat `fault-02` passed the planned late-window injection:
CAN stopped on `target_ipc_pending` under the original 20 ms guard, zero motor
enable attempts were made, all 31 motors were confirmed disabled, and CPU
frequency was restored. The operator entry references `disabled-04` and
`fault-02`; failed attempts remain separate evidence.

Battery exhaustion interrupted connectivity before a terminal-interrupt result
was available. After battery replacement, CAN scheduler permission survived,
but the installed driver reverted to `A2BA84DDB53927675FFA1CB`. The operator
restored the qualified temporary driver `404E8D65A6EFF60079EAEC1` using:

```sh
sudo bash /home/tony/sprite-sm-walk-r1-fast-20261119/load-echo-order-driver.sh
```

This loader configures all four buses and their timestamps without sending
motor frames. After future reboots, use this loader for the R1 fast/five-minute
profiles; the earlier combined setup still selects the previous driver.

The first post-reboot interrupt attempt (`interrupt-01`) stopped before policy
takeover with `startup pose outside limits`; all 31 disabled readbacks were
obtained. The failing joint was not recorded, so its identity is unresolved.
A subsequent two-second zero-gain sample (`pose-after-reboot-01`) received
3,100/3,100 feedback frames, confirmed all 31 motors disabled and found every
joint within the unchanged limits. No limit was relaxed.

`interrupt-02` then passed the terminal process-group SIGINT test during the
Xbox window: zero enable attempts, 31 final disabled readbacks, CPU minimum
restored to 729,600 kHz and no remaining control processes. SIGINT exit code
`-2` is expected for this test and is not a normal full-run success code.

The final read-only operator preflight passed. Deployment does not initiate a
powered five-minute test. The operator still confirms the physical conditions
at the `READY` prompt. For an orderly early stop, release LT and allow braking
first; terminal Ctrl+C requests shutdown and motor disable, not controlled
braking. Retain fall protection when disabling.

## Local backup and final audit

The complete archive is saved locally under
`outputs/deploy-sm-walk-r1-xbox300-20261119/validation-evidence.tar.gz`.
Its 600,142,546-byte length and SHA256 were verified after transfer:

`f05f5183cbc7843f9b0ca3e55266efd3e2d234d867ef11752924d4fb8c6e8a5a`.

It contains exact source, the qualified CAN and actor binaries, the frozen R1
ONNX, raw successful and failed trials, reboot pose/interrupt evidence and the
preceding successful 120-second powered run. Fifteen key files were extracted
to the adjacent `validated/` directory. `local-backup-verification.json`
records the completed verification.

All 1,593,721 TX records in `disabled-04` and 1,529,724 in `fault-02` have
software and hardware timestamps plus hardware-completion receipt timestamps.
Maximum per-bus capacity usage was 552,278 events, below the new 700,000 bound.
The full normal run measured median inference 2.788213 ms, maximum inference
5.789444 ms, and maximum oldest-feedback-to-all-TX-receipt 13.864416 ms.
The left hip hardware send interval peaked at 24.104 ms. No hard real-time
guarantee or powered-motion result is implied by these no-enable measurements.
