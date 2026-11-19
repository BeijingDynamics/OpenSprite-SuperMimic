# PM01 live-state and Xbox preview qualification: November 16, 2026

## First powered zero-command result

### Repeat with operator-reported slack sling and push response

For `stand-powered-20261116-143533-2690`, the operator reported that the robot
stood with the sling slack and stepped forward when pushed on the body. This
is useful qualitative evidence of unsupported standing and disturbance-induced
stepping, not a measurement of recovery success or commanded walking speed.
All 1001 recorded command vectors were exactly zero; Xbox was not connected
to control. The script stopped at the 20 s standing checkpoint because the
joint/body settling counters were zero. It did not open the subsequent sling
window or complete the configured 30.5 s protocol. Near the checkpoint the
body was moving: final gyro norm was 0.592 rad/s, with a 1.718 rad/s peak in
the last approximately one second. Final tilt was 6.946 degrees. Exact push
timing and sling load are not measured by these logs.

The only CAN-reported error was `zero-command standing not settled at sling
checkpoint`; there were no deadline misses or missing motor feedback. Maximum
feedback-to-target acceptance was 13.294 ms. The actor's subsequent state
receive error followed CAN shutdown; it was not the initiating fault. TX audit
passed and all 31 motors were individually confirmed disabled. No control
process remained when inspected. Local evidence preserves the failed protocol
result alongside the operator's positive physical observation. No gains or
protection thresholds were changed, and no further powered run was started.

`stand-powered-20261116-143200-2547` completed the 30.5 s zero-command protocol.
The operator explicitly reported that the sling was NOT slackened. This is a
supported powered test, not evidence of unsupported standing. The sensor and
joint settling checks passed, with final-second tilt peak 4.439 degrees and
gyro norm peak 0.00367 rad/s. Full training gains were used on all 31 joints,
with direct takeover and no gain ramp. Maximum inference was 7.346 ms;
oldest-feedback-to-target-acceptance maximum was 13.309 ms. CAN reported no
errors, the TX audit passed, and all 31 motors were individually read back
disabled at exit. No control process remained running when inspected. Session
evidence was copied locally. The user intends to repeat the same operator
script; no new powered session was started by the assistant.

## Post-reboot CAN setup

Use this single command after reboot, before any robot test:

```bash
sudo bash /home/tony/setup_supermimic_can_on_253.sh
```

The maintained source is `scripts/setup_supermimic_can_on_253.sh`. It delegates
to the pinned, previously validated temporary driver loader, which restores
the patched driver, configures kcan1 through kcan4 at 1 Mbit/s arbitration and
5 Mbit/s data, enables/verifies hardware TX/RX timestamps, and restores/checks
the shared CAN binary's CPU 2 / FIFO 50 capability. It uses the existing
commissioning lock and rollback path. It sends no motor frames and does not
enable motors. Do not run it during a robot control session.

This replaces the two operator commands; the original CAN-only script remains
intact because the sealed driver loader verifies its hash. The installed
kernel module is unchanged, so reboot still restores the original driver.
CAN interfaces being UP alone does not establish timestamp readiness.

After the battery-change reboot on November 16, the observed loaded driver
srcversion was `2371CFECC14D355A8DCBA83`, although all four CAN interfaces were
UP. The required temporary driver's srcversion is `A2BA84DDB53927675FFA1CB`.
The combined entry point was deployed with matching local/remote SHA256
`703cb995ff1349d3c17b1fa49337dfab65fe040b2f927237095f8e322ef8a419`.
Shell syntax and dependency-hash checks passed on Jetson. Its `--check` mode
only verifies dependencies; it does not configure or certify current buses.
The operator subsequently executed the privileged setup. Readback confirmed
driver srcversion `A2BA84DDB53927675FFA1CB`, TX type 1 and RX filter 1 on all
four adapters. kcan1..kcan4 were UP in CAN-FD mode at the expected bitrates,
with zero reported RX/TX errors or drops and no bus-off events. The standing
script's read-only preflight passed, including the qualified evidence/file
identities and CAN CPU 2 / FIFO 50 scheduler check. These follow-up checks sent
no motor frames and did not enable motors. Powered operation still requires
the operator's current on-site confirmation in the test script.

All sessions in this record were disabled-motor tests. No powered admission
has been granted. Existing gains, limits, dynamic PD saturation and the CAN
binary were retained. The CAN scheduler was verified as CPU 2 / FIFO 50;
actor affinity remains CPUs 4 and 5, using two ORT threads.

## Completed checks

The first live-state attempt stopped before IPC because hardware TX/RX
timestamps were not active after reboot. The user restored the previously
qualified temporary timestamp driver. SPI performance persistence was active.
ICM45686 800 Hz stationary bias calibration passed; it was not a horizontal
attitude reset.

`/home/tony/supermimic-pm01-live-1116/disabled-02` passed the 10-second zero
command test: zero enable attempts, all 31 motors read back disabled. Maximum
feedback-to-target-return age was 13.798 ms; inference maximum was 7.799 ms.
`fault-01` passed an intentional actor pause, tripping the unchanged deadline
at `target_ipc_pending`, again with all 31 disabled.

Xbox was enumerated as `/dev/input/js0`. The previously qualified mapping is
forward axis 1/sign -1, turn axis 2/sign -1, permission axis 5. Independent
input-only recording showed forward commands and both signed turn commands
up to +/-0.20 rad/s, followed by zero after LT release. The first turn attempt
used the wrong stick; the user's repeated right-stick test captured both signs.

## Integrated Xbox preview and remaining gate

An isolated actor at `/home/tony/supermimic-pm01-xbox-1116` adds
`--gamepad-preview-device`, reusing the native Xbox permission, disconnect,
deadzone and command-slew implementation. Current input-only caps are forward
0.10 m/s, reverse 0.05 m/s, yaw 0.20 rad/s; lateral command is zero.
The launcher explicitly rejects powered mode when this option is present.
It also rejects combining joystick and scripted walking commands.

The first 30.5-second integrated preview passed timing and protection checks:
maximum chain age 13.298 ms; zero enable attempts; all 31 disabled. However,
all 1526 recorded policy commands were zero. This is NOT evidence of successful
nonzero Xbox-to-policy integration. The same candidate passed deadline-fault
injection, with all 31 disabled. Neither result qualifies powered operation.

The next actor build adds per-frame connection/permission/raw-axis/command
audit records and emits `GAMEPAD_PREVIEW_READY`. The launcher relays that
readiness message to the terminal, eliminating reliance on guessed calibration
timing. Native gamepad and SuperMimic-history tests pass. This final logging
build completed its interactive nonzero-input run, same-build deadline check,
and integrated release-to-zero test as recorded below. Powered operation
remains unqualified.

### Manual preview at 14:01:04

`manual-20261116-140104` recorded 1526 policy frames over 30.5 seconds.
The controller stayed connected throughout. There were 690 nonzero-command
frames, including forward commands up to 0.10 m/s and both signed yaw commands
up to +/-0.20 rad/s. Lateral command remained zero. Commands were checked
against both the recorded policy inputs and the final three elements of the
actual 993-dimensional observation; the latter differed by at most 7.38e-9
(float32 conversion).

The recording ended with LT permission still active and command
`[0.10, 0.0, 0.20]`. This run does not verify release-to-zero from a nonzero
command. An earlier permission release occurred while commands were already
zero. Independent input-only release evidence must not be substituted for
this integrated check.

Inference median/p99/maximum were 3.061/6.912/7.664 ms. Maximum age from oldest
motor feedback to target acceptance was 13.829 ms, below the unchanged 20 ms
deadline; there were no deadline misses or CAN errors. The action TX audit
passed for 1525 groups / 47275 motor dispatches. Maximum oldest-feedback to
driver dispatch upper bound was 15.555 ms; driver dispatch is not motor
execution acknowledgement.

Enable attempts were zero, and all 31 motors were read back disabled at exit.
No powered standing or walking conclusion follows from this test. Gains and
protection were unchanged. Integrated nonzero-command release remains unverified.
The derived audit is stored alongside the immutable session directory as
`manual-20261116-140104-command-audit.json` in the local evidence archive.

### Same-build deadline injection

`fault-02` passed the deliberate actor pause test. All recorded fingerprints
match the manual preview, including the actor, launcher, CAN binary, model,
gain tables and limits. The original oldest-feedback 20 ms protection rejected
the missing target at `target_ipc_pending`. There were zero enable attempts;
all 31 motors were read back disabled. Evidence was copied back locally.

The remaining interactive check should release LT while a nonzero forward or
turn command is present, then leave at least five seconds of recording. The
existing command limiter decelerates toward zero: 0.10 m/s^2 for translation
and 0.20 rad/s^2 for yaw. At the preview caps, nominal return time is one second
after permission revocation; this is not an instantaneous command-zero promise.

### Integrated release-to-zero at 14:13:22

`manual-20261116-141322` passed the remaining disabled-only interactive check.
At frame 560, LT permission changed from enabled to revoked while the previous
command was `[0.10, 0, 0.104054]` (m/s, m/s, rad/s). Both sticks remained
deflected: forward axis -1.0 and turn axis -0.577837. The actual policy command
reached zero at frame 610, while both axes were still at those values. The
interval between recorded state-receive timestamps was 0.999947 seconds;
this measures sampled permission-to-command response, not Bluetooth event
latency. Commands remained zero for all 916 remaining frames.

All 1526 samples were connected. Recorded commands match the actual policy
observation to within 3.64e-9 (float32 conversion). The fingerprint set exactly
matches `fault-02`. Inference median/maximum were 2.997/7.034 ms; maximum
oldest-feedback-to-target-acceptance age was 13.302 ms. No CAN errors or deadline
misses occurred; the action TX audit passed. There were zero enable attempts
and all 31 motors were read back disabled at exit. Gains and protections were
unchanged. This completes these disabled-only integration checks; it does not
qualify powered standing or walking, and the preview launcher still rejects
powered mode.

Session logs and `manual-20261116-141322-command-audit.json` are stored in the
local evidence archive.

### Fixed-zero standing script preparation

The unchanged native build and launcher passed `stand-zero-disabled-01` and
`stand-zero-fault-01`, both with the scripted zero-command source and a 30.5 s
configured duration. The normal run's oldest-feedback-to-target-acceptance
maximum was 14.287 ms. All recorded command inputs were zero. Both sessions
had zero enable attempts and confirmed all 31 motors disabled. The fault test
verified the same original 20 ms rejection; no protection was relaxed.

`scripts/jetson_pm01_stand_zero.sh` is deployed as
`/home/tony/supermimic-pm01-xbox-1116/operator-stand-zero.sh`. Its `--check` path
passed the evidence, current file hash, loaded driver and CAN CPU 2 / FIFO 50
checks without starting a hardware session. Shell syntax was also checked.
The normal path requires the operator to type `READY` before starting IMU
calibration and powered commissioning. Xbox is not opened by this test.
Full training gains, direct policy takeover, fresh history, dynamic PD
saturation and existing limits remain unchanged.

The operator keeps sling support during the first 20 s. Only after the existing
standing check may the operator gradually slacken the sling, provided the robot
is supporting itself, while retaining fall protection and immediate power
cutoff access. The remaining window is 10 s. Sensor qualification is not proof
of unsupported balance. The launcher requires final per-motor disable feedback;
incomplete feedback or a remaining green motor indicator requires physical
motor power cutoff. No powered run was executed during script preparation.

Run the disabled-only interactive test in WindTerm:

```bash
bash /home/tony/supermimic-pm01-xbox-1116/operator-preview.sh
```

Keep the robot stationary and retain sling support. Wait for
`GAMEPAD PREVIEW READY`, center both sticks, press/release LT, then hold LT
while testing left-stick forward and right-stick left/right; release LT at
the end. No sling release or motor enabling is part of this preview.

All collected completed-session evidence is archived locally under
`outputs/deploy-20261116/live-evidence`. Original deployment and models remain
available for rollback. Xbox input-only logs are under `jetson-evidence`.
