# Bounded PM01 Xbox control trial

## Body-only admission correction

The corrected stage is `/home/tony/supermimic-pm01-xbox-body-1116`.
`--xbox-body-admission` is accepted only for direct, coherent, bounded sling
commissioning without scripted walking. It uses the same body-only readiness
predicate as the existing walking branch. The joint settling counter remains
in diagnostics. The Python end-of-run validator also uses body-only settling
for Xbox control, avoiding a second accidental joint-error gate at shutdown.
The 0.15 rad/s body angular-speed criterion, 10 degree tilt criterion, full
50-frame stability requirement and current-target/sequence checks are intact.
All motor-space protections, deadlines, gains and gamepad limits are unchanged.

Ten native tests passed, including explicit regression cases for the recorded
body-ready/joint-unsettled state and rejection of an unstable body or stale
accepted target. Three recorded sessions were re-evaluated: the original
supported standing pass still passes, the moving-body push trial still fails,
and the ankle-error-only rejection now passes. The policy actor is unchanged
(SHA256 `6ee5bd70b83fc18f97b99cce20020ee89cd2a67f8c4dcdffb76695392d0c4f28`).
The new CAN SHA256 is
`52508b976aa6024732a5a15e5317916256dd4b601d3bbd3b41effff2743b718a`;
the launcher SHA256 is
`2741e57e085a8dbabcad152d365e250f368d3c9bd66bfbf52a98e672a7fff14a`.
The operator restored CAP_SYS_NICE on this new CAN executable, verified before
live disabled validation. The normal operator script now invokes the scheduler
helper automatically, so later missing capabilities are caught before enable.

The first corrected disabled run passed (maximum feedback-to-target age
13.827 ms, all 31 disabled). The first planned fault run did NOT pass: a real
deadline trip occurred at sequence 631 before injection. Recorded inference
wall time was 14.900096 ms, feedback-to-receive was 5.826578 ms, and total
feedback-to-target construction was 20.829427 ms. The actor rejected the target
before sending (`sent_ns=0`); CAN also rejected the pending target and confirmed
all 31 disabled. No CAN scheduler miss or missing motor feedback was reported.
The system cause of this isolated inference long-tail is not established.
The failed session is retained as `fault-01`, with a derived timing audit.
Same-build follow-up checks use new session directories; no deadline limit,
CPU governor, affinity, gain or motor protection was changed to obtain a pass.

The unchanged-build `disabled-02` subsequently completed all 30.5 s with a
13.321 ms maximum feedback-to-target age. `fault-02` then passed the deliberate
actor pause inside the input window at about 24.5 s, tripping the original
20 ms pending-target protection. Both had zero enable attempts and all 31
motors confirmed disabled. Their fingerprints match exactly; native reports
confirm `xbox_body_admission=true` and `walk_joint_settling_required=false`.
The final operator script selects these two passing sessions and its read-only
preflight passed. The earlier real inference long-tail remains an observed
limitation, not a resolved performance issue. All sessions remain archived.

## First powered attempt: admission mismatch

`xbox-powered-20261116-145058-3440` stopped at the 20 s standing checkpoint.
Body readiness passed (50 frames); joint readiness was zero. The left ankle
pitch goal was 0.380838 rad versus measured 0.235366 rad, an error of 0.145472
rad (8.335 degrees), exceeding the legacy all-joint 0.10 rad settling rule.
The final 50 samples had a gyro-norm maximum of 0.004309 rad/s and tilt maximum
of 8.271 degrees, within the unchanged 0.15 rad/s and 10 degree body criteria.

The integration used the zero-command sling branch, whose `!walking` argument
still requires joint settling. Previously the scripted walking branch had
already switched to body-only readiness. This branch mismatch blocked Xbox
admission before the input window opened. All 1001 recorded policy commands
were zero; this is not evidence that the policy ignores a 0.25 m/s command.

No torque saturation was reported. There were no deadline misses; maximum
feedback-to-target acceptance was 13.309 ms. All 31 motors were confirmed
disabled and no controller process remained when inspected. The actor receive
error followed the intentional CAN shutdown. Logs and a derived diagnosis
were archived locally. No threshold, gain or runtime code was changed during
this diagnosis. The required correction is to align the Xbox admission path
with the established body-only walking path while preserving all runtime
motor, body and freshness protections; it requires fresh validation before
another powered trial.

The operator requested powered Xbox control and a forward command maximum of
0.25 m/s. This trial uses the primary PM01 model, all 31 training gains, direct
policy takeover and raw policy-action history. Existing motor torque, speed,
position, temperature, body and 20 ms freshness protections are unchanged.
No torque feedforward is added. The initial trial reused the qualified CAN
executable by symlink. The body-admission correction replaces that executable
in a separate stage while retaining CPU 2 / FIFO 50 scheduling. The actor
from `/home/tony/supermimic-pm01-xbox-control-1116` is reused unchanged.

## Operator command

```bash
bash /home/tony/supermimic-pm01-xbox-body-1116/operator-xbox.sh
```

The script verifies same-build disabled and fault evidence, then asks for
`READY`. Confirm feet on the floor, initial sling support, fall protection
covering at least 3 m of clear level travel, nobody touching the robot and
immediate access to the motor power switch. Do not push the robot in this
command-control trial. The script performs stationary IMU bias calibration.

- The first 20 s stay at zero command and retain the existing standing check.
- After the standing-check message, slacken the sling only if the robot is
  supporting itself, keeping fall protection attached.
- At `XBOX_WINDOW_OPEN`, center sticks, release LT and then press/hold LT.
  A trigger held across the opening cannot authorize movement.
- Left stick controls forward/reverse, right stick controls yaw; lateral
  command remains zero. Caps are +0.25/-0.05 m/s and +/-0.20 rad/s.
- Translation acceleration remains 0.05 m/s^2, deceleration 0.10 m/s^2;
  yaw slew remains 0.20 rad/s^2. Reaching full forward command from zero takes
  approximately 5 s. LT release requests braking, not motor disable; full
  forward command takes up to 2.5 s to reach zero.
- The input window lasts approximately 7 s. At about 27 s it closes and
  commands decelerate toward zero regardless of LT/sticks. The bounded session
  ends at 30.5 s and requires individual disable feedback from all 31 motors.
  Incomplete disable feedback or a remaining green motor indicator requires
  physical motor power cutoff.

The short first trial preserves the existing CAN timing protocol. It is not an
indefinite teleoperation service. Startup/end posture checks remain active.
The older preview-only option still rejects powered mode; the new explicit
control option cannot be combined with scripted walking or preview input.

## Validation and evidence

Native `gamepad_control`, `gamepad` and `supermimic_core` tests passed on Jetson.
The new test covers a pre-held trigger, fresh in-window permission, the 0.25
command cap, LT release, disconnect braking, the timed stop and stale input.
Legacy profiles still reject forward caps above 0.10 unless the new explicit
control profile is selected. Python compilation and shell syntax checks pass.

`disabled-01` completed 30.5 s with no motor enable attempts and all 31 motors
read back disabled. Maximum oldest-feedback-to-target acceptance was 13.299 ms.
`fault-01` paused the actor about 24.5 s into the session, inside the input
window. The unchanged 20 ms rejection triggered at `target_ipc_pending`; all
31 motors were again confirmed disabled. Both evidence fingerprint sets match.
The operator script's read-only preflight passed. These runs qualify the
transport and shutdown path; they do not establish powered walking performance.
Nonzero input semantics have native tests plus the earlier integrated preview
records; do not describe the unattended new disabled run as a user-driven
0.25 m/s trial without checking its recorded commands.

Actor SHA256: `6ee5bd70b83fc18f97b99cce20020ee89cd2a67f8c4dcdffb76695392d0c4f28`.
Launcher SHA256: `c6845c06b779d80e820ca1cccd5f1fec7afaed248f9d3b8e0da9fff7375bb9ad`.
CAN SHA256: `8dc2c32bc4d611c38568ad574f4ee84a25df7605c2f44b4fb858556d530742ad`.

Sources, script and evidence are archived locally under
`outputs/deploy-20261116/live-evidence/supermimic-pm01-xbox-control-1116`.
The prior standing and preview directories remain intact for rollback.
No powered Xbox run was started by the assistant during preparation.
