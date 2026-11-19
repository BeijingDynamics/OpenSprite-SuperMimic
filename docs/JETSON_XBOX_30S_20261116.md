# Thirty-second Xbox window

The operator requested a full 30 seconds of Xbox control. The isolated stage
is `/home/tony/supermimic-pm01-xbox30-1116`; prior builds remain unchanged.
This supersedes the approximately seven-second input window described in
`JETSON_XBOX_CONTROL_20261116.md`.

## Schedule and unchanged controls

- First 20 s: zero command and the established body-only standing admission.
- Actor frames 1002 through 2501: exactly 1500 frames / 30 s of Xbox input.
  Fresh centered LT release/press is required after opening; a pre-held trigger
  cannot initiate movement. LT may be released and re-engaged during the window.
- Frame 2502 onward: inputs are inhibited and commands decelerate to zero.
  Up to 2.5 s is needed from full forward command. The full session ends at
  54 s, leaving a final quiet interval and requiring all 31 disable readbacks.

Forward/reverse caps remain +0.25/-0.05 m/s; yaw cap is +/-0.20 rad/s and lateral
command is zero. Translation acceleration/deceleration remain 0.05/0.10 m/s^2.
All 31 training gains, raw-action history, direct takeover, dynamic PD torque
saturation, position/speed/body/thermal protections and the 20 ms deadline are
unchanged. IMU capture lifetime is extended to cover calibration plus the full
session; its calibration, fusion and sampling configuration are unchanged.

The CAN parser requires the exact 54 s profile and a dedicated
`ENABLE_SUPERMIMIC_XBOX_30S` or `PREVIEW_SUPERMIMIC_XBOX_30S` acknowledgement.
Other standing/walking durations and the overall 60 s hard session bound are
unchanged. Xbox status events now describe its own standing and ending phases;
the former 10 s sling countdown is not reused for the longer input window.

Continuous full-forward input for the entire window has approximately 7.2 m
nominal commanded travel including acceleration/braking. Plan the movement
area and fall-protection travel accordingly, and release LT before reaching a
boundary. The window length is not an instruction to hold forward throughout.
LT release brakes the command; it does not immediately disable motors.

## Validation

Ten native tests passed on Jetson, including exact window length, full-speed
hold through the extended window, timed braking, release/disconnect behavior,
fresh permission, duration/acknowledgement rejection, and body-only admission.
Python compilation, shell syntax and whitespace checks passed. Live disabled
and intentional deadline tests are required on this exact build before the
operator script may enable motors. No powered trial is started during setup.

After the operator restored realtime capability, `disabled-01` completed all
54 s / 2701 policy frames. The full window opened and closed, with maximum
oldest-feedback-to-target-acceptance age 14.812 ms. `fault-01` then passed an
intentional actor pause at approximately 45.5 s, inside the latter part of the
Xbox window, triggering the unchanged 20 ms `target_ipc_pending` rejection.
Both sessions had zero enable attempts and all 31 motors confirmed disabled.
The build fingerprints match exactly. The final operator script's read-only
preflight passed, including CPU 2 / FIFO 50 and the body-only admission mode.
Evidence and the final source/script copies are saved locally under
`outputs/deploy-20261116/live-evidence/supermimic-pm01-xbox30-1116`.

Run the qualified bounded operator entry point in the onsite terminal:

```bash
bash /home/tony/supermimic-pm01-xbox30-1116/operator-xbox.sh
```

Type `READY` only after confirming the physical conditions printed by the
script. Wait for the body-stability message before slackening the sling, and
for `XBOX_WINDOW_OPEN` before the fresh centered LT release/press sequence.
Retain fall protection and immediate access to the motor power switch.
Incomplete disable feedback or a remaining green motor indicator after exit
requires physical motor power cutoff.

Build SHA256 identities:

| Component | SHA256 |
| --- | --- |
| CAN | `0c8ac1ba425baded3beb47c2992954d019d8877fe5869860c3ca2b5a95bfaee9` |
| Actor | `fbce3f69fc520dd3b4f0226ccacc7bc81a5e3e5d3067b5fb8ec9d83417bde4a2` |
| Launcher | `5d69b981f244e16aacbd3b30a04bff450fb2037b98ff4ff8babfcf83c1cf2ec1` |

The new CAN executable requires the usual CAP_SYS_NICE restoration. The stage's
`ensure-can-scheduler.sh` performs that step with onsite sudo and verifies CPU 2
/ FIFO 50 without motor traffic. The operator script also checks it before
enable. This does not reload the timestamp driver or alter motor settings.
