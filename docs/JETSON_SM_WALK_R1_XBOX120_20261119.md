# Sprite SM-Walk R1: extended Xbox profile

## Scope

The isolated deployment is `/home/tony/sprite-sm-walk-r1-xbox120-20261119`.
The original 30-second R1 deployment remains available and unchanged.
The operator requested a 120-second Xbox input window and a 0.40 m/s forward
cap after three powered attempts, reporting light foot contact and left torso lean.

| Parameter | Extended profile |
| --- | --- |
| Forward / reverse command | +0.40 / -0.05 m/s |
| Yaw command | +/-0.20 rad/s |
| Lateral command | 0 |
| Forward acceleration / braking | 0.05 / 0.10 m/s^2, unchanged |
| Initial supported zero-command phase | 20 seconds, unchanged |
| Xbox input window | 120 seconds, 6,000 policy frames |
| Maximum braking after closing the window | 4 seconds |
| Total control duration | 146 seconds |

The policy, full training gains, action scale/history, mixed-rate transmission,
CPU 2 / FIFO 50 CAN scheduling, CPU 4/5 frequency lease and protective thresholds
are unchanged. The duration exception is restricted to the exact 146-second
Xbox profile with its new acknowledgement. Other modes retain their prior limits.
LT must be released and pressed again after admission; pre-held LT cannot start motion.
LT release commands braking, not motor disable.

`scripts/prepare_r1_xbox120_040.py` derives the isolated sources from the archived
validated R1 source. `scripts/build_r1_xbox120_on_253.py` builds and checks them.
The native tests cover the 6,000-frame window, cap, release/disconnect braking,
stale inputs, rejection of mismatched duration and rejection of the old acknowledgement.
All 10 native tests and shell/Python syntax checks passed. Full no-enable and
late-window deadline validation must pass before the operator entry is qualified.
The operator entry checks matching evidence and file hashes before allowing READY.

## Powered source evidence and lean investigation

The original source and four session directories are saved locally in
`outputs/deploy-sm-walk-r1-xbox120-20261119/powered-source-evidence.tar.gz`.
SHA-256: `877ab60b5a4242175d4f84c578becfb02e5ac3439443b06d8785a02c9ff220aa`.

Two complete trials passed. A third powered attempt ended with a settling-gate
failure. All three powered attempts recorded 31 disable confirmations. The fourth
directory is a failed stationary IMU calibration before motor enable; it must not
be described as having 31 disable readbacks.

`scripts/diagnose_r1_torso_roll.py` reconstructs torso orientation using recorded
pelvis projected gravity, actual joint feedback, and the exact R1 MJCF asset.
Positive lateral lean means the torso-up axis points toward pelvis +Y (left),
which differs from the signed Euler roll convention. Global yaw is set to zero.
Results and traces are under the deployment output's `roll-analysis` directory.

In the two successful sessions, commanded-straight samples have torso mean left
lean of approximately 0.47 and 0.71 degrees, but their 95th percentiles reach 6.85
and 7.30 degrees. Averaging standing and initial acceleration with walking can hide
the visible late-session lean. The first successful session has sustained positive
torso lean late in its forward interval even though yaw command is exactly zero.
Thus the observation cannot be dismissed as a commanded turn.

Restricting to full 0.25 m/s commands with near-zero yaw yields 7.80 and 4.36
seconds in those two sessions. Mean torso left lean is 0.80 and 2.90 degrees;
the corresponding 95th percentiles are 7.16 and 8.59 degrees. These short,
different segments are not matched trials and include gait onset/transients.

Both measured pelvis orientation and waist roll contribute. Measured waist roll
also differs from policy target during motion. This is an observation, not an
identified cause: pre-protection targets, transmitted targets, joint tracking,
and left/right leg loading still need to be separated before assigning a cause.
No IMU bias, joint-zero, gain or policy compensation is applied in this profile.
The logs do not measure sling load, foot forces or global velocity. Target FK on
the measured pelvis is not a forecast of future body orientation.

## Qualification status

The first long disabled run exposed the old per-socket 200,000-entry TX evidence
capacity. It stopped on capacity exhaustion; subsequent timestamp bookkeeping
also reported unmatched IDs. Final readback confirmed all 31 motors disabled,
zero enable attempts, and CPU-frequency restoration. This failed run is retained
as `failed-capacity-disabled-01`; it does not qualify the operator script.

The fixed build preallocates 600,000 evidence entries per socket before control.
Its regression test records and timestamps 500,000 events without reallocating.
Overflow rejection and timestamp matching remain enforced. The end notice now
derives from the window-close frame plus maximum braking time (145 seconds),
instead of the previous hard-coded 53-second notice. All 10 tests passed again.
Full live no-enable validation and deadline injection are pending. No powered
session is launched by deployment or these validation scripts.

The corrected full 146-second disabled run passed, including 31 motor disable
readbacks and CPU-frequency restoration. Maximum reported inference was 9.004 ms;
maximum oldest-feedback-to-target timing was 16.255 ms.

The first scheduled fault-injection attempt instead encountered a real timing
tail before injection, at sequence 1234 (about 24.6 seconds). Inference took
15.443 ms; feedback-to-inference-completion was 21.325 ms. CAN and actor both
rejected the expired cycle under the unchanged 20 ms protection. CAN maximum
wakeup lateness was only 0.038 ms. All 31 motors were confirmed disabled, with
zero enable attempts, and CPU-frequency settings restored. This is retained as
`failed-natural-tail-01`, not counted as a successful planned injection. A
same-build injection repeat was then scheduled. The cause of the inference tail is not
identified; successful repeats must not erase this limitation.

## Completed qualification

The same-build repeat passed the planned late-window actor pause, with the
unchanged 20 ms watchdog stopping the session. Both the complete disabled run
and planned fault run recorded zero enable attempts, all 31 motors disabled,
and restored frequency settings. The final operator `--check` passed, including
CPU 2 / FIFO 50 and exact matching model, build and evidence hashes.

The complete run dispatched 7,300 actions / 226,300 first motor sends. Median
inference was 2.836 ms, P99 3.207 ms, maximum 9.004 ms. Oldest feedback to all
TX completion receipts was median 10.378 ms, P99 10.504 ms, maximum 19.534 ms.
Left hip new-action hardware interval reached 26.068 ms. These measurements do
not eliminate the separately recorded real timing failure.

The fixture header, default pose, action offset, action scale and every fixture
state match the old profile. Only its frame count grows from 2,701 to 7,301.
Motor tables, joint safety tables, feedback transform and target protection
configuration hashes match the original R1 validation.

Validation archive SHA-256:
`24e0f12e4360f09b1ffb65ef979313e23059337672c8618ad74cd9689aa0830f`.
The archive includes both failed attempts as well as the two passing runs.
The qualified operator command is:

```bash
bash /home/tony/sprite-sm-walk-r1-xbox120-20261119/operator-xbox.sh
```

This is an operator-launched, bounded powered test with READY confirmation and
retained fall protection. It is not evidence that 120-second unsupported walking
or 0.40 m/s physical tracking has already succeeded. Runtime guards can end it early.
