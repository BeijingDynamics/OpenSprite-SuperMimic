# R1 faster start and feedback torque observation

The operator explicitly requested keeping waist roll unchanged, faster command
acceleration, and no shutdown merely for an instantaneous torque excursion when
the drive has not reported a fault. This authorizes a specific change to the
previous torque guard, rather than weakening a guard to make an unexplained
validation failure pass. The original deployment remains intact.

New stage: `/home/tony/sprite-sm-walk-r1-fast-20261119`.

## Exact changes

- Forward command acceleration: 0.05 to 0.10 m/s^2; nominal time from zero to
  0.40 m/s is four seconds. Braking stays 0.10 m/s^2; yaw limits and acceleration,
  reverse cap, lateral zero and 120-second input window are unchanged.
- The explicitly acknowledged extended Xbox profile records instantaneous
  feedback torque excursions instead of stopping solely on that predicate.
  Positive and negative full-scale feedback remain visible as separate counters.
  The first excursion records position, velocity and decoded torque.
- Motor status/identity errors, position, speed, raw and filtered temperature,
  feedback validity and 20 ms timing guards remain active. PD command saturation
  and full training gains, including waist roll Kp 60 / Kd 3, are unchanged.
- Other native profiles retain the instantaneous feedback torque stop. The new
  profile uses a distinct acknowledgement and reports its observation mode.
- Dynamic guard exceptions now identify the predicate and measured values.

Torque RMS over one and ten seconds was already telemetry only. It remains
telemetry; there is no new sustained-load stop criterion in this change. An early
conversation description calling it an existing sustained-load protection was
corrected after inspecting the source.

The 27.993 Nm boundary is retained for counting right-knee excursions. It is not
raised beyond the plus/minus 28 Nm decoded range. A full-scale value cannot reveal
the actual torque beyond that range, and absence of a drive fault is not proof
of mechanical torque margin. This deployment does not claim otherwise.

## Evidence and scope

`scripts/prepare_r1_fast_start.py` creates the isolated source from the validated
120-second deployment. `scripts/test_r1_feedback_observation.inc` feeds the
production CAN feedback decoder through Unix sockets without a CAN interface.
Positive and negative full-scale feedback are recorded; fault status, position,
speed and temperature still reject. The old behavior is also tested with the
observation flag off. All 10 native tests and script syntax checks passed.

`scripts/audit_r1_fast_offline.py` checked 17,972 recorded policy states from three
powered sessions. No direct-drive position or velocity violation appeared in
those published states. The CAN-rejected packet from the knee trip is not in
policy history, so this is not a complete raw torque replay. Parallel ankles
are excluded from the linear state reconstruction. The native decoder tests
separately cover the reported full-scale torque condition.

At initial preparation, the actor source differed only in command acceleration. Model, observation
history, PD calculation and saturation code are unchanged. Recorded old actions
cannot establish the closed-loop response to faster command acceleration.

At preparation time, full live no-enable integration and late-window deadline
injection are pending. Deployment never starts a powered run; the operator
entry requires matching evidence and explicit onsite READY confirmation.

## First live validation attempt

The 146-second disabled session completed, confirmed all 31 motors disabled,
and restored the CPU-frequency lease. Its post-run audit failed on sequence
5288: the last seven motor sends began more than 2.5 ms after target acceptance,
with a maximum of 2.527481 ms. That frame's inference took 9.294364 ms. Maximum
oldest-feedback-to-TX-completion-receipt age was 19.543460 ms, below 20 ms.
The complete trace is retained as `failed-send-tail-01`. No threshold or runtime
schedule was changed to make this audit pass. A single same-build repeat was
started; a passing repeat cannot establish that the latency tail is eliminated.

## Initial qualification failure

The same-build repeat also failed. The post-run audit first reported a missing
accepted action group, but its native CAN report identifies the actual cause:
the original 20 ms deadline rejected sequence 5513 before dispatch. It had been
accepted at approximately 19.763 ms after the oldest feedback. The group was
therefore intentionally not sent, rather than lost during shutdown. The native
report has 5,513 states/targets and 5,511 dispatched action groups. All 31 motors
were confirmed disabled; no enable was attempted; CPU settings were restored.
No further attempts were made to obtain a passing result by repetition.

A CPU-only shared/pinned/shared comparison used the exact R1 model and recorded
inputs, 1,000 frames per case, with the existing 1.344 GHz lease:

| Placement | Median inference | Maximum inference | Maximum action difference |
| --- | --- | --- | --- |
| Both threads allowed on CPU 4/5 | 2.674 ms | 2.961 ms | 0 |
| Main CPU 4, worker CPU 5 | 2.717 ms | 3.524 ms | 0 |
| Both threads allowed on CPU 4/5, repeat | 2.748 ms | 3.180 ms | 0 |

This did not reproduce the live tail or demonstrate that separate thread pinning
would solve it. No production affinity change was made. Recorded CPU temperature
after the failure was about 43.9 C; that single snapshot does not establish the
cause or rule out transient clock/scheduler effects during control.

At that stage, functional changes and decoder tests were complete but live
qualification was incomplete. The operator entry remained evidence-gated.
That failure initiated the live timing investigation documented below.

## Final pre-power qualification

The completed investigation is recorded in
[R1 live thread, coherent target and timestamp fixes](R1_LIVE_THREAD_LAYOUT_FIX_20261119.md).
It corrected stale direct-drive target caches at coherent commits, removed
unnecessary mapping-boundary waits, enabled ordinary CAN compiler optimization,
fixed the driver's echo registration race, and selected pinned two-thread ORT
inference with idle spinning disabled. Model, training gains, PD saturation,
position, speed, temperature and timing thresholds are unchanged.

`sleeping-disabled-01`, `sleeping-disabled-02` and `sleeping-fault-01` all passed.
The first two ran the full 146-second schedule; the third deliberately triggered
the original 20 ms timeout. All had zero enable attempts and 31 individually
confirmed disabled motors. All 2,085,580 recorded sends have software and hardware
completion timestamps. No occupied-echo-slot kernel errors recurred during these
trials. Maximum oldest-feedback-to-hardware-completion-receipt ages in the full
runs were 11.857 and 13.841 ms. These are bounded trials, not a hard realtime
guarantee or evidence of powered gait improvement.

The onsite operator entry is:

```sh
bash /home/tony/sprite-sm-walk-r1-fast-20261119/operator-xbox.sh
```

It verifies the exact evidence and binaries before the READY prompt. The Xbox
window is 120 seconds, forward cap is 0.40 m/s, and forward acceleration is
0.10 m/s^2. The operator retains fall protection and observes shutdown. Individual
disable feedback remains mandatory; missing acknowledgements require physical
power cutoff as stated by the script. No powered trial was started by deployment.

The driver is temporary. After a Jetson reboot, restore this qualified driver
and CAN/timestamps with:

```sh
sudo bash /home/tony/sprite-sm-walk-r1-fast-20261119/load-echo-order-driver.sh
```
