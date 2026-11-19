# R1 right-knee guard during a backward pull

The operator reported pulling the robot backward in session
`xbox-powered-20261119-143531-4916`. The control started at 14:35:48,
the Xbox window opened at 14:36:08, and shutdown completed at 14:36:56
(Asia/Shanghai). The right-knee dynamic guard ended the session. All 31 motors
were individually confirmed disabled and the CPU-frequency lease was restored.

The small completed CAN report was copied locally as
`outputs/deploy-sm-walk-r1-xbox120-20261119/143531-can.json`.
A later operator-controlled session was active during diagnosis, so no bulk
archive, rebuild, deployment, or additional hardware test was started.

## Recorded evidence

| Right-knee quantity | Recorded value / configured limit |
| --- | --- |
| Maximum absolute feedback torque | 28.000 Nm |
| Feedback guard | 27.993162 Nm |
| Feedback decoding range | -28 to +28 Nm, 12 bits |
| Configured mechanical peak | 40 Nm |
| PD command limit | 36 Nm, 90% of configured peak |
| Maximum estimated command torque | 35.970087 Nm |
| Maximum unsaturated command torque | 38.307754 Nm |
| Maximum measured motor speed | 5.504274 rad/s; limit 9.3 rad/s |
| Maximum 1-second feedback RMS torque | 8.673568 Nm |
| Maximum 10-second feedback RMS torque | 4.389367 Nm |
| Maximum raw MOS / rotor temperatures | 31 / 30 C |
| Deadline stage | Empty; no deadline error reported |

The command maximum and feedback maximum are whole-session statistics; they
must not be presented as simultaneous samples.

`set_peak_feedback_caps` in the deployed commissioning launcher computes:

```text
min(0.90 * mechanical_peak, feedback_range_max - feedback_step / 2)
= min(36, 28 - 56 / (2 * 4095))
= 27.993162393162393 Nm
```

This intentionally rejects an extreme feedback code when the feedback range
is smaller than the configured mechanical peak. The 0.00684 Nm difference
from 28 is a quantization boundary, not a meaningful mechanical overload margin.
The direct-drive PD command saturation can allow 36 Nm while this feedback
guard rejects a full-scale 28 Nm report. External loading and movement between
target updates can also differ from the host-side PD estimate.

The observed torque peak satisfies the feedback guard condition and is the
leading explanation. Speed and temperature statistics do not indicate their
limits were exceeded. The native exception combines position, speed and torque
into one message and does not save the offending packet's position/velocity/
torque tuple. A simultaneous position violation cannot be definitively excluded
from the summary alone. The error message should eventually identify the exact
predicate and values without changing protection behavior.

## Interpretation and next checks

This is strong evidence of the feedback full-scale protection limiting the
disturbance test. It is not proof that physical motor torque exceeded 40 Nm,
nor that the policy could or could not recover from the disturbance without
shutdown. When a feedback value saturates, its true magnitude above the range
is unknown. Raising the decoded guard to 36 Nm while leaving decoding at
plus/minus 28 would make that guard unreachable, not verify safe 36 Nm feedback.

Before changing this behavior, verify the actual firmware feedback torque
range and scaling against the deployed decoder. If a larger genuine range is
available, update the protocol mapping and validate it separately. If it is not,
the treatment of momentary full-scale feedback requires an explicit protective
design decision with retained sustained-load, temperature, speed, position and
deadline protections. No such change was made in this diagnosis.
