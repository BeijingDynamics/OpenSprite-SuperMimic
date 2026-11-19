# Powered sensor-to-command latency audit

## Scope

Joined the two November 16 Xbox30 powered runs by action sequence:
`xbox-powered-20261116-152917-5629` and
`xbox-powered-20261116-153129-5687`. Each contains 2700 formal actions and
83,700 first motor transmissions (31 per action). The initial zero-action
lifecycle handshake is excluded. This investigation accessed local files only.

Inputs: `native.bin.ipc.csv`, `can.json.timing.csv`,
`action-motor-latency.csv`, `native.bin.inputs`, and `imu.csv`.
The motor latency CSV is produced by the previously passed `action_tx_audit.py`
from `can.json.tx.csv`, including software TX and hardware TX completion evidence.

Script: `scripts/audit_live_sensor_to_tx_latency.py`.
Results: `outputs/deploy-20261116/sensor-to-tx-latency/` contains joined per-frame
and per-motor CSVs, full statistics, old AMP statistics, and `sensor_to_tx.png`.

## Timestamp meanings

- Joint source: oldest motor's kernel **software RX** timestamp, conservatively
  converted from realtime to host monotonic time. It is not an encoder sampling
  timestamp. Sensor conversion, motor response, and adapter RX transport before
  kernel receipt are not measured by this origin.
- IMU source: DRDY event timestamp of the sample actually used by the actor.
  Matched using all three gyro and projected-gravity values in the recorded
  actor input, within its valid time window. All 5400 formal frames matched
  uniquely; none were missing or ambiguous. This excludes intrinsic sensor and
  filter group delay. Actor receive precedes its IMU snapshot by a few operations,
  so its age is an actor-entry proxy, slightly earlier than the exact snapshot.
- Driver TX: software transmit timestamp converted to a bounded host interval.
- Physical TX: host interval from send-call entry to receipt of hardware TX
  completion. The receipt is an **upper bound**, not exact on-wire time.
- Hardware adapter raw clocks are not synchronized to the host or across buses;
  they are never subtracted from host timestamps.
- No event proves the motor's internal setpoint latch or torque response time.
  Thus this is sensor-to-transmission evidence, not full sensor-to-mechanical-
  response measurement.

Each frame checks sequence identity, source/target timestamps, 31 unique motors,
monotonic stage ordering, and exact equality between the sum of stage durations
and its oldest-feedback-to-final-driver upper bound. Per-stage percentile sums
must not be presented as end-to-end percentiles.

## Stage results

Milliseconds, all formal frames. Paired entries are run 1 / run 2.

| Stage | Median | P99 | Maximum |
| --- | ---: | ---: | ---: |
| Oldest joint RX to state publication | 5.815 / 5.816 | 5.853 / 5.864 | 6.045 / 5.906 |
| State IPC to actor | 0.038 / 0.040 | 0.065 / 0.072 | 0.089 / 0.271 |
| Actor preprocessing | 0.093 / 0.094 | 0.131 / 0.130 | 0.277 / 0.379 |
| ONNX inference | 3.010 / 3.034 | 6.859 / 6.883 | 8.191 / 7.802 |
| Actor postprocessing/protection | 0.026 / 0.028 | 0.065 / 0.064 | 0.123 / 0.158 |
| Target ready to all driver TX, upper | 0.800 / 0.763 | 2.372 / 2.419 | 2.477 / 2.509 |
| Oldest joint RX to target ready | 8.998 / 9.023 | 12.826 / 12.856 | 14.138 / 13.873 |
| Oldest joint RX to all driver TX, upper | 9.734 / 9.736 | 13.568 / 13.572 | 15.537 / 15.558 |
| Oldest joint RX to all TX receipts, upper | 10.419 / 10.417 | 14.374 / 14.382 | 16.350 / 16.355 |

The oldest motor is left hip pitch in 2570/2700 and 2599/2700 actions;
right hip pitch accounts for the remainder. The approximately 5.8 ms age
precedes inference and belongs to feedback acquisition/publication scheduling;
do not label it entirely as wire-transfer time or actor compute.

## IMU path

| Stage | Median, run 1 / run 2 | P99 | Maximum |
| --- | ---: | ---: | ---: |
| Consumed DRDY to actor entry | 3.355 / 3.279 | 6.145 / 6.205 | 8.293 / 6.976 |
| Consumed DRDY to target ready | 6.987 / 6.775 | 11.990 / 11.462 | 13.250 / 13.360 |
| Consumed DRDY to all driver TX, upper | 7.918 / 7.636 | 13.255 / 12.934 | 14.433 / 14.922 |
| Consumed DRDY to all TX receipts, upper | 8.637 / 8.353 | 14.101 / 13.738 | 15.276 / 15.734 |

The 800 Hz producer/fusion does not imply a <=1.25 ms age at the actor.
The existing receiver polls for the latest packet every 5 ms. Capture, IPC
polling and the 50 Hz actor are separate schedules. This receiver behavior is
shared with the old AMP runtime.

## Joint groups and update jitter

For first transmission of a new action, direct-motor target-to-driver medians
are 0.715/0.674 ms; ankle medians are 0.656/0.613 ms. They do not show a separate
20 ms wait for hips/knees or a one-action delay between ankles and legs.
The existing TX audit verifies coherent action groups and no stale sequence
sent after the next group. Ankle 500 Hz refreshes between these first sends
recompute external PD from the held policy target; they are not new policy
actions and are not counted as 500 Hz actor updates.

Left hip pitch new-action driver-send intervals have median 20.000 ms in both
runs, but maxima 25.873 and 25.882 ms, with minima 15.981 and 14.011 ms.
Inference variation and dispatch boundaries create update jitter even while
the per-action oldest-feedback deadline remains below 20 ms. Motor-local PD
continues between direct-joint commands.

## Comparison with archived AMP/Xbox

Read six October 19 powered actor timing logs from
`v5_joint_run002/results/outputs/walk025-direct-20261110/xbox-timing-reference/`.
Their manifests bind the old actor and G74 model.

- `compute_ms` median: 0.256–0.278 ms across runs.
- P99: 0.443–0.522 ms; maxima: 1.014–2.236 ms.
- This field includes observation/policy/protection/send work, rather than
  isolating ONNX inference. Current SuperMimic ONNX alone is approximately
  11–12 times that old full-compute median; this is a historical comparison,
  not a controlled benchmark with identical runtime build or CPU workload.
- Old consumed IMU age just before IPC send: median 3.56–3.61 ms;
  current target-ready IMU age median 6.77–6.99 ms. These nearby endpoints
  support a roughly 3.2–3.4 ms typical increase before controller dispatch.
- Old `source_age_ms` starts at state publication, unlike the new oldest-motor
  RX timestamp. Do not compare those columns as if they had the same origin.
- The retained AMP logs do not contain matching per-action hardware TX evidence.
  A numerical claim about its complete sensor-to-motor delay is unsupported.

## Posture interpretation and next diagnostic

Run 2's late 40–50 s interval has inference median 2.998 ms, P99 6.732 ms,
and maximum 6.907 ms. Its joint-source-to-all-TX-receipts upper bound has median
10.416 ms and maximum 14.388 ms. There is no progressive late latency increase
matching the accumulated torso lean. Delay may reduce control margin, but
these logs do not establish it as the sole source of drift.

The previous fixed 10/20 ms **target-delay** experiment is not equivalent to
this measured chain: actual delay includes asynchronous sensor ages, actor
computation, per-action jitter and dispatch. A better next closed-loop model
should delay actor observations and command application separately, replay
the measured temporal distribution, and retain fast local direct-joint PD and
500 Hz external ankle PD. Optimize inference tails and acquisition/IPC timing
only after comparing these effects; keep all protections unchanged.

Both original runs recorded all 31 motors disabled. This audit made no hardware
transmission, changed no parameters, and did not start another test.
