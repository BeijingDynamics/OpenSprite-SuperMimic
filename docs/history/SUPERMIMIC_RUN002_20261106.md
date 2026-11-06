# Run002 model, offline runtime package and remaining hardware gate

The later [Jetson CPU validation](SUPERMIMIC_RUN002_JETSON_20261106.md) passed
on Nov6 after the user powered253. The rented-host results below remain a
separate stage; motor admission is still blocked.

Training completed on2026-11-06 at03:25:58 China time:15,398 updates,
43,200.55seconds, fresh random initialization. The final model won the
post-training comparison with the retained earlier candidates. No run001
checkpoint initialization or training-reward changes were used during run002.

| Identity | Value |
|---|---|
| Training run | `joint-waist-v5-002` |
| Upstream revision | `05774a9a37f47ed8a66f82ed36324e7f712bdfcb` |
| Asset | `sprite0825_v5_4340_shoulders_waist20261105` |
| Checkpoint SHA256 | `094b52267159d26f4d5bfb41edc5823a9becc32f706c87df05f8aa36b4016282` |
| ONNX SHA256 | `f3529c927a44f970908d48e5694e63302d0fc838121aba2fa87c0a0863968bba` |
| Published control SHA256 | `184ec989fd24b4bc6ea5034e6fdae796b5b9ce83ef2c884dfe4fc61d8d6987dc` |
| Training PD / policy |500Hz /50Hz,10substeps |
| Runtime interface |10chronological frames, term-major993inputs,31actions |
| Motor / hardware admission |Blocked |

The published gains, torque caps and action defaults were adopted together.
The source200Hz snapshot was preserved unchanged; the effective experiment
explicitly overrides timing to500Hz. The new waist mass is3.7391kg; total
model mass is16.319556kg. Actual GPU actuator gains, individual and aggregate
torque limits, and20ms outer stepping were checked before training. TF32 was
disabled throughout. The effective experiment/source snapshot is stored with
the local simulation release, independently of this source repository.

## Simulation results

20command cases across seeds101/202/303 completed60/60trials of30seconds
without a fall or reset. These use deterministic serial simulation without
domain randomization; changing seeds does not establish hardware robustness.
The four continuous62-second speed/turn/reverse/restart sequences also survive.
Motion tracking survives15/15source-clip trials; each clip is7.98seconds.
Earlier chat reports incorrectly called the motion trials30seconds. Their
JSON metadata always recorded the actual7.98seconds; only velocity trials are30s.

Examples below use seed101 and the last5seconds:

| Command | Measured |
|---|---|
| vx0.15m/s |vx0.104m/s,vy0.037m/s |
| vx0.30m/s |vx0.208m/s,vy0.028m/s |
| vx0.45m/s |vx0.451m/s |
| vx0.60m/s |vx0.560m/s |
| vx-0.30m/s |vx-0.220m/s |
| vx0.30,yaw+0.15rad/s |vx0.249,yaw+0.167 |
| vx0.30,yaw-0.15rad/s |vx0.223,yaw-0.135 |

Low-speed initiation recovered after the3869update candidate had mostly
stood still at0.15m/s. Continuous stop/restart also recovers motion. Velocity
undershoot and lateral drift remain, especially slow right turns. Survival,
reward, and an arbitrary2cm foot-lift heuristic do not qualify the gait.

## Offline runtime results

Tests run on the rented x86_64Linux host, ORTCPU1.31.1. Jetson253 remained off;
no new target timing or physical sensor/CAN measurement is claimed.

- 297 Python tests and 33 subtests pass; both native core tests pass.
- 40 native replay cases cover 20 commands with handoff disabled/enabled,
  totaling60,000frames. C++ and independently assembled Python observations,
  actions and targets match; recorded simulator comparisons remain within1e-4.
- 14 malformed/native-profile checks and 8 real-bundle tamper/admission checks pass.
- 30 seconds of recorded-state real-time IPC completes 1500 frames with zero
  reverse state-socket bytes, zero motor TX and zero physical-device access.
  Four stale/order/disconnect/IMU-freeze faults fail closed.

`open_sprite_runtime.supermimic_run002` creates and validates a version-bound
offline package. It pins the model/URDF/profile identities and verifies all31
named gains, effort limits, defaults, action scales and velocity-cost fields.
Rehashing a changed manifest does not authorize changed gains or model inputs.
Hardware admission remains blocked by the existing contract validator.

## Protected drive simulation

Seven commands x30seconds pass each of serial500HzPD, exact calibrated
transmission, torque/speed-envelope saturation,10msdelay with multirate
control, and the additional2msfeedback/5msresponse-lag case. These retain
the measured linear ankle/head mapping and the documented motor assumptions.

The stricter reject-on-envelope-excess mode stops at1.30seconds, BEFORE the
2second command onset, for the same left-hip-pitch startup event in all cases.
Requested modeled motor torque is-20.4948Nm against a15.1129Nm envelope at
-3.6086rad/s. This is one common startup gate, not seven independent gait
failures. No motor limit was raised to obtain a pass.

The modeled torque envelope uses conservative nameplate/speed proxies and a
MIT feedforward-field-range proxy; it is not a measured total motor-torque
curve. MIT field range, serial training cap and physical peak torque are
different quantities. Some speed limits are proxies because measured values
are missing. Published serial Kd5 also differs from the qualified motor-side
software ceiling3. A serial torque cap, particularly wrist0.03Nm, cannot be
assumed to emerge from embedded motor PD merely by copying Kp/Kd.
The exported `velocity_limit` vector here contains training reward thresholds,
not measured hard motor-speed limits. Actual limits need the separate hardware
contract. These distinctions remain explicit hardware-admission gaps.

Protection diagnostics now freeze at the first trip. A regression compares
5second and30second trials and confirms identical pre-trip statistics;
passive post-trip falling cannot inflate the admitted controller's statistics.
Post-trip actuation stays zero. Old reports lacking this scope must not use
their full-run motor extrema as pre-trip operating measurements.

## Commands after253 is powered for CPU-only verification

Use a NEW development directory and the provided source/package, retaining
the deployed runtime. Reuse the previously validated CPUORT environment and
matching headers; the existing target ORT was1.23.2. Set the paths to that
environment explicitly; do not upgrade the deployed environment implicitly.

```bash
export SPRITE_REPLAY_PYTHON=/path/to/existing/venv/bin/python
export SPRITE_ORT_INCLUDE=/path/to/headers/matching/installed/ort
export PYTHONPATH="$PWD/src"
bash tools/build_supermimic_replay.sh

"$SPRITE_REPLAY_PYTHON" -m open_sprite_runtime.supermimic_run002 verify \
  --bundle /path/to/run002/bundle

bash tools/run_supermimic_run002_offline.sh \
  /path/to/run002/bundle /path/to/rollout.npz /path/to/summary.json \
  reports/run002-native

"$SPRITE_REPLAY_PYTHON" tools/benchmark_supermimic_jetson.py \
  --contract /path/to/run002/bundle/contract.json \
  --rollout /path/to/rollout.npz --summary /path/to/summary.json \
  --binary build/supermimic_offline/sprite_supermimic_replay \
  --output-dir reports/run002-cpu-timing --seconds 60 --threads 1 --cpus 5
```

The CPU-only package has no motor-enable or CAN
entry. Real powered walking remains blocked pending the startup envelope,
actuator realization, calibration and target timing qualification.

Local experiment evidence: `E:/supermimic_run/v5_joint_run002/results`.
The release contains the custom reference bank, source/config hashes,
checkpoint, ONNX, reports and videos; generic datasets, dependency caches
and public mesh copies are excluded. Local execution-session interruption
delayed post-training runtime preparation beyond the originally requested
14hourwindow. Server-side training/evaluation/export completed on schedule;
the final evidence was subsequently pulled and verified locally.
