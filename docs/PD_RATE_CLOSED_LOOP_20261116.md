# Existing-policy PD-rate and posture diagnostics

## Outcome

Twelve offline closed-loop trials completed 34 seconds each without the
diagnostic fall condition. Changing direct-joint PD from 500 Hz to 1000 Hz
does not reproduce or remove the hardware's narrow steps and sustained torso
roll in this plant. A fixed 20 ms target delay narrows steps and reduces speed,
but also fails to reproduce the observed hardware posture drift.

No training, hardware access, deployment changes, gain changes, action-scale
changes, or protection-threshold changes were performed. This report does not
qualify a new hardware test.

## Frozen inputs and validation

- Actor: `outputs/deploy-20261116/live-bundle10000/policy.onnx`.
- ONNX SHA256: `ec638542ea681da6fcd56a9888f3f09ef13eb92a60ab1d75b9aae9131600abf6`.
- Robot XML SHA256: `2cdfb00df9867076271d49dd699c95e2a8ec2f4c6d88be25d5c9ccc6d73fd9e1`.
- All 31 action scales: 0.25; original exported Kp/Kd and serial effort caps.
- Policy: 50 Hz; raw actor output enters action history.
- Sources: `xbox-powered-20261116-152917-5629` and
  `xbox-powered-20261116-153129-5687` in the locally archived Xbox30 evidence.
- Initial joint state, angular velocity, projected gravity and policy history
  come from frame 1000, approximately 20 s into each real trial. Each actor then
  runs in feedback with its own simulated state using the recorded next 34 s
  of commands. Recorded future actions are not imposed.
- ONNX replay at every twentieth inference frame of each real log passed
  `atol=1e-5, rtol=1e-4`; maximum raw-action difference was 1.431e-6 for both.
  Frame zero is a verified zero-action lifecycle handshake, not inference, and
  is excluded. An initial audit that included it correctly failed and was fixed
  without changing actor tolerance.
- Initial simulated local gyro and gravity are asserted equal to the recorded
  inputs. Model hashes and uniform action scale are asserted before execution.

## Controlled comparisons

Each source has six cases: 2 ms physics/500 Hz PD; 1 ms physics/500 Hz PD;
1 ms physics/1000 Hz direct PD; baseline without turn commands; baseline with
10 ms target delay; baseline with 20 ms target delay. Ankle PD remains 500 Hz.
At 1 ms integration and 500 Hz PD, torque is explicitly held for the intervening
physics step. All targets remain policy-rate updates.

Baseline delay uses a paired seed and 0/2 ms target-delay hypothesis, refreshed
every 20 ms with 0.9 hold probability. It is fixed in seconds across integration
rates. This is an approximation of the training command buffer, not exact
per-joint RNG replay. Torque response lag remains disabled.

Forward samples mean command vx > 0.15 m/s, including transitions and turns.
These are not constant-command speed-tracking scores.

| Source | Configuration | Mean forward speed, m/s | Median foot-center width, mm | Torso roll median at original 40–50 s, deg |
| --- | --- | ---: | ---: | ---: |
| Run 1 | 2 ms / 500 Hz | 0.20224 | 89.48 | -0.577 |
| Run 1 | 1 ms / 500 Hz | 0.20268 | 89.52 | -0.580 |
| Run 1 | 1 ms / direct 1000 Hz | 0.20206 | 89.51 | -0.599 |
| Run 1 | No turn command | 0.19762 | 89.80 | -0.668 |
| Run 1 | 10 ms target delay | 0.20078 | 85.95 | -0.630 |
| Run 1 | 20 ms target delay | 0.18930 | 80.07 | -0.653 |
| Run 2 | 2 ms / 500 Hz | 0.19176 | 89.79 | -0.632 |
| Run 2 | 1 ms / 500 Hz | 0.19226 | 90.06 | -0.624 |
| Run 2 | 1 ms / direct 1000 Hz | 0.19182 | 89.70 | -0.630 |
| Run 2 | No turn command | 0.18914 | 90.27 | -0.699 |
| Run 2 | 10 ms target delay | 0.19094 | 86.97 | -0.651 |
| Run 2 | 20 ms target delay | 0.18064 | 82.23 | -0.815 |

The real reconstructed torso roll medians over the same 40–50 s windows are
-4.17 and -6.19 degrees. The simulated foot boxes do not overlap; baseline
minimum separating-axis margins during forward samples are 33.3 and 30.2 mm,
versus modeled overlaps in the real-feedback reconstruction. A 20 ms target
delay reduces baseline speed by 6.4%/5.8% and median width by 9.4/7.6 mm.
It does not reproduce the complete hardware failure pattern.

## IMU consistency check

From original time 20 s, integrate the already bias-corrected body gyro using
the recorded event intervals and the fused quaternion as initial orientation.
Compare its gravity direction to the recorded complementary-filter gravity.
The CSV writer was checked: `wx_body/wy_body/wz_body` already have startup bias
removed. No second bias subtraction is applied.

| Quantity | Run 1 | Run 2 |
| --- | ---: | ---: |
| Gravity-direction disagreement, p95 | 0.97 deg | 5.56 deg |
| Gravity-direction disagreement, final | 0.31 deg | 3.19 deg |
| Late fused pelvis roll, median | -1.15 deg | -0.57 deg |
| Late gyro-only pelvis roll, median | -0.82 deg | -1.47 deg |
| Acceleration norm p05–p95 | 8.85–11.58 m/s² | 7.88–15.72 m/s² |

This is a consistency comparison, not an accuracy ranking: gyro-only integration
can drift, and accelerometer correction can be affected by motion. The estimator
uses a magnitude-dependent accelerometer trust and does not learn gyro bias
online. Neither fused nor integrated attitude is independent ground truth.
The waist encoder also records drift, so the upper-body issue cannot be treated
as merely a display-angle artifact. Sensor processing remains a separate
candidate contribution and has not been modified.

## Interpretation boundary and next work

This CPU scene uses the exact robot XML, flat ground and explicit serial PD.
It is not a bitwise reproduction of the training mjlab/MuJoCo Warp scene or a
complete hardware emulator. It omits physical parallel transmission, motor-space
protection/quantization, measured per-motor feedback age, delayed actor inputs,
and sensor fusion. Root height is inferred from the lower sole; global linear
velocity is unavailable and starts at zero. There is no sling force. Changes
to contact solver/backend can affect absolute trajectories. The near-identical
rate results are evidence within this controlled plant, not universal proof
that internal PD frequency never matters.

Before retraining, the highest-value extension is the actual deployed
motor-space mapping and bounded PD execution with recorded feedback and action
latency, followed by sensor-path ablations. Compare loaded hip-roll tracking
errors and waist-roll target/feedback separately. Do not transplant the older
`SimulationDriveBank` unchanged: it contains historical envelope and damping
assumptions that are not the current qualified runtime. Learned-world-model
versus simulator prediction/gradient auditing is also still outstanding; these
trials exercise the exported policy in physics and do not audit that model.

## Reproduction and saved evidence

Scripts:

- `scripts/evaluate_recorded_pd_rates.py`
- `scripts/summarize_recorded_pd_rates.py`

Output directories under `outputs/deploy-20261116/`:

- `pd-rate-diagnostic-v2`: six rate trials and actor-parity evidence.
- `pd-rate-no-turn`: two command ablations.
- `pd-rate-delay10` and `pd-rate-delay20`: four fixed-delay trials.
- `pd-rate-analysis`: matched-window summary, IMU arrays and
  `rate_and_posture.png`.

For example:

```powershell
python scripts/evaluate_recorded_pd_rates.py --output outputs/deploy-20261116/new-rate-run
python scripts/evaluate_recorded_pd_rates.py --output outputs/deploy-20261116/new-delay-run --profiles pd500_dt2 --delay-ms 20
```

The runner refuses an existing output directory. Saved trajectories include
commands, simulated qpos, joint q/qd, policy targets and geometry metrics.
All original real logs and deployed artifacts remain unchanged.
