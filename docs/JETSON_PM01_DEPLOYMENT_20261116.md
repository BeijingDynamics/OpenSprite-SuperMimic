# PM01 finalist deployment preparation: November 16, 2026

The new models are staged at `/home/tony/supermimic-pm01-1116` on Jetson
192.168.0.253. The existing powered runtime and operator entry points remain
unchanged. This deployment is qualified for file-only CPU replay, not powered
operation. No CAN or IMU device was opened and no motor command was sent.

## Candidates and export

- Primary: seed45, quarter actor learning rate, update 10000.
  Checkpoint SHA256 `aab860df068d5ccc67aa94ea315f5517dd333373c0e7be52fe6344bb5a98ee8a`.
- Comparison/fallback: the same run at update 9000.
  Checkpoint SHA256 `50a6aa050b9b3f6c78fff4e0dcd60e5cbc87a3cfb5912a6a0f6517c49e1d9492`.

Both were exported on the local Windows computer without accessing L40S.
`scripts/export_backed_up_policy.py` uses the exact controller/wrapper class
definitions, checkpoint state dictionaries and recorded-input parity. The
asset/control identity, joint order, full gains, effort caps, defaults and
training revision are checked against the prior validated export contract.
Local export uses an isolated CPU environment; it does not change Jetson ORT.
Each model passed 512 recorded-input comparisons. Maximum ONNX action errors
were 2.15e-6 and 2.63e-6 for updates 10000 and 9000, respectively.

The interface is 993 inputs, 31 outputs, ten chronological observation frames,
50 Hz policy, action scale 0.25. The trained IMU frame is pelvis/root local;
gyro is rad/s and gravity is a unit projected vector. Previous action means
raw clipped policy output before motor-target protection. Direct takeover
requires a new history and zero gain/target ramp duration; this replay does
not implement or authorize a new live startup sequence.

## Native validation

The existing mixed-rate deployment's file-only C++ replay binary was copied
into the isolated `bin` directory, with a local link to installed ORT 1.23.2.
Its SHA256 is saved in `native-binary.sha256`. No CAN binary or capability was
changed. The Python replay validator now explicitly recognizes v2 semantics;
negative checks reject wrong history/frame/ramp fields and hardware admission.

| Candidate | Cases | Frames | Maximum inference | Maximum finish lateness | 20 ms deadline misses |
| --- | ---: | ---: | ---: | ---: | ---: |
| 10000 | 5 | 5000 | 13.481 ms | 13.562 ms | 0 |
| 9000 | 2 | 2000 | 13.762 ms | 13.874 ms | 0 |

Primary cases: zero, forward 0.25/0.35 m/s, left/right curves. Fallback cases:
zero and forward 0.25 m/s. All native/Python recurrence comparisons passed;
all comparisons with recorded simulation actions passed. Each native case
uses 1000 frames, 50 Hz pacing, one ORT thread and 50 warmup calls; replay
resets state after warmup. These timing measurements exclude live CAN/IMU/IPC
load and do not establish the complete feedback-to-motor 20 ms budget.

Two preparation failures were resolved before passing runs: Windows shell
line endings and the old binary's relative shared-library lookup. Failed logs
are retained. The successful run is `cpu-validation-v2.log`, with reports in
`replay-10000-v2` and `replay-9000-v2`. Deployed package files were hash-checked
with no mismatches, and successful evidence was copied back locally.

## Next integration gate

Use the previously selected direct-joint 50 Hz / parallel-joint 500 Hz runtime.
Map this contract into live pelvis-frame IMU observations and raw-action
history, preserve full training gains and existing motor-space protections,
then qualify no-enable feedback/IPC freshness and complete control timing.
Only a separately authorized powered session may enable motors; its shutdown
must verify all 31 motors disabled. This file-only result grants no such admission.

Local models and evidence: `outputs/deploy-20261116/`, including the final
deployment archive, per-model contracts/export parity and `jetson-evidence/`.
