# Runtime selection after the all-joint 500 Hz experiment

On November 11, the operator selected the previous mixed-rate SuperMimic
runtime as the default for subsequent work. Keep `supermimic_500hz` as an
experimental comparison branch; do not merge its all-joint schedule into the
default runtime.

The SuperMimic all-joint 500 Hz trial walked, but the operator observed no clear
improvement in foot dragging or outward foot placement. The operator also
reported a separate comparison with the previously walking Xbox policy:
all-joint 500 Hz brought no noticeable performance or disturbance-rejection
benefit, while the original mixed schedule initiated walking more decisively.
Those Xbox observations are operator reports; this project did not collect
that experiment's logs or establish a controlled statistical comparison.

These observations support choosing the simpler existing schedule. They do not
prove that higher command rates can never help another robot or controller.
The remaining gait investigation should compare policy/reference behavior and
measured tracking, rather than assuming a higher CAN rate will fix it.

## Selected deployment

- Source baseline: `32d6dfb996fa5b1c68c75a323127e277f0d88f1f` on `main`.
- Existing Jetson stage: `walk025-sync-en-1111`.
- Policy: unchanged 50 Hz C++/ONNX inference, 10 history frames / 993 inputs.
- Original nominal schedule: 27 motors at 50 Hz and four parallel-ankle motors
  at 500 Hz. Preserve the baseline coherent whole-action commit behavior;
  nominal rates do not describe every extra coherent-commit transmission.
- Preserve full training Kp/Kd on all 31 joints, raw previous-action history,
  fresh history at direct takeover, separate warmup, original action scale,
  pure-PD dynamic position saturation, and existing protection thresholds.
- Preserve CAN CPU 2 / SCHED_FIFO 50 and the persistent SPI performance setting.
- No controller rebuild, gain rollback, motor-zero change or model replacement
  is needed to select the retained stage.

The retained operator entry point is:

```bash
bash /home/tony/supermimic-run002-20261106/walk025-sync-en-1111/operator-test.sh
```

It checks matching validation, current scheduler/CAN state and IMU calibration,
and requires onsite `READY` before enabling. Its existing trial remains
0.25 m/s forward for 10 s, lateral/yaw zero, with the same startup/sling window,
speed ramps and final 31-motor disabled-state verification.

At selection time SSH to the Jetson timed out. The default source/deployment
choice is recorded locally; no remote pointer, service or process was changed,
and no hardware validation or actuation was performed for this selection.
The next operator launch must perform its normal current-state checks. The
retained mixed-rate validation is `20261111-140349-3122`; its local evidence
archive has SHA256
`9ac4ddd2e305b2c052e4069ff1938cd303601ee6a34b3f929bb27ae05c5443e9`.

The all-joint experiment and its failures remain reproducible on the
[experimental branch](https://github.com/BeijingDynamics/OpenSprite-SuperMimic/tree/supermimic_500hz).
Do not delete or rewrite that historical evidence.
