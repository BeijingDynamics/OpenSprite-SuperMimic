# SuperMimic previous-action contract and verification

The deployed run002 velocity actor consumes the preceding policy-step action in normalized action coordinates. This is distinct from measured joint position, a PD-limited target, motor torque, and a target-achieved flag.

## Sources and precise meaning

The original [SuperMimic paper, Appendix A, page 13](https://arxiv.org/pdf/2610.28480#page=13) lists previous action as a separate dimensionless observation and defines the joint setpoint as `q_sp = q_default + alpha * a`. Its PD discussion explains that the setpoint produces joint torque; reaching that position is not a prerequisite for the next policy step. Section 3.1, page 4, distinguishes deterministic policy actions from the noisy actions used for simulator data collection. The paper does not describe our custom deployment target-projection/history rewrite.

The official project page was inspected on 2026-11-10. No linked official training repository was used as code evidence. The following code evidence is the actual run002 implementation, with hashes checked against `provenance/original-training-source-sha256.json`, not a claim about uninspected author code.

- `scripts/train_joint.py`: collection computes `clamp(actor(history, goal) + action_noise, -20, 20)` and passes that value to `env.step`.
- `opensprite_tracker/supermimic/env.py`: `simulate(a)` sends `a` to the action manager, performs ten 2 ms physics steps, then executes `self.a_prev.copy_(a)`.
- `opensprite_tracker/supermimic/joint_interfaces.py`: proprioception concatenates actual `q`, `qd`, gyro, gravity, and `a_prev` as separate fields.
- `opensprite_tracker/supermimic/joint_learner.py`: imagined transitions use the same action for world-model stepping and the next observation's previous-action term.
- `scripts/evaluate_joint.py`: deterministic evaluation adds no exploration noise. `scripts/export_joint.py` includes the `[-20,20]` action clamp inside ONNX. Thus deployment history uses the exported ONNX output, not the unclamped internal decoder value.

The actuator memory also contains a normalized previous target in `env.memory()`. That is a distinct privileged state component for dynamics modeling. The 99-dimensional actor proprioception takes `S[:6]` and the separate `a_prev`; the actuator-memory term must not be substituted into actor action history.

## Runtime mapping and timing

For run002, `action_scale=0.25` on all 31 joints, policy period 20 ms, PD/physics period 2 ms. At control tick t:

1. Read actual q_t, qd_t and IMU.
2. Form the newest observation with previous action a_(t-1).
3. Run ONNX once to obtain a_t, already clamped by the exported graph.
4. Compute q_sp = action_offset + action_scale * a_t. Apply the deployment startup transition and unchanged safety protection downstream.
5. Store a_t for the next observation, regardless of whether the PD target was limited. Actual next q/qd remain independent sensor feedback.

The 993-dimensional runtime input is term-major: q_rel `[0:310]`, qd `[310:620]`, action history `[620:930]`, gyro `[930:960]`, gravity `[960:990]`, command `[990:993]`. The newest previous action is `[899:930]`. The training's per-frame 99-dimensional action slice is `[68:99]`; the ONNX exporter performs the order conversion.

Startup interpolation does not justify rewriting the actor's history. The full-body candidate retains the existing measured-pose/gain ramp, then continuously follows all 31 policy targets. Downstream safety projections may change physical execution, but must not impersonate the policy's preceding action. The eight-frame legacy core retains its default history behavior.

## Numerical verification

The selected simulation artifact and deployed ONNX share SHA256 `f3529c927a44f970908d48e5694e63302d0fc838121aba2fa87c0a0863968bba`. The continuous evaluation contains 3100 steps in each of four environments.

- All 384,276 checked `obs[t].previous_action` scalars exactly equal `action[t-1]`; maximum error 0.
- Actual joint feedback alignment also has maximum error 0: `obs[t].q_rel == poststep_q[t-1] - default`.
- There are 24,370 joint transitions where raw peak torque exceeds applied peak torque. Previous action still exactly matches the original action on these transitions.
- Actual post-step joint position differs from the requested setpoint: absolute error median 0.834 degrees, P95 5.253 degrees, maximum 35.297 degrees. This is evidence that action history is not an achieved-position report.

The selected artifact/source audit is reproducible with `v5_joint_run002/scripts/audit_previous_action_semantics.py`. Private local results are in `v5_joint_run002/results/outputs/action-history-semantics-20261110/audit.json`; do not publish restricted reference data with this document.

## Closed-loop diagnostic and limits on the cause claim

A new four-case 0.25 m/s diagnostic uses the same measured pre-ramp pose/history, the archived MuJoCo 3.11.0 training scene, 500 Hz direct joint PD, 3-second command ramp and 10-second plateau. It excludes real ankle transmission, CAN timing, sling forces and manual pushes. The initial pose is from the earlier 0.30 m/s run, not the manually adjusted pose after the latest test.

| Execution | Previous-action semantics | Last-5-second mean forward speed | Final waist yaw |
|---|---|---:|---:|
| Full training gains, all policy targets | Admitted target converted back to action | 0.17840 m/s | -6.46 deg |
| Full training gains, all policy targets | Original policy action | 0.17774 m/s | -6.65 deg |
| Old low upper-body gains, head/wrists held | Admitted target converted back to action | 0.62022 m/s | -19.81 deg |
| Old low upper-body gains, head/wrists held | Original policy action | 0.15792 m/s | +6.15 deg |

All four simulate motion and survive 13 seconds. Full execution changes raw versus admitted action on only 8-9 of 650 frames; old held execution changes it on every frame. This verifies that history modification can affect behavior and interacts with execution constraints. It does **not** reproduce the hardware failure to initiate walking or establish history modification as its sole cause. Full execution still has lateral velocity about 0.063 m/s in this diagnostic; no claim of perfect tracking or hardware qualification follows.

Run `v5_joint_run002/scripts/diagnose_recorded_closed_loop.py --history-audit` for the paired diagnostic. The new candidate must still pass no-enable tests, including exact action-history recurrence and a 20 ms timeout injection, before powered execution.
