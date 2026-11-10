# SuperMimic direct policy handover, 2026-11-10

The new explicit `--direct-policy-start` mode removes deployment gain ramping and both position-target handoff interpolations. It requires full-body training gains. Existing experimental modes keep their previous behavior.

Inference warmup uses a separate `PolicyCore`. The first transport transaction remains a disabled, measured-position, zero-gain handshake, but it does **not** run inference or latch an action. On the next fresh state the formal core resets its history/counters, fills ten sensor-history slots from the fresh measured state, initializes previous actions to zero as at training reset, and directly maps `offset + scale * raw_action`. Every subsequent previous-action observation contains the raw preceding output, independently of downstream saturation. No head/wrist hold remains in this mode.

All 31 joints use the sealed training Kp/Kd from the first formal target. CAN additionally rejects any formal target whose gains differ from the configured full gains. Existing position limits, pure PD dynamic position saturation, motor peak/encoding bounds, feedback torque protection, velocity protection, IMU checks, 2 ms CAN deadline and 20 ms oldest-feedback deadline remain. The initial handshake safety check and final per-motor disable readback remain. Removing gain ramping does not remove the existing commanded-speed ramp or change the walking schedule.

## Scheduling is automatic and checked

The operator and stage entries run `ensure-can-scheduler.sh`, packaged from `runtime/tools/supermimic_can_preflight.sh`. A missing `CAP_SYS_NICE` triggers `sudo setcap` automatically; sudo may request the onsite user's password, but no separate manual command is needed. The helper reads the launcher's CPU profile rather than duplicating a core number. Missing permissions or failed scheduler setup stop the entry. Commissioning reads back the actual CAN process affinity, scheduling policy and priority before the actor connects and saves `scheduler-actual.json`.

The present verified partition is CAN CPU 2 / SCHED_FIFO 50, actor CPUs 4/5, ICM CPU 3. Old Xbox used CAN CPU 5 and actor CPU 4; old integration found ordinary Linux scheduling insufficient even when pinned. SuperMimic CPU replay separately compared one actor thread on CPU 5 with two on CPUs 4/5; the latter had lower P99 latency. This is evidence for the current division, not proof that CPU 2 is intrinsically better for CAN. CPU allocation is unchanged in this candidate to isolate startup behavior.

## Verification

- Eight native tests passed, including core reset, immediate full-body gains and existing motor protections; six cache tests passed, including rejection of evidence from the other startup mode.
- Archived 2476-state audit: one non-policy handshake, 2475 formal actions; warmup 0 versus 100 produced bit-identical records. No target interpolation; fresh history and raw action history verified. Native mapped/quantized motor checks rejected zero frames and recorded 661 saturation events. These are frozen-state checks, not a prediction of physical motion.
- Jetson disabled session `20261110-180022-6540`: all 2476 targets accepted, CAN wake maximum 0.091984 ms, maximum oldest-feedback-to-target 14.840137 ms, no deadline misses, zero enables, 31/31 disabled.
- Same-build intentional actor pause: original 20 ms deadline rejected the pending target after 1804 recorded targets; zero enables and 31/31 disabled. The expected fault is preserved in CAN logs; the fault-validation report passes because it observed that rejection.
- Log-copy negative tests reject a handshake action, inherited action history, gain ramp and target interpolation. Standing-evidence negative tests and file-only cache selection passed.
- Actual scheduler readback in both live disabled runs: affinity `{2}`, SCHED_FIFO, priority 50. No motors enabled in this change's validation.

Isolated Jetson entry:

```bash
bash /home/tony/supermimic-run002-20261106/walk025-direct-1110/operator-test.sh
```

Default reuses matching validated dry/fault records and checks current runtime identity. It retains fresh IMU calibration and onsite READY before powered work. The schedule stays zero-speed control 20 s, 10 s supported release window, 3 s velocity rise, 0.25 m/s plateau 10 s, 3 s deceleration and 3.5 s zero tail. Full gains start immediately at formal handover, without a four-second gain ramp. Body checkpoints remain before the sling-release window and before forward motion. Termination must read back all 31 disabled; missing confirmations produce a cut-power warning, never a success claim.

`--verify-only` runs disabled validation and the deadline fault without enabling. `--check-cache-only` checks files only. Do not represent these results as powered walking qualification.

Local evidence: `v5_joint_run002/results/outputs/walk025-direct-20261110/VALIDATION.json` and `validation-evidence-180022.tar.gz`, SHA256 `7533ee8f6e85dbbce517141b368091f14886f3c3e532f1cc564314df4e67ce98`, verified against Jetson. The archive contains actual binaries, sources, original disabled/fault logs and offline checks. Early file-only packaging failures (missing ORT symlink and omitted audit source) are retained; both were fixed before any hardware validation. No generic datasets are included.
