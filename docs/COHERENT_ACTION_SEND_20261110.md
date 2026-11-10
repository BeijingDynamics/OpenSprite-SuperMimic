# Coherent action sending on Jetson 253

Validation status: final same-build disabled and deliberate deadline-fault validation PASSED (`20261110-192830-7325`). This change has not yet undergone a powered walking test. No powered run is authorized by this document.

## Change and preserved controls

The old packed schedule sent 27 low-rate motors early in each 20 ms cycle, published feedback at 6.5–8.5 ms, and usually received the next policy target after those motors' slots. Ankles adopted that target earlier than hips/knees. This was additional transport latency despite passing the existing oldest-feedback-to-IPC-accept deadline.

`--coherent-action-send` requires direct policy startup. At the next 2 ms motor-mapping boundary after a newly accepted action, the CAN thread validates all 31 mapped payloads before writing any of them, then dispatches one whole group. It preserves action sequence identity; skipped or regressed sequences are rejected. Regular refreshes use the currently committed action. CAN serialization still produces a finite group span, which must be measured.

The original periodic refresh remains (four ankles at 500 Hz, other 27 motors at 50 Hz). The new group adds one dispatch per policy cycle; therefore total frame traffic increases. This is not a claim that all 31 motors now receive 500 Hz CAN commands, nor a change to the motor's internal PD loop. Existing temperature filters and RMS diagnostics consume nominal-cadence samples; all received frames retain instantaneous position, speed, torque and raw-temperature checks.

Policy binary, weights, action history, action scale, all 31 training Kp/Kd, pure-PD dynamic position saturation, parallel-ankle mapping, position/velocity/temperature/feedback torque limits and 2 ms/20 ms guards are preserved. The existing 20 ms deadline is also checked before each member of a new action group is dispatched. No feedforward torque is introduced. Source comparison confirms the relevant protection/mapping functions are unchanged.

## Evidence and clock interpretation

`can.json.tx.csv` records interface, socket packet ID, motor ID, action sequence, oldest feedback and accept timestamps, whole-group flag, powered flag, syscall entry/exit, software driver TX stamp, raw adapter TX stamp and receipt of the hardware completion notification. It is written after shutdown rather than from the real-time loop.

The adapter uses raw device clocks (`ts_mode=3`), not host wall time. Do not subtract a policy monotonic timestamp directly from a raw hardware timestamp. `action-motor-latency.csv` gives host-time physical-send bounds from send-call entry through receipt of the hardware TX completion. Per-bus hardware group spans use differences within the same device clock. A hardware TX completion is not a motor execution acknowledgement.

`action_tx_audit.py` checks all 31 members for every accepted action, sequence order in both host dispatch and hardware timestamps, hardware/SW timestamp completeness, nominal monitor cadence, original 20 ms completion bound, group spans and final 31-motor disable readback. Missing hardware timestamps fail admission; they are never replaced with write-return times. Validation cache identity includes the loaded driver's source version.

## Driver compatibility issue

The installed KH module exposes its timestamp handler through `ndo_do_ioctl`. Linux 5.15 routes `SIOC[GS]HWTSTAMP` through `ndo_eth_ioctl`, so configuration returned `EOPNOTSUPP`. The first isolated patch fixed that hook. A second issue then became observable: the driver's echo handler assigned the hardware stamp but never called `skb_tstamp_tx`; Linux 5.15 `can_get_echo_skb` only returns a loopback frame, not the requested error-queue TX completion. The final patch also delivers that completion with the socket packet ID. No motor-control or bus timing code is changed. References: kernel [ioctl routing](https://github.com/torvalds/linux/blob/v5.15/net/core/dev_ioctl.c) and [CAN echo handling](https://github.com/torvalds/linux/blob/v5.15/drivers/net/can/dev/skb.c).

The user explicitly approved and ran the temporary module load. It restores the existing four-bus configuration and enables TX timestamps, without sending motor frames. The installed module is not overwritten. Reboot restores the original module; timestamp preflight then blocks this candidate until the reviewed temporary module is loaded again. Module replacement is not an automatic side effect of the normal powered test script.

Final candidate module SHA256: `54b048400a4d8497fbb9301b869f3a9b257f298a01dce4c4315004b313793982`; source version `A2BA84DDB53927675FFA1CB`. The first hook-only candidate (`cac9a67f…`, source version `6DE75B7BBC5DE71FEF307A7`) is retained as failed timestamp evidence. Original installed module SHA256: `d2d68c051e5d9ee7cda59cdce38362681d2f15bb1049eabedc6dcb959bdb8576`. Vendor source/binaries remain private evidence, outside this AGPL repository; preserve their own GPL licensing.

## Checks completed before live validation

- Nine native tests passed; nine TX evidence tests and seven cache tests passed.
- 2476 recorded states: one non-policy handshake, 2475 formal policy actions. Warmup 0/100 bit-identical; new/old policy replay bit-identical. Final motor audit recompiled from the candidate matches the previous output exactly: zero rejected motor frames, 661 dynamic saturation events on the frozen recorded trajectory. This does not predict powered motion.
- Old timing logs replayed through the nominal schedule predict hip-pitch wait falling from about 10.5–10.7 ms to 0.8–1.0 ms on average; knee from 13.5–13.7 ms. This counterfactual excludes bus queues and is not hardware timing evidence.
- First disabled trial, before the driver hook fix: 1471 complete groups, all 31 members each; maximum accept-to-call 1.859 ms and software group span 0.343 ms. It failed: a 16.97 ms policy inference triggered the original oldest-feedback deadline, and hardware timestamps were absent. Zero enables, 31/31 disabled. Preserve this failure rather than presenting the run as a pass.
- Separate 2475-frame 50 Hz CPU-only replay after reboot: inference mean 4.34 ms, P99 6.95 ms, max 8.41 ms, zero deadline misses. The earlier rare inference tail is not proven eliminated.
- Hook-only driver trial `20261110-192140-5811`: complete control run, 2476 accepted targets, CAN wake max 0.080303 ms, oldest-feedback-to-target max 13.297059 ms, zero enables and 31/31 disabled. Overall admission failed because the hardware TX error-queue callback was missing. This run must not be reused as a passing validation.

Current stage: `/home/tony/supermimic-run002-20261106/walk025-sync-1110`.
Final CAN binary SHA256: `8dc2c32bc4d611c38568ad574f4ee84a25df7605c2f44b4fb858556d530742ad`.
Policy actor SHA256 remains `26d7eb913502c961d7eb89b872cf7f0ca9b61c9bfaaaf31a1a6ab0da51dc306d`.

Powered schedule remains 20 s zero command, 10 s sling-release window, 3 s velocity rise, 0.25 m/s for 10 s, 3 s deceleration and final zero command. Existing body checkpoints, onsite READY and final disable verification remain. Powered qualification requires the completed same-build disabled and fault records.

## Final disabled hardware results

| Measurement | Full disabled run | Deliberate actor-pause run |
|---|---:|---:|
| Accepted formal action groups | 2475 | 1804 |
| Matched motor dispatches with SW and hardware TX stamps | 76725 | 55924 |
| Whole-group driver span, maximum | 0.302133 ms | 0.325755 ms |
| Per-CAN-bus physical TX group span, maximum | 0.646 ms | 0.600 ms |
| Accept → driver TX, conservative maximum | 1.791933 ms | 1.811074 ms |
| Policy target → physical TX upper bound, maximum | 3.108810 ms | 3.098127 ms |
| Motor enable attempts | 0 | 0 |
| Final disabled readbacks | 31/31 | 31/31 |

All accepted actions have exactly 31 group members. No older action was sent after a newer one in the per-bus hardware timestamp order. Full run: 2476 records including handshake, CAN wake max 0.122638 ms, oldest-feedback-to-target-accept max 13.311584 ms, inference max 7.116456 ms, no deadline misses. Fault run: intentional actor pause in the forward command interval triggered the unchanged `target_ipc_pending` 20 ms guard; all previously accepted action groups retained complete TX evidence.

The entry below reuses matching final validation records, checks the actual scheduling profile and hardware timestamp settings, calibrates the IMU and requests onsite READY. It does not automatically load a different kernel driver. Keep the existing fall protection and power-switch procedure; a missing disable readback remains a cut-power condition.

```bash
bash /home/tony/supermimic-run002-20261106/walk025-sync-1110/operator-test.sh
```

After a reboot, the installed original driver is restored and this entry is blocked by timestamp/identity preflight. The reviewed temporary timestamp driver must be loaded again before using this candidate. Loaded module identity is part of the validation cache, so earlier software-only or hook-only runs cannot qualify it.

## Public source and archived deployment

The November 10 publication translates operator prompts and generated labels to
English. Historical logs and the validated Jetson stage are preserved byte for
byte; that stage still has its original prompts. The English launcher has not
been deployed or hardware-qualified. Changing it changes its fingerprint and
requires fresh same-build disabled and fault validation before powered use.

The shell entries under `runtime/tools` are templates for the recorded isolated
stage, not a general installation command. They reference site-specific bundles,
hardware configuration, reviewed profiles and evidence checkers. Do not copy a
single script onto another robot and infer deployment approval. Both the stage
launcher and validation-cache selector explicitly request direct startup and
coherent sending; old-mode validation cannot qualify this mode.

Final evidence was retrieved before Jetson shutdown. The private archive
`validation-evidence-192830.tar.gz` contains 234 manifest-verified files plus its
manifest; archive SHA256 is
`74638e6b8079e493ada227b30aee4992b1a715e081a65a02983a34ca7e7627d7`.
Raw hardware logs and binaries are kept locally rather than added to Git.
