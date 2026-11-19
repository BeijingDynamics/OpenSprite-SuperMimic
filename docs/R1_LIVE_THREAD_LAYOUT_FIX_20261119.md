# R1 live inference thread placement

## Reproduced problem

The faster-start profile could not qualify because the live actor occasionally
took 9 to 15 ms despite healthy CPU-only playback. CAN timing limits remained
unchanged. Two failures were preserved: accept-to-send reached 2.527481 ms
(limit 2.5 ms), and another action aged past 20 ms between acceptance and send.
Neither case justifies discarding evidence or relaxing a timing threshold.

The live actor contains four Linux threads: inference main, one ONNX Runtime
worker, an IMU IPC receiver and a joystick receiver. Previously all inherited
the process mask `{4,5}`. The CPU-only probe has no live receiver threads.
The IMU producer remains a separate C++ process on CPU 3; the policy IMU
receiver polls it every 5 ms. These are distinct threads and rates.

## Diagnostic evidence

Optional per-call instrumentation buffers main/process CPU time, context
switches, current CPU, and scheduler counters. It writes after control exits.
Diagnostic-on and diagnostic-off output records matched byte for byte on a
1,000-frame R1 recurrence fixture. `/proc/sys/kernel/sched_schedstats` is zero
on this Jetson; scheduler wait counters are therefore not used as evidence
of zero waiting. CPU clocks and `getrusage` counters remain useful.

| Same live 146-second workload | Shared mask `{4,5}` | Isolated compute and IO |
| --- | --- | --- |
| Median instrumented inference | 2.886 ms | 2.796 ms |
| P99 | 3.313 ms | 3.032 ms |
| Maximum | 10.501 ms | 4.141 ms |
| Calls above 6 ms | 7 | 0 |
| Full live timing audit | Failed | Passed |

The slowest shared-layout call recorded 40 involuntary context switches,
5.136 ms main CPU time and 10.525 ms total process CPU time over 10.501 ms wall
time. Other slow calls migrated between CPU 4 and 5. The isolated run retained
the same model, frequency lease, action semantics, CAN build and protection
thresholds. Both runs confirmed 31 motors disabled and zero enable attempts.

The controlled change combines dedicated compute placement and moving receiver
threads off those cores. It supports this combined layout as a remedy under
the live workload; it does not separately quantify the contribution of each
receiver, migration, memory contention or ORT worker behavior.

## Implementation

- Inference main: CPU 4.
- The single ORT intra-op worker: CPU 5.
- Policy IMU IPC and Xbox receiver threads: CPU 1.
- CAN: CPU 2 / SCHED_FIFO 50, same binary and capability.
- IMU acquisition/fusion producer: CPU 3, unchanged.
- CPU 4/5: existing 1.344 GHz lifetime lease and automatic restoration.

The actor identifies the thread created by ORT, requires exactly one worker,
pins it and the main thread, and pins each IO thread through its native handle.
Before connecting the control stream, every thread's actual affinity is read
back and saved to `native.bin.affinity.json`. Unexpected thread counts, missing
affinity or unreadable verification fail startup. The final launcher always
requests this layout, and the operator gate requires matching affinity evidence.
No realtime priority or new capability is assigned to the actor.

The old and isolated actors, with instrumentation both off and on, produced
identical recorded outputs (SHA-256
`0bc4612dab90c5e78b8cb59a82110b07cb885bf5b4e3d7cec71e6a26dd8ac076`).
All 10 native tests passed. The original CAN binary hash remains
`c7ce58b948196779d8dcf03bbae71174e081530f955a1758ac4011d82cc1aabd`.

## Final validation

Two full uninstrumented disabled runs and a late-window fault injection are
required with the sealed launcher. Qualification is pending at implementation.
Finite passing trials cannot prove hard realtime behavior under every future
system load; all original timing checks remain enabled. No powered run is
started by this work.

### Residual CAN delay and stale target cache

The first uninstrumented isolated run did not qualify. All 7,301 policy states
were processed and all 31 motors were confirmed disabled, but action 2,959
waited 2.928 ms from acceptance to its first write and 3.116 ms to its last.
The writes themselves spanned only 0.188 ms. This is distinct from slow model
inference: the remaining delay preceded the group's first write. The unchanged
2.5 ms accept-to-write audit correctly rejected the run.

Inspection found two avoidable implementation costs and one correctness defect:

- A received action waited until the next 2 ms mapping boundary even though the
  transport thread already wakes every 0.5 ms.
- The CAN target was compiled without optimization. This increases the cost of
  matrix mapping and nested coupling calculations; it is not proof of the full
  cause of the observed 2.928 ms delay.
- `preview_final_motor_commands()` refreshed direct-drive target caches only
  when `motor_preview_count % 10 == 0`, while ankles used the current action.
  A coherent send label did not guarantee coherent payload contents. A new
  action arriving at other phases could send an older direct-drive target.

The new commit refreshes all motor targets from the accepted action, validates
and encodes the full group, and dispatches in the current transport slot.
Between policy actions, periodic ankle remapping remains enabled. Model rate,
gains, PD saturation and safety thresholds are unchanged. CAN uses `-O2` with
`-ffp-contract=off`, without fast-math. This rebuild needs the existing
`CAP_SYS_NICE` permission restored; its actual CPU 2 / FIFO 50 placement is
verified before hardware use.

The periodic schedule remains 50 Hz for ordinary motors and 500 Hz for parallel
ankles. Coherent new-action sends are additional events when they do not coincide
with a nominal send slot; these nominal rates are not claims about exact total
frame counts on each bus. The policy remains 50 Hz. Existing per-packet TX traces
and monitor coverage checks continue to audit both kinds of send.

Production-code regression tests reproduce stale direct-drive caches at each
nonrefresh phase, then compare all 31 encoded packets against a fresh mapping.
A recorded 7,301-frame no-enable sequence passes 2,263,000 phase/motor checks.
The deliberately stale replay differs by as much as 0.163034 rad; this is a
counterfactual phase sweep, not a measurement of actual historical wire targets
or proof that it caused a particular walking symptom. Feedback for this replay
is reconstructed from recorded joint states and the exported linear mapping;
it does not replace live nonlinear ankle feedback validation.

Failed evidence is retained as `failed-isolated-dispatch-01`. Final qualification
must use the corrected CAN build, with inference diagnostics disabled.

Optimized and unoptimized replay output files are byte-identical, SHA-256
`b2ffb63bf28bb7b53ff9564a6eeb54992a72da5926137ad41f56d7ac4e4205ff`.
The same phase sweep took 8.128 seconds with `-O2` and 74.282 seconds with `-O0`.
This is a CPU-only workload comparison, not a claim that end-to-end control
latency improves by the same factor. No CAN interfaces are opened by the replay.

## Reproduction and source retention

The deployment changes are isolated under
`outputs/deploy-sm-walk-r1-fast-20261119/source`, preserving prior stages.
The preparation sequence after `prepare_r1_fast_start.py` is
`patch_r1_live_diagnostics.py`, `patch_r1_isolated_inference.py`,
`finalize_r1_inference_layout.py`, then `patch_r1_atomic_commit.py`.
`r1_atomic_replay.cpp` reads recorded inputs and actor outputs without opening
hardware. `test_r1_atomic_commit.inc` is included by the production-code native
test. Final source is archived as `source-atomic-commit.tar.gz` with an individual
file manifest, rather than overwriting the prediagnosis source archive.

## Corrected-build long runs

Both consecutive 146-second no-enable runs passed with inference diagnostics
disabled. Each contains 7,300 coherent groups / 226,300 motor sends, zero enable
attempts and all 31 individual disable confirmations. CPU frequency was restored.

| Metric | First run | Second run |
| --- | --- | --- |
| Maximum inference | 8.022 ms | 11.076 ms |
| Accept to driver handoff upper bound | 0.638 ms | 0.481 ms |
| Whole-group driver handoff span | 0.528 ms | 0.278 ms |
| Oldest feedback to driver handoff upper bound | 14.615 ms | 17.662 ms |
| Actor target to hardware completion receipt upper bound | 1.994 ms | 2.045 ms |

The final runs demonstrate that inference tails still occur. The earlier
instrumented 4.141 ms maximum is not a permanent latency bound. The corrected
send path removes avoidable waiting and stale caches; both long tests passed
the unchanged 2.5 ms and 20 ms audits. Hardware completion receipt is an upper
bound on adapter transmission, not acknowledgement of motor execution.
These trials used the previous timestamp driver. They are preserved as
`passed-atomic-previous-driver-01/02`; final qualification must be repeated with
the echo-order correction below.

## Timestamp registration race found by the fault trial

The late-window timeout correctly stopped at `target_ipc_pending`, with all 31
motors disabled. Its TX audit nevertheless failed on an earlier timestamp:
`kcan2` packet 49,958, action 1,488, was associated with hardware completion
approximately 153 ms after its send. Packet 50,214 (exactly 256 packets later)
had neither software nor hardware completion. At the matching wall-clock time,
the kernel reported:

```text
kcan 1-2.1:1.0 kcan2: can_put_echo_skb: BUG! echo_skb 35 is occupied!
```

The loaded driver made a frame visible through `kcan_fifo_put()` and
`__kcan_dev_start_writing()` before `can_put_echo_skb()` registered its echo.
An early completion can clear the reserved marker while the skb is absent.
Subsequent registration leaves an orphaned skb; after the 8-bit marker wraps,
the new skb is rejected as occupied and the new completion is matched to the
old skb. This explains both the occupied-slot message and the 256-packet offset.
It is not evidence that the motor executed an old command 153 ms later.

The isolated candidate registers and checks the echo before FIFO publication,
and releases both skb and reserved marker if enqueue fails. The installed
driver is not overwritten. The source patch, kernel evidence, failed trial and
module hash are retained. The offline ownership model reproduces the old race
and checks completion, ring wrap and error cleanup, but does not replace kernel
and hardware validation. Driver source and license notices remain unchanged
except for the targeted patch.

Candidate module SHA-256:
`c8587bb874ad973d50ee59136ce6e3695af162cdf070e89673b84fe6c985528a`.
Candidate source version: `404E8D65A6EFF60079EAEC1`.
The local audit thresholds are unchanged. Driver loading requires onsite sudo;
the dedicated loader configures CAN/timestamps without sending motor frames and
rolls back to the preceding temporary module on setup failure. Reboot restores
the installed module, so final operator qualification also checks the loaded
source version. Final live validation and evidence export remain pending.

The first loader attempt failed because the isolated stage omitted
`tools/can_tx_timestamp_setup.py`. The module rollback restored the previous
temporary source version; the buses were UP, but the missing helper also
prevented timestamp re-enabling during rollback. The unchanged helper has now
been copied from the previously validated stage. The loader checks every file,
required executable, helper import and module hash before unloading anything;
its unprivileged `--check` passed. No motor session was started during this repair.

## Residual inference tail after the driver fix

The first final-driver disabled run passed, but its same-build repeat failed
the unchanged 20 ms hardware-completion audit. The repeat's maximum accepted
target age was 19.270 ms; inference reached 11.583 ms. It completed 7,300 action
groups and confirmed 31 disabled motors. No new occupied-echo-slot messages
appeared. This is a distinct timing failure, not the earlier timestamp identity
race, and is retained rather than relabelled as a pass.

The actor now exposes diagnostic alternatives: one ORT compute thread pinned
to CPU 4, or two pinned threads with idle spinning disabled. IO remains on CPU 1;
the CAN binary, gains and timing limits are unchanged. A 1,000-frame recurrence
comparison produced byte-identical outputs in all three modes (same SHA-256
`0bc4612dab90c5e78b8cb59a82110b07cb885bf5b4e3d7cec71e6a26dd8ac076`).

| CPU-only paced mode | Median inference | Maximum inference |
| --- | --- | --- |
| Two threads, default spinning | 2.740 ms | 3.107 ms |
| Two threads, idle spinning disabled | 2.740 ms | 3.249 ms |
| One thread | 3.858 ms | 4.322 ms |

These CPU-only maxima do not prove live timing behavior. A full live
single-thread trial is required before selecting it for the operator profile.

The first single-thread live trial (`single-disabled-01`) passed, with maximum
inference 9.005 ms and maximum oldest-feedback-to-target-accept 18.334 ms.
This does not provide a clean latency margin despite eliminating the ORT
worker. A live two-thread, no-idle-spinning comparison is therefore required;
the single-thread pass alone is not the selected final operating mode.

## Selected inference mode

The selected mode retains two compute threads on CPU 4/5 and sets
`session.intra_op.allow_spinning=0`. The operator entry explicitly exports this
choice, fixes the thread count to two and disables optional instrumentation.
Its evidence gate requires the matching no-spinning report. The disabled helper
uses the same fixed options; it cannot silently fall back to default spinning.
No extra realtime capability is granted to the actor.

Both full 146-second trials passed on the corrected driver:

| Trial | Maximum inference | Maximum oldest feedback to target acceptance |
| --- | --- | --- |
| `sleeping-disabled-01` | 4.221 ms | 10.815 ms |
| `sleeping-disabled-02` | 6.045 ms | 12.795 ms |

Both have zero enable attempts, 31 individual disable confirmations and restored
CPU-frequency settings. Each passed the unchanged per-action hardware TX audit.
The prior spinning and single-thread results are retained as comparisons, not
counted as final no-spinning qualification. `sleeping-fault-01` also passed:
the original 20 ms timeout stopped the controller and all 31 disable states
were confirmed. Whole-stream timestamp coverage, kernel-log inspection and
verified local evidence transfer are the final collection steps.

The final collector passed: 712,777 sends in `sleeping-disabled-01`, 714,103 in
`sleeping-disabled-02`, and 658,700 in `sleeping-fault-01`, with zero missing
software or hardware completion timestamps. No occupied echo-slot errors were
reported in the qualified kernel-log interval. The full-run maximum oldest
feedback to hardware TX receipt was 11.857217 / 13.840905 ms; maximum actor target
to all-driver handoff was 0.964878 / 0.957512 ms. Physical motor execution is not
measured by these adapter completion bounds.

The final evidence archive is 887,778,404 bytes, SHA-256
`08b495c8f9de3e86f47a38bf8bd2f7f4866255a6d02223fe8dea0255d964a277`.
It includes the exact R1 ONNX model, CAN and actor binaries, driver source and
module, final successful trials and prior failures. Final first-party source is
separately retained in `source-final-no-spin.tar.gz`, SHA-256
`73d752522aa16adc745028827658878399620d26ec40b6493958e9ca3a5dff5f`.
All eleven deployed source entries present in the final build manifest match
their local counterparts. This handoff permits the operator-controlled protected
trial; it does not claim powered validation of the updated profile.

Local transfer verification completed successfully. Archive size and SHA-256
match the Jetson manifest. Nineteen selected files were materialized under
`outputs/deploy-sm-walk-r1-fast-20261119/validated`, including the exact R1 model,
CAN/actor binaries, corrected driver and all final run reports. Binary/model
hashes match the final build manifest. The machine-readable receipt is
`local-backup-verification.json` in the deployment output directory.
