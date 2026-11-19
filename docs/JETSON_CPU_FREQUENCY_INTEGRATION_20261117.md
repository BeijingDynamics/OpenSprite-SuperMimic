# Automatic CPU frequency lifecycle and disabled integration

## Result

The new isolated runtime stage is
`/home/tony/supermimic-pm01-cpufreq-1117`. The inference lifecycle now acquires
CPU 4/5 at 1344 MHz and restores the original CPU settings after shutdown.
Full 54-second no-enable integration and the unchanged 20 ms timeout injection
passed. Every hardware run ended with individual disable feedback from all
31 motors and zero enable attempts. No powered test was run.

The model, native CAN/actor/IMU binaries, training gain configuration, motor
limits, transmission mapping, target protection, action history and 50 Hz
policy / mixed-rate CAN schedule remain unchanged. The script verifies their
fingerprints across runs. CAN remains CPU 2 / SCHED_FIFO 50; the actor remains
two ORT threads on the CPU 4/5 mask. This change does not impose new separate
per-thread affinity on the live actor.

## Lifecycle implementation

- `runtime/tools/cpu_frequency_guard.py`: a narrow root-owned Unix-socket
  service, installed as `open-sprite-cpu-frequency.service`. It accepts only
  the configured user/root, only the fixed 1344000 kHz lease, and only the
  expected CPU 4/5 cluster with an existing 1344000 kHz cap. It does not
  execute caller commands or open robot devices.
- `runtime/tools/cpu_frequency_client.py`: unprivileged acquisition, liveness
  check and explicit restoration acknowledgement.
- `runtime/tools/run_supermimic_commissioning.py`: obtains the lease before
  session preparation, passes its descriptor to the actor, checks service
  liveness while waiting, and releases it after actor/controller cleanup.
  Acquisition/restoration evidence is saved in `cpu-frequency.json` and
  `commissioning.json`. The installed guard's source hash must match.
- `runtime/tools/install_cpu_frequency_guard.sh`: one-time root installation.
  The root-owned helper and systemd unit persist across boot. The idle service
  does not lock frequency. No recurring manual sudo/setcap is required for
  lease acquisition.

The state journal is written before changing the frequency floor. Normal
release, client disconnect and service termination restore the snapshot.
Systemd's `ExecStopPost` and startup recovery handle a killed service. If the
launcher dies while its actor is still alive, the inherited descriptor retains
the lease until that actor exits. A competing client cannot steal the lease.
Neither CPU maximum frequency nor governor is changed. The service verifies
the cap/floor during the lease; a lost lease causes commissioning cleanup
through the existing termination path, not a relaxed control deadline.

Seven Linux fake-filesystem tests passed: normal release, client SIGKILL,
actor inheritance, daemon SIGKILL/restart recovery, wrong-cap rejection,
competing-client rejection and external frequency-change notification.
An additional installed-service test killed a dummy client with SIGKILL and
confirmed real frequency restoration, without accessing hardware devices.

## Live timing comparison

`baseline-disabled-01` used the same new launcher without the lease; it
dispatched 1,806 formal actions before an inference lasting 15.356616 ms
triggered the unchanged oldest-feedback deadline. Its statistics below cover
accepted/dispatched actions only. The rejected final action is not silently
included as a transmitted frame.

`fixed-disabled-01` completed all 2,700 formal actions / 83,700 first motor
transmissions. It passed the existing per-action TX audit, including all
hardware timestamps, coherent groups, 20 ms timing and 2 ms one-bus spans.

| Metric | Original dynamic frequency | Automatic 1344 MHz lease |
| --- | ---: | ---: |
| Inference median | 3.025 ms | 2.796 ms |
| Inference P99, dispatched frames | 6.724 ms | 3.125 ms |
| Maximum inference, including rejected baseline frame | 15.357 ms | 3.483 ms |
| Oldest feedback -> all driver handoffs, median | 9.747 ms | 9.565 ms |
| Oldest feedback -> all driver handoffs, P99 | 13.573 ms | 9.751 ms |
| Oldest feedback -> all TX completion receipts, median | 10.424 ms | 10.379 ms |
| Oldest feedback -> all TX completion receipts, P99 | 14.380 ms | 10.465 ms |
| Oldest feedback -> all TX completion receipts, maximum | 16.347 ms | 12.351 ms |
| Left hip new-action hardware interval, P99 | 22.068 ms | 20.117 ms |
| Left hip new-action hardware interval, maximum | 24.024 ms | 20.175 ms |

The end-to-end receipt P99 decreases by approximately 27.2%. Median latency
changes little; the material benefit is lower tail latency and action interval
jitter. The fixed 2 ms coherent send boundary and the existing sensor pipeline
remain part of the latency budget.

CAN feedback timestamps are conservative host conversions of kernel RX,
not measured motor encoder sampling instants. Hardware completion receipt
is a host upper bound on TX completion, not acknowledgement of motor torque
execution. Left-hip raw hardware intervals use one adapter clock only. These
measurements are zero-gain disabled transport, not powered gait qualification.
The failed and complete runs have different sample lengths; do not treat this
as a large statistical study or a hard worst-case execution-time guarantee.

## Timeout test and retained failures

`fixed-fault-02` passed the unchanged timeout-injection gate: actor suspension
caused `target_ipc_pending` and the oldest-feedback 20 ms rejection. It had
2,250 coherent action groups before stopping, zero enable attempts, and all
31 motors confirmed disabled. The frequency lease was released and restored.
Its maximum observed one-bus hardware group span was 0.600 ms.

The first attempt, `fixed-fault-01`, correctly stopped on the injected deadline
and restored frequency, but failed the separate hardware TX audit: sequence
1075 on kcan1 spanned 2.021 ms against the unchanged 2.000 ms bound. At the
time, the prior logs were being compressed/transferred. Concurrent archival
load is a plausible contributor, not a proven sole cause. The failed evidence
is retained. Repeating with no concurrent archive/transfer passed; the audit
limit was not relaxed. Keep archive/compression out of active control tests.

## Deployment and final state

The updated operator entry point in this stage acquires the lease
automatically and requires `fixed-disabled-01` plus `fixed-fault-02` with
matching fingerprints. Only its `--check` path was run; it passed without
starting CAN/IMU or enabling motors. The old stage remains a historical
deployment and does not automatically inherit this frequency behavior.

Final readback after the hardware sessions and dummy-client crash test:
minimum 729600 kHz, maximum 1344000 kHz, governor schedutil. The frequency
service remains idle and available. Hardware sessions confirmed all motors
disabled before the later CPU-only dummy test.

Local evidence: `outputs/cpufreq-integration-20261117/`, including raw passed
and failed logs, comparison CSV/JSON, the installed-service crash test and
hash manifest. Reproduce the timing join with:

```bash
python scripts/compare_live_cpu_frequency.py outputs/cpufreq-integration-20261117/live --runs baseline-disabled-01 fixed-disabled-01
```

The CPU-only mechanism diagnosis remains in
[the earlier replay report](JETSON_CPU_REPLAY_DIAGNOSIS_20261117.md). These
results establish a timing improvement; they do not yet establish an
improvement in foot clearance, stance width or torso stability on hardware.
