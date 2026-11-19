# Jetson CPU replay latency diagnosis, November 17

The dominant reproduced 3-to-7 ms mode is caused by a changing hardware
performance state under paced inference. A controlled CPU-floor intervention
at the existing 1344 MHz cap removes it; restoring the original floor brings
it back. The intervention also keeps the measured EMC clock at 665.6 MHz,
whereas the original mode switches EMC between 204 and 665.6 MHz. CPU and
memory contributions are coupled in this test, so their independent shares
are not established. This finding does not attribute every historical outlier
or prove that latency caused the physical torso lean.

## Scope and identity

All experiments run a standalone C++ file-only probe with ONNX Runtime 1.23.2
on Jetson Orin Nano. No CAN, IMU, joystick or controller IPC is opened. The
existing runtime binaries, model, gains and protection files are unchanged.
The caller verifies that controller processes are absent before CPU load.

Policy SHA256:
`ec638542ea681da6fcd56a9888f3f09ef13eb92a60ab1d75b9aae9131600abf6`.
Inputs and expected raw actions come from the saved November 16 powered run
`xbox-powered-20261116-153129-5687/native.bin`. The handshake record is skipped.
Each call uses the same recorded 993-element observation, independent of
earlier replay outputs. This isolates inference from closed-loop behavior.

Sources: `scripts/native_cpu_replay_probe.cpp`, `scripts/run_cpu_replay_probe.py`,
`scripts/summarize_cpu_replay_probe.py`, `scripts/plot_cpu_replay_diagnosis.py`,
and `scripts/cpu_replay_power_control.py`.
Local evidence: `outputs/inference-audit-20261117/jetson/`.
Remote isolated directory: `/home/tony/inference-diagnosis-20261117/`.

## Thread comparison

Each mode has two uninstrumented 1,000-frame trials at 50 Hz, with 100 warmups.
Order: single, shared, pinned, pinned, shared, single. No realtime priority,
power mode or governor is changed. The initial power mode is 25 W; CPU 4/5
share cpufreq policy4, using schedutil, minimum 729.6 MHz and cap 1344 MHz.

| Mode | Frames pooled | Median (ms) | P99 (ms) | Maximum (ms) |
| --- | ---: | ---: | ---: | ---: |
| Single thread, CPU 4 | 2,000 | 9.590 | 10.109 | 11.985 |
| Two threads, both eligible for CPU 4/5 | 2,000 | 4.067 | 6.980 | 12.957 |
| Two threads, main CPU 4 / worker CPU 5 | 2,000 | 3.569 | 6.844 | 7.131 |

Separate affinity is verified through each Linux thread ID. It does not remove
the main 6-7 ms mode. A rare 12.957 ms shared-affinity outlier lacks the detailed
counters needed to attribute that particular event; do not assign every
outlier to the same cause.

## Where slow-frame time goes

Instrumented, pinned 50 Hz replay contains 319 frames above 6 ms. Within this
group median inference wall time is 6.731 ms and main-thread CPU time is
6.686 ms. The median difference is 0.036 ms, and the measured main-thread
scheduler wait is zero at the median. OS preemption of the main thread does
not account for the additional approximately 4 ms in the common slow mode.
CPU time alone can include spinning and memory stalls, so operator profiling
is also required.

A separate ORT profile, excluding 100 warmups, compares 239 fast calls below
4 ms and 159 slow calls between 6 and 10 ms:

| Operator | Fast median (ms) | Slow median (ms) | Slow median reported thread-pool Wait |
| --- | ---: | ---: | ---: |
| Decoder 1022 -> 2048 Gemm | 1.082 | 2.642 | 3 microseconds |
| Decoder 2048 -> 1024 Gemm | 1.097 | 2.632 | 4 microseconds |

The extra time is dominated by the two large matrix operations' execution,
not the recorded thread-pool Wait fields. Kernel execution includes memory
access; this does not establish an arithmetic-only bottleneck. LayerNorm/GELU
and input assembly do not explain the approximately 3 ms growth in these two
operators. Profiling changes overhead and generated a 20.437 ms outlier;
profiled timings are diagnostic evidence, not live deadline qualification.

## Controlled follow-ups

All rows below use the same unchanged model. Each row has 1,000 calls except
the active-wait repeat, which has 1,500. Every output matched its saved raw
action exactly in these trials (maximum absolute difference zero).

| Condition | Median (ms) | P99 (ms) | Maximum (ms) |
| --- | ---: | ---: | ---: |
| Pinned, repeated identical observation, 50 Hz sleep | 4.532 | 6.841 | 6.989 |
| Pinned, ORT dynamic_block_base=4, 50 Hz sleep | 4.027 | 6.894 | 7.079 |
| Pinned, worker spinning disabled, 50 Hz sleep | 6.757 | 7.950 | 15.057 |
| Pinned, unpaced continuous inference | 2.727 | 2.792 | 3.059 |
| Single thread, unpaced continuous inference | 3.817 | 3.867 | 4.150 |
| Pinned, active wait between calls, still 50 Hz | 2.822 | 2.959 | 3.174 |
| Pinned, active wait repeat, still 50 Hz | 2.772 | 3.021 | 4.645 |
| Pinned, return to sleeping between calls | 4.494 | 6.948 | 7.139 |

The repeated identical input retains the slow mode, excluding changing robot
pose/input values as a necessary cause. Changing thread-pool block size and
disabling worker spinning do not solve it. Sustained activity removes the
6-7 ms mode in these samples, and returning to sleeping restores it.
During the instrumented active-wait trial, sampled CPU frequency remains
1344 MHz; normal sleeping trials use lower, changing frequencies.

Active waiting is an experiment, not a deployed solution: it occupies one CPU
core between policy calls. Unpaced inference also changes duty cycle and
cannot be substituted for the actual 50 Hz timing requirement.

## CPU and memory frequency isolation

The user-run privileged helper only launches the unprivileged file-only probe,
reads EMC clock telemetry, and temporarily raises policy4's CPU minimum to its
existing cap. It never raises that cap. The original minimum is restored in
`finally`, including failures and handled signals. No controller is launched.

The first attempt completed the baseline then stopped at immediate CPU-floor
readback validation. The old minimum was restored to 729600 kHz. Attempts to
rerun that version correctly refused to overwrite its evidence. The corrected
version waits up to three seconds for asynchronous readback, reports actual
values on failure, and gives each run a unique evidence prefix. Delayed
readback and restoration paths were checked locally before deployment.

The first baseline's privileged telemetry records EMC switching between
204 MHz and 665.6 MHz. Using the most recent sample preceding inference,
265 of 278 calls above 6 ms (95.3%) fall in the 204 MHz category. This is
strong evidence of a memory-performance-state contribution, not proof that
all delay is caused by EMC alone: sampling is approximately 10-20 ms and
CPU/EMC states can change within one inference.

The corrected helper completed on the user's terminal. Its trace explicitly
shows asynchronous CPU-floor readback transitioning from 729600 to 1344000,
and back to 729600 kHz during restoration. All three phases retain 50 Hz
sleep pacing, the same two threads separately pinned to CPU 4/5, and the
same model/recorded observations. Each phase contains 1,000 measured calls.

| Phase | Median (ms) | P99 (ms) | Maximum (ms) | Calls above 6 ms |
| --- | ---: | ---: | ---: | ---: |
| Original 729.6 MHz floor / schedutil | 3.463 | 6.974 | 7.164 | 283 |
| CPU floor temporarily set to existing 1344 MHz cap | 2.801 | 2.846 | 3.079 | 0 |
| Original floor restored | 3.130 | 6.950 | 7.031 | 290 |

In the middle phase every clock sample within the inference interval reads
CPU 1344 MHz and EMC 665.6 MHz. Before and after it, both clocks vary and EMC
switches between 204 and 665.6 MHz. There was no explicit EMC clock write.
This is a controlled before/intervention/restoration result supporting the
combined power-state mechanism; it does not isolate CPU arithmetic frequency
from memory-controller effects or exclude additional rare scheduler outliers.

`scripts/analyze_cpu_power_control.py` saves the frame/clock join, validation
summary and `cpu_emc_control.png` under
`outputs/inference-audit-20261117/power-control/`. The successful raw prefix is
`power-20261117-123602-4536`. Clock samples precede inference by up to 23.1 ms,
so individual transition-frame clock assignments must not be treated as exact.

Across all 23,000 measured calls in the saved comparison set, output maximum
absolute difference from the corresponding recorded raw action is zero.
This count excludes warmups and the initial 50-call smoke test. It includes
the separate heavily instrumented operator-profile run; that run must not be
pooled into normal latency qualification.

Final SSH readback confirms minimum 729600 kHz, cap 1344000 kHz and schedutil.
The policy hash is unchanged, and no probe or robot-control process remains.
No motor frame or enable was sent by this work. This CPU-only workflow does
not read motor states and therefore does not claim a new 31-motor disable
verification.

## Interpretation and remaining gate

The reproduced slow mode is tied to the intermittent execution/power state,
with the large decoder matrix operations becoming slower. Missing CAN FIFO
permissions, a left-hip-specific bus delay, changing pose values, and simple
main-thread descheduling do not explain these CPU-only measurements.

The model can run below 3 ms on this CPU without changing weights. A literal
3 ms hardware floor is disproved by the continuous and active-wait controls.
However, any eventual runtime tuning still needs the real concurrent IMU/IPC/
CAN no-enable workload and existing freshness protections. CPU-only results
do not establish improved physical walking or explain persistent torso lean.

A concrete candidate is to keep the inference cluster at its existing
1344 MHz cap while the control service is active, with verified entry and
restoration on exit, while preserving CAN CPU 2 / FIFO 50 and the existing
50 Hz policy. This has not been installed in the runtime. Qualify it with
concurrent no-enable IMU/IPC/CAN timing before a powered test. Do not replace
sleeping with a permanent busy loop merely because that diagnostic also
reduced the tail.

Relevant ORT references:
[thread configuration](https://onnxruntime.ai/docs/performance/tune-performance/threading.html),
[latency variance guidance](https://onnxruntime.ai/docs/performance/tune-performance/troubleshooting.html),
and [the deployed version's supported keys](https://raw.githubusercontent.com/microsoft/onnxruntime/v1.23.2/include/onnxruntime/core/session/onnxruntime_session_options_config_keys.h).
