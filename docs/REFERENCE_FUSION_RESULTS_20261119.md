# Standing-reference fusion results and candidate handoff

The overnight campaign completed at 03:56:10 on November 19, Asia/Shanghai.
Both matched arms reached 6000 updates. Scheduled nominal, initial-tilt,
disturbance, delay and export stages completed. The bounded supervisor then
exited; the GPU was idle when inspected at 11:44. The morning review began
after the intended 11:00 handoff, and additional long-duration probes were
started during that review. Do not describe the idle interval as training.

## Candidate and matched comparison

The selected research candidate is `walk-stand80-6000`, with checkpoint SHA-256
`70cf3e70354c8b574777709f56f24f83864329d589c256cfa97d6a1236afdedd`.
It mixes the unchanged PM01 walking samples and reviewed Neutral standing
samples at 80/20 reset clip mass. Model, action scale, joint gains and rates
remain unchanged. No robot access or motor enabling occurred in the campaign.

| Metric | Parent | PM01-only continuation, 6000 | Walk plus standing, 6000 |
| --- | ---: | ---: | ---: |
| Standing median knee magnitudes L/R, degrees | 35.8 / 13.8 | 30.3 / 17.3 | 10.7 / 12.6 |
| Standing backward torso pitch, degrees | 1.37 | 2.11 | 0.51 |
| Forward0.25 actual speed, m/s | 0.207 | 0.193 | 0.177 |
| Forward0.50 actual speed, m/s | 0.324 | 0.338 | 0.329 |
| Forward0.50 swing peaks L/R, mm | 19.8 / 15.0 | 20.1 / 17.0 | 30.4 / 18.6 |
| Right-turn swing peaks L/R, mm | 16.8 / 16.1 | 18.0 / 22.1 | 27.2 / 19.4 |
| Right-turn mean yaw rate, rad/s (command -0.2) | -0.209 | -0.219 | -0.177 |

The standing improvement is substantially larger than the matched continuation,
supporting the benefit of the added standing reference in this seed. All seven
nominal cases survived. Forward0.5 has 28/28 qualifying lift events over the
same scoring window. Equal event counts do not prove full gait symmetry:
the right foot remains lower and spends more time loaded. The 0.25 m/s speed
response is weaker than the parent even though foot clearance increases.

The candidate does not pass every existing selection check. Left-turn P05
foot-center spacing is 108.4 mm versus approximately 114.8 mm in the parent,
a reduction of about 6.5 mm against a 5 mm retention allowance. The absolute
105 mm spacing check passes, but the failed retention check remains failed.
Standing P05 spacing is approximately 182.5 mm, considerably wider than the
parent's 122 mm. Standing appearance, gait asymmetry and turn response still
need review before hardware use. No threshold has been relaxed.

## Completed validation

- Two independent initial-tilt probes (seeds 202/303, up to 2 degrees, 30 seconds):
  14/14 cases survived; forward0.5 actual speeds are approximately 0.328-0.329 m/s.
- Four-direction 10 N, 0.2-second torso forces and lateral 5 N, 1-second pulls,
  during standing and walking: 12/12 cases survived. Last-five-second standing
  speeds returned near zero; walking returned to approximately 0.324-0.326 m/s.
  This is return to the biased baseline speed, not accurate 0.5 m/s tracking.
- Fixed 10 and 20 ms action-delay diagnostic plants: 14/14 cases survived.
  At 20 ms, forward0.5 swing peaks were 22.3/21.8 mm and actual speed 0.328 m/s.
  These are separate diagnostic overrides, not identified motor models.
- Exported 993-dimensional native interface: 1680 recorded-history samples
  passed ONNX parity; maximum absolute action error was 3.34e-6.

The local checkpoint and ONNX hashes were recomputed and matched their stored
identities. Checkpoints, source snapshots, reference QA, rollouts, validation
reports and videos are local under `outputs/reference-fusion-20261118/`.
The candidate export is not hardware-qualified; Jetson replay and existing
runtime checks remain separate from these simulator results.

## Morning completion work

An additional four-case, 60-second simulation probe was launched at the morning
review: joystick-like speed/turn/stop transitions, prolonged forward walking
and stop, standing fore/aft 10 N forces for one second, and walking lateral
10 N forces for one second. This increases force duration relative to the
overnight set and must be reported separately, including any failures. Results
are under the isolated server's `outputs/final-readiness-20261119`.

The physical rollout completed with all four cases surviving the full 60
seconds. The joystick-like sequence and prolonged forward walk both settled
after the command returned to zero: last-five-second mean planar speed errors
were approximately 0.00016 m/s. Standing recovered after each one-second
fore/aft 10 N force and ended near zero speed. Following the one-second lateral
10 N forces during walking, final mean forward speed was 0.329 m/s, close to
the undisturbed candidate response. This confirms the bounded simulated cases,
not arbitrary human-push robustness or real-motor performance. Global heading
restoration was not required.
