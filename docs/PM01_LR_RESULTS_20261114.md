# PM01 learning-rate comparison: completed results

Reviewed on November 14 at approximately 10:45 China time. Both continuation
arms reached update 10000. All eight milestone audits completed without
evaluator errors at 10:17. Final local backup completed at 10:29. Independent
seed 44/45 training started automatically at 09:59 and is still running.

The retained baseline remains PM01 7629. No new checkpoint dominates all of
its forward gait, stopping and robustness properties. These are simulation
research candidates, not hardware-qualified policies.

| Candidate | Forward vx at command 0.25 (m/s), seed 101 | Forward swing peak medians L/R (mm) | Left/right yaw at command +/-0.30 (rad/s) | 60 s zero-command mean horizontal speed (m/s) |
| --- | ---: | --- | --- | ---: |
| Parent 7629 | 0.1958 | 20.2 / 18.9 | +0.2345 / -0.2331 | 0.000534 |
| Quarter actor LR, 8500 | 0.1970 | 16.2 / 17.4 | +0.3005 / -0.2999 | 0.001122 |
| Quarter actor LR, 10000 | 0.2499 | 19.1 / 18.0 | +0.2967 / -0.2720 | 0.014088 |
| Control, 10000 | 0.1944 | 16.2 / 12.5 | +0.3131 / -0.3262 | 0.001604 |

Forward and turn entries use the 20-second standard evaluation, scored after
three seconds. Zero-command entries use the separate 60-second evaluation.
The ledger contains all three seeds and all intermediate candidates. All eight
candidates survived all 45 forward/stand/turn/stop priority cases under each
of three seeds; survival alone does not establish recovery or gait quality.

## Candidate decisions

**Retain quarter-LR 8500 for turning and quiet stopping.** Its 60-second left
and right curve yaw rates were +0.2997/-0.2987 rad/s; forward speed remained
about 0.198 m/s. The zero-command tail settled essentially to rest. All 40 force
events met the supplemental command-matched velocity/yaw-rate/tilt recovery
criterion in each seed, with maximum confirmation times 4.34/4.36/5.08 seconds.
However, forward foot clearance is lower than the parent, and uncommanded
forward-run yaw rate averaged about -0.0202 rad/s over the long run. A settled
heading offset after a disturbance is acceptable; continued uncommanded
rotation during ordinary forward motion is a different diagnostic.

**Retain quarter-LR 10000 for forward-speed progress, not as the default.** Its
60-second forward speed was 0.2548 m/s and bilateral clearance remained close
to the parent. Tradeoffs are greater zero-command motion, curve forward-speed
overshoot, and only 36/37/37 of 40 force events satisfying the supplemental
recovery criterion across the three seeds. Investigate the unmatched events
before interpreting them as a specific failure mechanism. The zero-command
tail still had about 0.0052 m/s mean horizontal velocity and 0.0177 rad/s mean
yaw rate; the long forward run had about +0.0165 rad/s uncommanded yaw rate.

**Keep the control result as evidence.** The final control stands quietly and
recovers under the supplemental criterion, but the right-foot swing median
fell to 12.5 mm without a forward-speed improvement over the parent.

Lower actor learning rate is useful evidence, not a universal cure: its 9000
checkpoint had approximately 0.101 m/s zero-command mean horizontal speed,
then its 10000 checkpoint changed behavior again. Preserve intermediate
checkpoints and physical evaluations rather than selecting by update number
or training return.

## Preserved artifacts and next comparison

Local evidence is in `outputs/retrain-20261114/actor-lr-comparison`:
`COMPARISON.md`, `comparison-ledger.json`, all audited checkpoints and actual
rollouts/videos, and `reviewed-shortlist-20261114.json`. The shortlist records
checkpoint SHA-256 values, verified against evaluation metadata. The earlier
7629 checkpoint and full-minute video remain available.

Independent seeds 44 and 45 test whether the PM01 recipe reproduces useful
behavior without relying on seed 43. They use the original full learning-rate
schedule and do not inherit the new quarter-LR checkpoints. Their primary
audits are at updates 6000, 7500 and 9000. Do not describe these queued/running
replications as finished results.
