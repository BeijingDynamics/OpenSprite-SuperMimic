# Posture and spacing continuation: results reviewed November 18

Training reached update 10000 on November 17 at 21:34 China time. The bounded
queue finished its paired initial-tilt and finite-force evaluations at 21:46.
At the November 18 inspection, the queue was complete and GPU utilization was
0%; no further training had started automatically. This used the scheduled
campaign, not the entire rental duration.

## Candidate decision

Update 5000 is the strongest research shortlist entry under the existing
nominal screening criteria. It is **not** an accepted deployment candidate.
It fails the zero-speed upright target and left-turn left-foot clearance and
clearance symmetry requirements. Update 2000 remains a useful comparison.
Updates 8000 and 10000 regress; do not select the final checkpoint by recency.

The following metrics use the five nominal 20-second cases, scoring after
three seconds and before the first failure. Negative torso pitch denotes
backward lean. Swing heights are per-foot median peaks over detected swings.

| Metric | Original parent | Update 5000 | Update 10000 |
| --- | ---: | ---: | ---: |
| Zero-speed median torso pitch, degrees | -5.03 | -3.99 | -4.58 |
| Forward 0.25 m/s median torso pitch, degrees | +1.65 | +0.97 | -3.88 |
| Forward 0.25 m/s P05 foot-center spacing, mm | 84.6 | 115.8 | 118.1 |
| Forward 0.25 m/s measured speed, m/s | 0.204 | 0.199 | 0.207 |
| Forward 0.25 m/s swing peaks L/R, mm | 16.1 / 18.4 | 17.6 / 20.8 | 5.8 / 7.1 |
| Left-turn swing peaks L/R, mm | 15.1 / 14.4 | 12.2 / 19.7 | 6.4 / 7.5 |
| Left-turn yaw rate, rad/s; command +0.2 | +0.158 | +0.148 | +0.189 |
| Right-turn yaw rate, rad/s; command -0.2 | -0.194 | -0.246 | -0.179 |

All listed checkpoints survived the five nominal cases. Survival alone misses
the substantial clearance regression at update 10000. No checkpoint meets the
combined upright/clearance/turning screen. The recovery curriculum increases
between updates 1000 and 8000, but this temporal association does not establish
that it caused the late regression. An isolated continuation comparison is
needed before changing the curriculum or attributing the failure to it.

## Additional validation and its limits

The original parent, update 5000 and update 2000 each survived all five cases
for each of two paired initial-tilt seeds (30 seconds per case), and all 48
finite-force push/pull cases (20 seconds). The tests do not establish hardware
robustness, and identical survival counts do not demonstrate an improvement
over the parent. Update 5000 retained approximately four degrees of backward
lean in both initial-tilt evaluations. At the 0.15 m/s command its measured
speed was approximately 0.122 m/s in those longer evaluations, versus about
0.154 m/s for the parent.

## Local preservation

All scheduled checkpoint evaluations and videos through update 10000, and all
nine follow-on evaluation directories, were copied into
`outputs/retrain-posture-width-20261117/evidence/`. Intermittent SSH timeouts
were recorded by the backup helpers; retries completed the scheduled copies.
Checkpoint hashes were checked again locally on November 18:

| Update | SHA256 |
| --- | --- |
| 2000 | `b1c19b28601d7c57ce32abcf1ef40882fd1c03d8b5f83dd2bf303f95a0c29bf5` |
| 5000 | `8351af3dfe72236de5592a6ecd97592d001beab331bf223cc153500ba04068ac` |
| 8000 | `c4658221a1ab9290f60eb5bb301a144ad03862188255a4183e8930996ba6c416` |
| 10000 | `a873967cfe266ff799a7244a2c953dc021df58550bc5ffb6afcbad5a2768fae6` |

Next research should preserve update 5000 as the rollback point and isolate
zero-speed upright posture and turning clearance, rather than continue the
degrading update-10000 trajectory without a hypothesis. No new training,
robot deployment, gain change or protection change is claimed by this report.
