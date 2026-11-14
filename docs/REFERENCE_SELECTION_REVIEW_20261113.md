# Reference selection after the mixed-bank failure

## Decision

The user reviewed all five Neutral source/reference videos and rejected the
walking quality, noting irregular motions and excessive lateral leg travel.
The standing reference may remain archived as a separate candidate; it is not
automatically admitted to the next run. Neutral walking and lateral clips are
excluded from the current training direction. Use PM01 as the walking-reference
baseline. Preserve the failed mixed-bank experiment, source QA, videos and
metrics unchanged as historical evidence.

This is a selection decision, not a claim that a replacement training job has
started or that the remaining PM01 clearance/zero-command issues are solved.
The PM01 A/B checkpoints remain the controls for further targeted work. Keep
published gains, action/history semantics and the full SuperMimic method.

## Re-screening evidence

The available Geno-converted `Neutral_FW.bvh` and `Neutral_TR1.bvh` were scanned
inside the official frame cuts, inset by one second at each boundary. The scan
used 3/4/6/8/12-second windows at 15-frame spacing (approximately 0.25 seconds).
It checked active forward travel, pelvis heading variation, turn rate, travel
alignment, ordinary arm/torso posture, inferred support-foot heading, bilateral
swing events, alternation and stride-period variability. These are explicit
research screening criteria, not evidence that the entire 100STYLE collection
is unsuitable. No new retargeted candidate was admitted.

For the forward sequence, only two overlapping three-second windows passed
the straight-motion criteria: source frames [507,687) and [522,702). The first
is the already-used short forward reference, not a new alternative. It has
only two/three detected left/right swing peaks, too few on one side to assess
stride-period variation reliably with this screen.

| FW window length | Scanned overlapping windows | Passing straight-motion screen | Minimum smoothed pelvis heading span among all windows |
| --- | --- | --- | --- |
| 3 s | 455 | 2 | 2.28 degrees |
| 4 s | 451 | 0 | 38.35 degrees |
| 6 s | 443 | 0 | 95.46 degrees |
| 8 s | 435 | 0 | 99.19 degrees |
| 12 s | 419 | 0 | 201.28 degrees |

The minimum-heading windows are not necessarily otherwise qualified. These
counts are overlapping windows, not independent examples. TR1 produced no
window passing the straight-motion screen. Several longer FW windows have
regular alternating steps but contain substantial turns; rhythmic stepping
alone does not make them a clean straight-walking anchor. Curved walking is
not inherently defective, but these sources did not supply a newly qualified
alternative to PM01 for the user's present goal.

The result is limited to these available converted source sequences and the
specified task. It does not rule out other originals, conversions or styles.
Source motion and robot retargeting remain separate possible contributors to
the observed irregular poses. Do not attribute all visible artifacts solely
to the human dataset without an aligned comparison.

## Evidence locations and next constraints

Local re-screen evidence: `outputs/retrain-20261113/neutral-straight-rescreen/`
contains `screening.json`, every scored window in `all-windows.json`, and the
screening script. The five paired source/robot videos remain under
`outputs/retrain-20261113/selected-neutral-review/`.

PM01 corrected walking bank SHA-256:
`7c6dccdc87756956f58ed9c5c6c7432882172abc62f116b803f719ecc16e6870`.
It contains approximately 11.42 seconds and about 14 reference swings per
foot. This is a small clean baseline, not proof of adequate reverse/lateral
coverage or broad robustness. Evaluate those commands and perturbations
explicitly. The learned world model uses simulator transition experience;
adding questionable reference motions merely to increase dataset duration is
not a prerequisite for training it.

Do not silently narrow lateral motion, splice short loops, change actuator
parameters or reintroduce rejected Neutral clips as part of the fallback.
Any such change would be a separate, explicitly evaluated experiment.
