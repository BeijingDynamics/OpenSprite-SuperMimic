# Reference clearance and interface diagnostic

The user's concern that the initial PM01 motion itself has low foot clearance
is supported by the preserved v5 reference. This should be distinguished from
the corrected reference actually used by the current training.

| Reference | Duration | Maximum lowest-sole height L/R |
| --- | ---: | ---: |
| Initial PM01/G71 v5 baseline | 11.98 s | 14.99 / 13.91 mm |
| Current corrected PM01 reference | 11.42 s | 24.79 / 25.31 mm |

The corrected reference has 14 qualifying lift intervals per foot, median
per-step peaks 24.73 / 24.83 mm and median duration above 5 mm of 0.26 / 0.22 s.
It is therefore close in peak height but not perfectly equal in swing duration.
Its median instantaneous root planar speed is 0.227 m/s; this is not an
independent low-speed demonstration. The reference is one steady straight-walk
clip, not a library with independent starts/stops or left/right turns.

The initial baseline has a nonzero geometric sole floor offset (minimum
3.64 / 4.63 mm). Its threshold-based airborne counts and durations should not
be interpreted as measured contacts or directly compared with corrected
support phases. The corrected minimum sole heights are approximately 0.18 mm.
Joint names were remapped explicitly and measurements recomputed against the
verified local training XML. Source hashes and detailed measurements are in
`outputs/targeted-upright-clearance-20261118/evidence/reference-clearance-audit.json`.

The current velocity controller can lose much more clearance than this
reference difference: upright-clearance600 low-speed right-foot maximum is
4.82 mm, with no qualifying event above 5 mm. Thus original low clearance and
limited motion coverage are plausible contributors, but do not establish that
the current failure is solely due to an insufficient reference peak or that
the problem has no solution. Reference height is not a hard policy output cap.

The code's velocity gradients do not update the shared action decoder; that
decoder is trained through motion tracking. Limited demonstration coverage
therefore matters to its learned action repertoire. However, actual learned
tracking fidelity must be measured before deciding to change the data again.

## Additional diagnostic

At the upcoming 1600/3000-update physical evaluation boundaries, an additional
motion-interface rollout is now serialized before the next GPU training job.
It uses the exact same checkpoint and the unchanged reference, and records
actual physical motion-tracking foot clearance. A report then compares that
interface with velocity-conditioned behavior. If motion tracking retains
clearance but velocity control loses it, attention shifts toward velocity
conditioning, learned latent choices and task tradeoffs. If motion tracking
also fails to retain clearance, investigate imitation accuracy, model error,
decoder limitations and reference dynamic feasibility. This comparison narrows
the problem; it does not uniquely identify one of those mechanisms.

The new diagnostic does not change data, optimization, gains or protections.
It begins at the reference state and lasts one clip, whereas velocity tests
begin from rest, so durations and initial conditions are explicitly reported.
It has a 240-second timeout; diagnostic failure is recorded without silently
turning into a passing result or changing the original experiment. Evaluation
directories, including the extra rollout and report, are copied by the existing
local backup after the stage completes.
