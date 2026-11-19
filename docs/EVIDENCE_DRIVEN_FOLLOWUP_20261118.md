# Evidence-driven follow-up to the three-arm comparison

The user requested that training completion lead to measured performance review,
reason analysis and active improvement rather than an idle GPU awaiting a chat.
A separate supervisor now waits for the complete original comparison. It does
not interrupt training, change the original arms or run concurrent GPU jobs.
This is a bounded additional experiment, not an autonomous proof of root cause.

## Decision rules

The supervisor reviews every completed checkpoint with the improvement-retention
screen. Parent eligibility requires survival, no crossed-foot samples, standing
pitch within 0.3 degree of upright-clearance600, and each case's P05 spacing
within 5 mm of that anchor, plus the existing bounded research gate. Ranking
among eligible parents uses the recorded failed preservation/quality checks
and posture metrics. A failed quality screen remains a failed screen.

For the upright and upright-clearance arms, the latest common update supplies
a diagnostic score for both low-speed foot peaks and right-turn right-foot
peak, each capped at the parent5000 target. A missing swing counts as zero.
This deliberately focuses on the unresolved failures; it is not an overall
gait-quality score. Full forward/turning quality is still checked separately.

If the selected parent already passes the combined quality and retention
screens, its recipe is replicated with a new seed. Otherwise, the follow-up
is a matched pair from that same preserved parent and seed 48: unchanged
objective versus one clearance-weight change. An existing positive clearance
weight is doubled, capped at 0.30, only if matched-stage evidence favors the
clearance arm by at least 0.05 on the diagnostic score; otherwise it is halved.
If the selected parent has no clearance term, a small 0.075 term is tested.
Pitch, spacing, physics, gains, reference and fixed recovery strength remain
unchanged within the pair. Both arms reset critics and optimizers consistently
through the explicit transfer path. This tests a next hypothesis, not a claim
that coefficient tuning necessarily resolves the failure.

Fresh updates 600 and 1600 receive physical evaluation, the original quality
screen, the improvement-retention screen and video. Failed research gates stop
an arm. No parent meeting the preserved-posture/spacing requirements means no
automatic training launch. Failure of the original comparison also blocks the
follow-up rather than silently changing evidence. The queue ends by November
19 at 14:00 China time, with a local copier through 14:55. It requires at least
three hours remaining before starting the paired stage. There is no automatic
hardware deployment or cloud lease renewal.

Decision inputs, chosen hypothesis, parent hashes, exact configurations, model
checkpoints, metrics and videos are preserved locally in
`outputs/evidence-followup-20261118/`. Syntax checks and a decision dry run on
partial saved results passed; the supervisor does not act on that dry run and
waits for the complete comparison before deciding again.

## Newly observed upright-only result

At fresh update 600, upright-only has zero-command pitch -2.86 degrees, forward
0.25 m/s swing peaks 11.04/13.60 mm, low-speed peaks 7.62/5.47 mm, left-turn
peaks 8.73/10.83 mm and right-turn peaks 9.49/10.37 mm. Upright-clearance600 has
better standing posture, forward clearance and left-turn clearance, while
both arms have unresolved low-speed and right-foot issues. The unchanged
reward control is now training. These early matched observations rule out
attributing all clearance regression solely to the presence of the new
clearance term; they do not yet isolate the standing objective, continued
training, world-model error or shared-controller adaptation as the cause.

## November 18 priority update: standing legs and low-speed alternation

The user clarified that the desired stance should have more extended legs,
0.15 m/s walking must alternate reliably, useful right turning should be
retained rather than optimized for exactly equal foot peaks, and arm movement
should eventually look less rigid. Torso pitch alone does not measure these
goals. The existing three arms remain unchanged.

At upright-clearance600, zero-command left/right knee joint readings are about
+37.7/-14.3 degrees; at update1600 they are +38.6/-15.3 degrees. The knee axes
have opposite signs and both MJCF reference positions are zero. These are named
joint readings, not an externally measured anatomical knee angle. Median root
height drops from 0.454 to 0.452 m while torso pitch remains around -2 degrees.
This is a crouched, asymmetric stationary configuration despite improved torso
uprightness. The selection criteria must not treat pitch improvement alone as
the requested standing improvement.

The current reference's shoulder-pitch P05-to-P95 spans are approximately
8.5/4.3 degrees. At upright-clearance600 low-speed walking they fall to about
1.5/1.7 degrees. Thus modest source arm swing and additional low-speed policy
suppression both contribute; no runtime arm gain or artificial arm trajectory
is introduced to hide it.

At update1600, the added motion-interface test survived and achieved median
left/right swing peaks of 24.54/24.56 mm. The same model's velocity interface
at 0.15 m/s achieved 9.94/8.87 mm with 26/26 qualifying lift intervals. This
shows that the controller can physically track the existing higher-clearance
reference, while velocity-conditioned behavior still loses clearance. It also
shows recovery from the earlier 7/0 event result. Initial conditions differ
between interfaces, so this is not a complete causal identification.

The idle follow-up supervisor was refreshed to prioritize parents with retained
low-speed step counts before ranking clearance deficits and standing knee
flexion. It still requires the established torso/spacing preservation gate.
If low-speed alternation has recovered but the maximum median standing knee
reading exceeds 25 degrees, the paired follow-up tests one additional
zero-command knee cost instead of automatically changing the clearance weight.

The separately versioned `sprite_velocity_standing_knee_v3` term is a soft
penalty on flexion magnitude beyond a 20-degree deadband, normalized by
20 degrees and weighted 0.2. It has no incentive to force knees to zero or to
extend them beyond that deadband. A smooth command gate decays with planar
speed scale 0.03 m/s and yaw-command scale 0.05 rad/s; at 0.15 m/s its multiplier
is below 1e-9. This confines direct shaping to near-zero commands. Walking may
still change indirectly through shared parameters and must pass physical
regression checks. Existing posture, width, clearance, recovery, gains and
protections are preserved within the matched pair.

Thirty-one reward/contract tests passed, including gradient direction, mirrored
joint signs, negligible walking/turning gate, and exact prior-reward equality
when the added weight is zero. Before the treatment's long training, the
supervisor runs a separate three-update GPU smoke with actor updates enabled;
that smoke checkpoint is never promoted or used to resume production training.
The revised decision was dry-run on available partial results, selecting the
1600-update parent and the standing-knee hypothesis. The actual choice is
recomputed only when the complete original comparison is available. Current
GPU training was not interrupted by refreshing the idle supervisor.

## First completed matched comparison

The supervisor selected `eval-upright-clearance-1600` after the original three
arms completed 3000 updates. It did not resume from the low-speed-collapsed
3000-update checkpoint. Both new arms completed 600 updates and nominal
evaluation; the standing-cost smoke also completed. At approximately 18:47
China time, the matched control was at update 1320 of its 1600-update stage.
The treatment's 1600 stage follows serially. Neither 600-update result passes
the full quality and preservation screen.

| Metric | Matched continuation, 600 | Standing-knee cost, 600 |
| --- | ---: | ---: |
| Standing knee flexion magnitude L/R, degrees | 35.2 / 13.7 | 31.5 / 13.6 |
| Standing backward torso pitch, degrees | 1.79 | 1.79 |
| Low-speed mean forward velocity, m/s | 0.155 | 0.148 |
| Low-speed lift events L/R | 20 / 21 | 24 / 17 |
| Low-speed median swing peaks L/R, mm | 16.47 / 11.18 | 15.39 / 8.21 |
| Forward0.25 median swing peaks L/R, mm | 17.63 / 16.01 | 17.00 / 18.55 |
| Right-turn mean yaw rate, rad/s (command -0.2) | -0.179 | -0.170 |
| Right-turn median swing peaks L/R, mm | 14.77 / 13.56 | 14.68 / 13.67 |
| Right-turn P05 foot-center spacing, mm | 110.2 | 124.9 |

The new term reduces left-knee flexion by approximately 3.7 degrees relative
to its matched continuation, with similar torso pitch. It also widens standing
P05 foot-center spacing from 131.4 to 153.5 mm. This is an early single-seed
result, not proof of a natural standing posture. Low-speed right-foot lift
weakens relative to the matched control and turn-rate tracking is slower;
zero-command gating did not prevent indirect effects on walking. Continue the
scheduled matched 1600 comparison before promoting this recipe. Gain,
action-scale, model asset and robot protection settings remain unchanged.

Both 600 checkpoints and their numerical/video evidence have been copied
locally with checkpoint hash verification. The cross-experiment numerical
record is `three-arms-3000-and-standing600-audit.json` under the original
comparison's local evidence directory.
