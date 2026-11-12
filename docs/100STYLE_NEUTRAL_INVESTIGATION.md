# 100STYLE Neutral locomotion candidate

The objective is natural command-controlled Sprite walking, including stops,
turns and recovery. Neutral is investigated as a complementary locomotion
library, not as a reason to add all 100 performance styles or replace the current
PM01 controlled experiment mid-run.

## Sources and reproducibility

- Author page: https://www.ianxmason.com/100style/
- Original archive and trim metadata: https://zenodo.org/records/8127870
- Author-linked skeleton conversion: https://github.com/orangeduck/100style-retarget
- Data license: CC BY 4.0; preserve Ian Mason's credit and Daniel Holden's conversion attribution.
- Geno mesh: separately restricted to non-commercial research.

The original data is approximately 60 Hz and contains forward/backward walking
and running, side steps, idle, and transitions. It was captured from one actor;
many frames do not imply many independent performers. The original XSens rig
and the Geno-converted rig have different hierarchies. Only the latter is passed
to the existing Geno surface-correspondence pipeline, after exact skeleton and
rest-offset validation.

```bash
python scripts/download_100style_neutral.py \
  --output data/sources/100style-neutral-geno
python scripts/screen_100style_neutral.py \
  --source-dir data/sources/100style-neutral-geno \
  --output outputs/neutral-source-screen
```

Place the author's `Frame_Cuts.csv` beside the downloaded files before screening.
It is also inside the original Zenodo ZIP. Google Drive was unreachable from the
rented host, so the investigation retrieved this small metadata file from Zenodo
and used byte ranges to download only Neutral entries from the Geno archive.

Trims use the official boundaries with one-second inward margins to avoid
index-base ambiguity. Further crops preserve original sample order and timing.
Do not append overlapping windows as independent examples or count mirrored
clips as independent performers. Reserve disjoint whole clips for validation
before constructing a training bank.

## Initial source findings

The first forward-walk file contains 7,896 frames at 59.9988 Hz. The official
trim is frames 327 to 7,445. It includes frequent room turns; it is not two
minutes of straight walking. The screened interior has median human speed
0.729 m/s, low torso tilt and hands beside the torso. Robot speed must be
recomputed after embodiment scaling, not copied from this human value.

An initial all-frame foot-yaw check rejected every long window. Inspection
showed that toe-off can make the ankle-to-toe horizontal projection nearly
zero, so its yaw is ill-conditioned. The revised screening keeps the 30-degree
90th-percentile heading criterion but applies it to inferred support samples.
It also reports all-frame yaw separately. This is a kinematic proxy, not measured
contact force. Source swing landmark height is not robot sole clearance.

The first pass selected two 20-second natural paths, one 3-second straight
segment and one 3-second turn. These are isolated retarget candidates, not an
accepted library. In particular, short straight segments alone would be an
inadequate replacement for a diverse command-control library.

## Admission criteria

Native UMR is followed by source-contact and torso-aware trajectory checks on
the unchanged v5 geometry. Inspect support slip, actual toe/heel clearance,
foot headings, bilateral step timing, torso and arm posture, joint bounds,
velocity, root acceleration and collisions. Keep native outputs and failed QA
records; do not hide a failed candidate through clipping or relaxed thresholds.
Review front and side videos before accepting a reference.

Reference commands must be derived from the final robot trajectory. The natural
paths include turns faster than the present random-command range; report this
coverage explicitly rather than silently clamping labels or changing the active
PM01 experiment. Full joint SuperMimic and runtime safeguards remain unchanged.

## November 12 retargeting review

All eight Neutral files were retrieved and CRC-checked without an account or
gated access. The complete first screening selected three non-overlapping
20-second natural paths, two six-second idle segments, and a three-second
forward segment. A separate earlier three-second turn is also retained for
research. One idle segment is reserved for validation. These are source
candidates, not an accepted robot-motion bank.

The default contact refiner passed coarse geometry checks on the short forward
clip but introduced excessive root acceleration and rapid knee motion. The
isolated `refine_neutral_contacts.py` investigation keeps the native output and
uses source-bounded root translation, calibrated source pelvis and torso
orientation, a 3 m/s^2 root acceleration bound, and the natural knee-flexion
branch. It preserves measured support headings and interpolates heading through
the ill-conditioned human toe-off projection. Complete rigid-sole swings use
source timing, support anchors and peak height with a smooth trajectory; this
is an explicitly engineered retargeting step, not unmodified motion capture.

The seventh short-clip candidate has approximately 32.5/41.3 mm peak left/right
sole clearance, two/three sustained events above 5 mm, 0.80 mm maximum inferred
support-target residual, and 0.038 m/s near-floor center-slip proxy. Its maximum
joint-speed ratio remains approximately 0.956, so a numerical pass is not a
claim of generous actuator margin or dynamic feasibility. Front/side sampled
frames show alternating steps and arms beside the torso. Long-path, turn and
idle processing remains separate; the active PM01 training is unchanged.

Failed candidates remain evidence: one long native path failed the unchanged
human/native alignment check at approximately 0.299 m RMS; an earlier smooth
swing candidate slid near the floor and was rejected. No existing training
quality gate or runtime protection was relaxed to admit these candidates.

Local research artifacts live under `outputs/retrain-20261112/`; public source
BVH files remain on the rented host and are reproducible with the downloader.
Derived motion remains outside Git. Candidate reports preserve source, native
input and refiner hashes. No candidate is approved for hardware use.

### Upper-body audit supersedes the preliminary visual impression

The seventh candidate must not be admitted based on its foot-only numerical
pass. Direct comparison with source shoulder-elbow and elbow-wrist directions
found large arm errors: the short clip reached approximately 58 degrees, and
the long path approximately 81 degrees. Native surface retargeting plus a
corrected pelvis/torso did not preserve the human's arm style sufficiently.

The ninth candidate therefore adds source unit-limb-direction tasks and source
head orientation. Limb directions avoid imposing human bone lengths on Sprite.
The direction Jacobian is checked against finite differences; all five focused
preprocessing tests pass. New candidate admission also requires arm/head
95th-percentile errors below 10 degrees and maxima below 20 degrees. Existing
foot, joint, root and collision checks remain in place. The first short result
has arm error 6.24 degrees at the 95th percentile and 7.98 degrees maximum,
with head error below 0.08 degrees. Long-path review remains separate.

The two rejected long native paths are also being investigated as independent
five-second solves to reduce sequential native-root drift. This preserves source
time and ordering; it does not synthesize new motion. Only child clips, never a
duplicate parent, may enter a bank. All children of source frames 1587..2787
are held out together; children of 3507..4707 are training candidates. A split
does not turn one actor into independent subjects or remove the need for QA.

### Verified source frame and retiming before optimization

The tenth candidate replaces alignment inferred from the optimized native robot
root with the actual, known Geno source transform. The native retargeter maps
Y-up coordinates to Z-up as `(x, -z, y)`. `audit_geno_source_frame.py` compares
every source frame against the native FBX bone-transform path before robot IK.
It checks root translation and orientation and reports the small non-root bind
offset differences. A drifting surface-IK robot root is not a calibration
reference. The previous best-fit RMS is retained as a diagnostic, but it is no
longer an admission gate for this independently verified coordinate transform.
This deliberately corrects a data-calibration heuristic; robot/control limits
are unchanged.

Time scaling is explicit and happens before constrained trajectory optimization.
Current candidates use 0.75 source speed for forward/backward paths, 0.5 for
sideways paths, and unchanged timing for idle. Source contact phases, pelvis,
torso, head and arm tasks are resampled together. Commands must be recomputed
from the final robot trajectory. Six focused preprocessing tests pass, including
source-task/contact preservation under time scaling.

A post-hoc 0.75-speed resampling experiment failed the unchanged 2 mm support
target check at 2.30 mm and remains rejected. Slower interpolation does not
automatically preserve nonlinear foot geometry. The first pre-optimization
retimed forward candidate passes with 1.51 mm maximum support residual and
6.42/7.95 degree arm direction p95/maximum errors. This is still a reference
candidate, not evidence of learned policy performance. Other command categories
remain under independent numerical and visual review.

### Root-height diagnosis and the eleventh candidate

The backward candidate's initial QP certificate identified a geometric
initialization problem: the native surface solve's median pelvis height was
about 0.405 m. Applying the standard initial leg pose at that height placed
some sole corners about 78 mm below the floor. The optimizer's local step and
bounded root displacement could not remove that penetration in one step.
Increasing motor limits or accepting penetrating output would not fix this
source conversion error.

Root height now comes from source hip-center height above the inferred support
ankle, scaled by the robot/human leg-length ratio, plus the robot's actual
ankle-to-sole height and minus its rotated pelvis-to-hip offset. Native root
height is retained only as a diagnostic. Source vertical motion is preserved;
the final root remains within the existing 20/20/40 mm XYZ displacement bounds
around this source-derived prior. Seven focused tests pass, including vertical
translation invariance and reconstruction of the robot hip height.

The backward solve now converges geometrically, but the short candidate still
fails the required repeated bilateral lift check. It remains rejected. A
rightward candidate passes the numerical gates, while a leftward candidate
fails support residual and arm-direction checks. A numerically successful
source-height correction therefore does not automatically admit an entire
motion family.

Longer backward research candidates explicitly include measured heading
changes rather than claiming straight, constant-heading walking. They retain
the torso, hand and stance-foot style checks, preserve held-out intervals, and
use declared 0.5 source timing before optimization. One native conversion was
rejected because it required 35 mm ground correction, above the existing 30 mm
cap. Failed conversion and refinement records are retained.
