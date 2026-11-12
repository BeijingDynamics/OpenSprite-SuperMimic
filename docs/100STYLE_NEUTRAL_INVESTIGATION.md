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
