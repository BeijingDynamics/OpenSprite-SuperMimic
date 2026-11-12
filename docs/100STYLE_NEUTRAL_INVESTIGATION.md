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
