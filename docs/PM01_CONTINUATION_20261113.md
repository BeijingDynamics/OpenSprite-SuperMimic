# PM01-only bounded continuation

## Completed review on November 14

Training completed update 9000. Standard and supplementary audits completed
at updates 6424, 7629 and 9000. No trainer remains active. Select 7629 as the
current research baseline; reject the latest checkpoint for command-control
regression. This selection is not hardware admission.

At 7629, all 48 cases survived under each of three evaluation seeds. The
seed-101 0.25 m/s command produced 0.19585 m/s mean forward speed. Left/right
swing-event peak medians were 20.24/18.91 mm, with 18/19 events over the scored
17 seconds. Their onset ordering alternated on every transition. Mean stride
periods were 0.928/0.929 s, with coefficients of variation 2.34%/1.93%.
Loaded near-floor corner RMS slip was 0.0184/0.0120 m/s. Loaded foot headings
relative to the pelvis were +3.87/-1.64 degrees; torso heading was -0.19 degrees.
These are recorded simulation measurements, not visual estimates or hardware
measurements.

The 60-second zero-command test settled, with mean horizontal speed error
0.000534 m/s and near-zero terminal velocity. The start/stop test also settled.
Limitations remain: forward speed undershoots; the -0.15 m/s reverse command
produces about -0.070 m/s; +/-0.3 rad/s turn commands produce about +/-0.234
rad/s; lateral commands do not produce reliable sustained lateral walking.

At 9000, the zero-command 60-second mean speed error regressed to 0.223 m/s;
reverse can produce forward motion, and survival is 47/48, 47/48 and 46/48.
The decline has not been causally isolated; additional iterations alone are
not a successful fix. Keep the 6424/7629/9000 audits for comparison.

The selected checkpoint SHA-256 is
`eaaba88997aa870dd95df92e12cbe684f877bbcf11790f86d70283d38290f8dc`.
Its local path and selection rationale are recorded in
`outputs/retrain-20261113/pm01-sole-resume/selected-candidate.json`.
The hourly backup had not saved the final checkpoint locally when checked on
November 14. A one-shot recovery fetched the remaining 42 files (575956645
compressed bytes), and both selected/final checkpoint hashes were checked
against their evaluation records. The new bright command-review video uses
the actual 7629 rollout, with sampled frames visually inspected.

## Actual state

On November 13 at 17:58 China time, verification found the GPU idle. Returning
to PM01 had previously reached only the reference-selection decision and
documentation; no new training had been launched. That execution gap was
reported to the user explicitly.

Training actually resumed at 18:00:14 China time, from PM01 candidate B update
6030, and optimizer updates through 6080 were verified. The isolated source
root is `/root/gpufree-data/supermimic_pm01_resume_20261113`; output is
`outputs/pm01-sole-resume-to9000`. Launch PIDs: trainer 309780, standard watcher
309781, supplementary gait watcher 310150.

## Controlled continuation

The parent is
`/root/gpufree-data/supermimic_sole_experiment/outputs/pm01-clearance-sole-height-20261112-125806/training/model_006030.pt`.
Its forward swing-event medians were approximately 16/17 mm, but zero-command
lateral drift remained approximately 0.076 m/s. It is a research candidate,
not hardware qualified.

This continuation preserves the exact corrected PM01 reference, full joint
SuperMimic method, published Kp/Kd, 500 Hz PD / 50 Hz policy, action scale,
physical model and command sampling. It also retains B's existing bounded
motion-interface sole-height reward extension (weight 0.25, standard deviation
0.015 m); this is explicitly an extension, not the unmodified paper reward.
No Neutral data or externally inferred motor parameters were introduced.

Only the update budget, cumulative wall-time allowance and descriptive
metadata differ from the parent configuration. The run stops at total update
9000, adding at most 2970 updates. The active perturbation curriculum continues
to its existing full-strength iteration 8000; it is not newly changed here.
Network, world-model, optimizer and sampler state are restored. Replay, RNG
and live simulation state are rebuilt, so this is not a bitwise uninterrupted
continuation. Resume contract checks passed in the running trainer.

The frozen parent source and all copied assets were hash-checked. Evaluation
overlays add the milestone watcher, checkpoint-selection helper and physical
geometry summarizer; trainer physics and reward code are unchanged.

## Evaluation and preservation

The standard watcher runs three seeds of the 48-case command/disturbance
evaluation, rendered videos, motion tracking and sole audits. It evaluates
saved intermediate checkpoints and explicitly the final update 9000.
The supplementary watcher adds bilateral swing events, loaded slip, toe/torso
heading and 60-second zero/start-stop tests for reviewed checkpoints.

The primary question is whether additional training preserves bilateral
clearance while improving zero-command stability. More updates are not
assumed to fix drift. Inspect the physical metrics before any further extension;
do not replace this assessment with reward or survival alone.

At the observed approximately 0.70 updates/s, training is expected to finish
around 19:15 China time, with final audits roughly 19:30--20:00; these are
estimates. No new physical performance result was available at update 6080.

A bounded local backup worker stores results under
`outputs/retrain-20261113/pm01-sole-resume` and remains active through the gait
watcher. Its initial backup completed. Parent checkpoints, the failed mixed
experiment, and source-selection evidence remain preserved separately.
