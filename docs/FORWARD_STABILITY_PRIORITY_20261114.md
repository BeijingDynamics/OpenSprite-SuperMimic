# Forward locomotion and stability priorities

## User-directed scope

Prioritize reliable forward walking, usable left/right turning, stable stops
and recovery from pushes/pulls. Modest speed undershoot is acceptable while
gait quality and robustness improve. Reverse and lateral tracking are secondary
and do not disqualify the current forward research candidate by themselves.
The selected baseline remains PM01 checkpoint 7629, whose checkpoint hash is
recorded in `outputs/retrain-20261113/pm01-sole-resume/selected-candidate.json`.
This prioritization does not change hardware protections or establish hardware
qualification. No runtime change or motor operation is part of this decision.

Keep the previously reviewed Neutral lateral clip as a possible future source
of lateral stepping structure. Its lateral excursion was rejected as too large;
reusing it requires separate step-width/foot-placement retargeting and review.
It must not be silently restored to the primary PM01 training bank.

## Robustness evidence requires more than survival

The existing 7629 test suite has 48/48 surviving cases under three evaluation
seeds. However, its original instantaneous recovery criterion is not a clean
measure of return to ordinary walking for this candidate. At seed 101, even
the unperturbed 0.25 m/s case never maintains that criterion for a full second
after 8 seconds: the longest interval is 0.88 seconds. Its instantaneous
velocity-error 95th percentile is 0.1032 m/s, just above the 0.1 m/s criterion,
as normal stride oscillations cross the cutoff. Thus the reported 26/40 force
events recovering under that criterion cannot be read as 14 genuine balance
failures; equally, survival cannot be read as complete recovery.

Next robustness assessments should retain the original raw metrics and failure
flags and separately compare perturbed gait with matched unperturbed gait:
cycle-averaged velocity and heading, body tilt, bilateral swing continuity,
loaded slip, displacement and settling after release. Do not hide sustained
drift behind averaging or change hardware protection thresholds. Test sustained
forward walking, curves, stop/restart, graded perturbations and different
initial poses before claiming robust behavior.

## Time-reversed PM01 as a secondary reference candidate

Time reversal is mathematically straightforward for kinematic references. For
a clip of duration T, use p_rev(t)=p(T-t), R_rev(t)=R(T-t), and q_rev(t)=q(T-t).
Linear, angular and joint velocities change sign after reversal. Recompute
derived commands and validate finite-difference consistency and clip boundaries;
do not leave the original positive forward-velocity labels attached to reversed
poses. Robot heading is retained, so reversed root travel becomes backward
travel relative to the body. For playback-rate s, velocity also scales by s.

This produces a candidate backward reference, not proof of executable backward
gait. Toe-off and landing sequences reverse; friction and dissipative impacts
are not dynamically time reversible. Inspect contact/slip, clearance, joint
speed and torque demand in closed-loop simulation before any training admission.
MuJoCo's constraint/friction model is documented at
https://mujoco.readthedocs.io/en/stable/computation/index.html .

No reversed clip has been admitted or new reverse training started by this
note. It remains lower priority than forward stability, as the user requested.
