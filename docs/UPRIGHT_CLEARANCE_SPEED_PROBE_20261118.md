# Upright-clearance command-speed probe

The user reprioritized teleoperated walking: poor stepping at 0.15 m/s is not
by itself a reason to reject a model if a useful higher command produces
stable walking. Standing posture, usable forward motion, foot clearance,
turning and spacing remain important. Historical low-speed scores and failed
screens remain unchanged as evidence; future selection should reflect this
priority rather than treating the old low-speed screen as the sole veto.

The requested test uses the upright-plus-clearance checkpoint at update 3000,
SHA-256 `884946b4f03a03d53057208bfdafe3b0c116f91fec06c493ec7307a49a9c5e4c`.
Its existing asset, gains, action scale, history, PD rate and policy rate are
unchanged. This is physical simulation only and does not access the robot.

Four parallel nominal cases use forward commands 0, 0.25, 0.5 and 1.0 m/s,
with zero lateral and yaw commands. Each starts from the same quiet reference
pose at physical rest. The first two seconds use zero command; the requested
command is then held for twenty seconds without a command ramp. No failure
reset is allowed. Summary metrics exclude the first three seconds and stop
at the first failure. Rendered physics after a failure remains marked failed.

The trained forward-command range is -0.3 to 0.6 m/s. Thus 0.5 m/s tests the
trained range while 1.0 m/s is an explicit extrapolation probe. Requested
speed must not be reported as achieved speed. Nominal survival is not evidence
of robustness to disturbances or qualification for powered hardware use.

The runner is retained in the isolated experiment source as
`scripts/run_requested_speed_probe.py`. Results are stored remotely under
`outputs/upright-clearance-speed-probe-20261118` and copied to the matching
local experiment directory. The existing paired standing-cost training is
left unchanged so its matched comparison remains interpretable.

## Results

All four cases completed without a failure in the 22-second nominal rollout.
The target was active for 20 seconds; the following metrics use seconds 3-22.

| Forward command, m/s | Measured forward speed, m/s | Median swing peak L/R, mm | Lift events L/R | Median torso pitch, degrees | P05 foot-center spacing, mm |
| --- | ---: | ---: | ---: | ---: | ---: |
| 0.25 | 0.208 | 18.24 / 16.86 | 25 / 25 | +0.54 | 116.1 |
| 0.50 | 0.324 | 19.80 / 15.05 | 29 / 28 | +3.18 | 115.8 |
| 1.00 | 0.324 | 19.04 / 16.75 | 30 / 30 | +6.11 | 110.1 |

Positive pitch denotes forward lean. Lift events use the existing threshold
of greater than 5 mm for at least two 50 Hz samples. No crossed-foot samples
were observed in these nominal cases. At 0.5 m/s command, bilateral stepping
is sustained and measured speed rises relative to the 0.25 control, but the
right-foot peak remains below the left. Doubling the command to 1.0 m/s does
not increase measured speed in this test; it increases forward lean and yaw
rate variation (mean absolute zero-yaw error 0.094 versus 0.061 rad/s). This
does not establish the mechanism of the apparent response plateau.

These results support evaluating this candidate at a useful teleoperation
command rather than rejecting it solely for the 0.15 m/s dead zone. They do
not solve standing knee flexion or establish disturbance robustness. There
is no evidence here that commanding 1.0 m/s improves practical locomotion
over commanding 0.5 m/s.

Video panels are zero / 0.25 m/s on the top row and 0.5 / 1.0 m/s on the
bottom row. The video is 960x720, 1100 frames at 50 fps, 22 seconds at original
speed. Local video and rollout SHA-256 values were checked against the remote
artifacts and recorded in `local-verification.json`; all evidence is saved
under the local `outputs/upright-clearance-speed-probe-20261118` directory.
