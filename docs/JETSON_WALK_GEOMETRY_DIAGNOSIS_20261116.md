# Narrow steps and upper-body drift: two powered Xbox trials

## Scope and evidence

The operator reported intermittent foot contact and deteriorating upper-body
posture after two 30-second Xbox windows. The operator confirmed that the sling
remained slack and nobody pushed or supported the robot during the second run.
This investigation used existing logs only; it changed no runtime parameters,
gains, protection limits, or motor state.

Local evidence is under `outputs/deploy-20261116/live-evidence/`
`supermimic-pm01-xbox30-1116/`:

- `xbox-powered-20261116-152917-5629`
- `xbox-powered-20261116-153129-5687`

Both runs recorded all 31 motors disabled at completion and no CAN errors.
The second run failed the final zero-command body-settling assessment:
last-second pelvis tilt peaked at 11.956 degrees and angular speed at
1.825 rad/s. It had already passed initial admission and completed the Xbox
window; the generic failure text mentioning startup is misleading here.

Feedback-to-target maximum ages were 14.301 and 14.295 ms; inference maxima
were 8.191 and 7.802 ms. Neither run hit the 20 ms deadline.

## Method and limitations

Run `python scripts/diagnose_xbox_walk_geometry.py`. Results and the figure are
written to `outputs/deploy-20261116/walking-diagnosis/`.

The script checks the MJCF hash against the deployed contract and verifies
joint order. It reconstructs poses from recorded joint feedback and projected
gravity. This is forward kinematics, not a dynamics replay. Global translation,
actual walking speed, ground reaction forces, and physical foot contact are
not measured by this procedure.

Foot width means signed horizontal lateral separation of the sole-box centers
in the pelvis heading frame, not clearance between their edges. The training
sole boxes measure 104 x 44 x 10 mm. A 15-axis oriented-box separation test
accounts for foot yaw, fore/aft separation, and swing height. A negative margin
means overlap of these modeled boxes; calibration and real sole geometry still
limit any conclusion about physical contact. Positive margins are lower bounds
on geometric distance. Policy-target FK uses the pre-protection joint targets
with the current measured root orientation; it is not an executed pose.

## Foot spacing

Statistics below select 20–50 seconds with forward command above 0.15 m/s.

| Quantity | Run 1 | Run 2 |
| --- | ---: | ---: |
| Feedback FK center width, median | 68.5 mm | 93.2 mm |
| Policy-target FK center width, median | 112.2 mm | 127.8 mm |
| Feedback FK center width, minimum | 36.5 mm | 34.3 mm |
| Modeled sole-box overlap samples, 50 Hz | 38 | 12 |
| Minimum 3D separating-axis margin | -10.3 mm | -7.7 mm |
| Left hip roll mean target minus feedback | +2.09 deg | +1.94 deg |
| Right hip roll mean target minus feedback | -2.34 deg | -1.86 deg |

Replacing only the two hip-roll feedback angles with simultaneous policy targets
in the geometry calculation increases width by a median 31.8 and 26.6 mm,
respectively. This localizes much of the geometric discrepancy to hip roll.
It does not prove that instantaneous perfect tracking is physically achievable,
or that increasing gains is the appropriate remedy. PD position error under
load is expected; the unresolved question is whether its magnitude and dynamics
match training.

The saved same-checkpoint 0.25 m/s simulation has median center width 89.3 mm
and minimum sole-box separation margin about 30.0 mm after the initial three
seconds. This is a different command schedule and initial condition, not a
matched replay. It indicates a relatively narrow nominal gait with a margin
that can be consumed by real tracking differences.

## Upper-body posture

In run 2, early control (22–32 s) versus late control (40–50 s) gives:

| Median quantity | Early | Late |
| --- | ---: | ---: |
| Reconstructed torso roll | -1.42 deg | -6.19 deg |
| Measured waist roll | -0.97 deg | -5.06 deg |
| Policy waist-roll target | -0.21 deg | -2.30 deg |

The policy target itself drifts, and execution adds further deviation. This is
not solely a fixed neutral-pose offset. Late waist yaw remains approximately
-0.58 degrees, unlike the large yaw rotation seen in earlier experiments.
The logs alone do not establish whether narrow steps cause the posture drift
or both arise from the same model mismatch.

Waist roll has no command-saturation events in either run. In run 2 its peak
command torque is 17.39 Nm and peak feedback torque 12.22 Nm, far below the
87.3 Nm limit. Increasing that protection limit has no evidential basis.

Both hip-roll feedback peaks in run 2 approach the 27.993 Nm feedback cap
(27.959 and 27.932 Nm). Their command-saturation counts are only 8 and 5,
respectively, across the entire run. These peaks deserve attention but do not
establish sustained clipping as the cause. Aggregate torque logs do not provide
the temporal alignment needed to link each peak to each narrow step.
Maximum recorded MOS temperature across the motors is 39 C; these logs do not
suggest thermal limiting. Battery-voltage effects are not established.

## Next investigation

1. Compare measured hip-roll and waist-roll tracking errors with matched
   closed-loop simulation using the recorded command schedule and initial pose.
   Compare loaded PD error, delay, motor gain encoding, and parallel-ankle
   transformation before changing any gains.
2. Reproduce the measured narrowing and drift with bounded model variations.
   An open-loop action replay or target FK alone cannot validate the cause.
3. If training changes are warranted, evaluate minimum foot clearance and
   separation together with sustained torso posture across at least the real
   30-second command window, stop/resume, turns, and perturbations. Preserve the
   full SuperMimic architecture and verify reference motions visually.

Do not compensate with arbitrary runtime hip offsets, added waist torque, or
weaker protection. Keep this checkpoint and the two original trials as baseline
evidence. No additional powered trial was performed during this diagnosis.

## Gain and rate audit

All 31 `kp_max`/`kd_max` pairs in run 2's `joints.tsv` match the exported
checkpoint contract's `stiffness`/`damping` arrays exactly (maximum absolute
difference zero). Direct-start control also rejects policy packets that deviate
from these configured gains after initialization. These are nominal joint-space
gains; motor mapping, protocol quantization and protective target saturation
remain separate from that equality.

The actual export contract specifies `policy_dt=0.020`, `physics_dt=0.002`,
and `decimation=10`. The copied upstream control-profile provenance file still
contains its original 200 Hz simulation timing; it must not be used to infer
this SuperMimic run's frequency. SuperMimic explicitly overrides that timing.

Training refreshes policy targets at 50 Hz and evaluates PD every 2 ms. In the
runtime, direct motors receive position/gain commands at 50 Hz and close PD
inside their own controller between packets. Parallel ankles instead compute
joint PD and map its torque into the two motor commands at 500 Hz. Their
motor packet gains are zero because the PD is external; this is not an extra
feedforward added to the policy's PD law.

Do not approximate 50 Hz direct-motor packets by holding PD torque constant for
20 ms in simulation: motor-local feedback continues between packets. However,
the exported contract explicitly says physical ankle differential modeling is
disabled. Training's serial-joint PD and stochastic command delay do not model
the complete runtime's feedback age, inference/transport latency, motor-space
limits, and parallel transmission. Those are the relevant execution differences
to audit in the next matched closed-loop replay, keeping physics PD at 500 Hz.

## Follow-up: internal PD rate and world-model sensitivity

The operator reports a 1000 Hz Damiao current loop. Treat a 1000 Hz MIT PD
update as a hypothesis until its firmware-specific rate is established:
current-loop sampling, current bandwidth, and the MIT impedance calculation
are distinct quantities. The manufacturer's 2025 product guide describes MIT
torque calculation feeding a current loop, but does not establish this robot's
firmware-specific PD update frequency. No motor register was changed or queried
on hardware during this investigation.

The local `VelocityLearner._velocity_step` uses `wm.step(state, action, xi_n)`
inside actor imagination, and `joint_policy_update` backpropagates that objective.
Thus learned-model prediction and derivative errors can affect training. This
is separate from simulator-to-hardware error; the deployed actor does not run
online world-model planning. Better stability of the previous AMP policy is a
valuable baseline, but is not a controlled comparison of learning algorithms.

The proposed rate diagnostic must separate numerical integration from PD rate:

| Variant | Physics step | Direct PD | Ankle external PD | Policy |
| --- | --- | --- | --- | --- |
| Existing baseline | 2 ms | 500 Hz | 500 Hz serial proxy | 50 Hz |
| Integration control | 1 ms | 500 Hz, torque held between updates | 500 Hz | 50 Hz |
| Internal-PD hypothesis | 1 ms | 1000 Hz | 500 Hz | 50 Hz |

The latter two cases must preserve command holds, inference/transport delay,
physical lag time constants, and randomization durations in seconds. Otherwise
changing the physics step also changes actuator delay and confounds the result.
Reproduce both real command schedules and extend stop/resume tests to expose
late drift. These experiments are planned, not completed results.

For torso drift, independently audit (1) pelvis IMU attitude and raw sensor
consistency under acceleration, (2) waist target versus feedback and torque,
(3) whether drift follows turn commands, narrow steps, or persistent load, and
(4) the same closed-loop scenarios in the simulator and learned world model.
Sensor consistency is not an independent ground-truth attitude measurement;
accelerometer tilt during walking must not be treated as gravity alone.
The late waist target/feedback gap of about 2.8 degrees corresponds to roughly
2.9 Nm of proportional effort at Kp=60 before damping. This is an order-of-
magnitude calculation, not a measured synchronized torque or a root cause.
