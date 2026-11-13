# Actuator model audit: November 13, 2026

This is a read-only audit. No training gains, physical assets, runtime protections,
or hardware settings were changed. No hardware was accessed.

## What the completed PM01 comparisons actually simulate

The training entry point explicitly constructs both interfaces with
`torque_lag_enabled=False` (`scripts/train_joint.py`). The actuator class is
`FilteredIdealPdActuator`, derived from mjlab's `IdealPdActuator`, but the name
does not mean its optional torque filter was active in these runs.

- Policy: 50 Hz; external joint-space PD: 500 Hz.
- Position target: default joint angle plus 0.25 rad times clipped actor output.
- Desired joint velocity and feedforward torque: zero.
- Torque: PD error followed by the published per-joint constant effort cap.
- Command delay: zero or one 2 ms physics write, with the inherited delay queue.
- Torque time constant: zero, not the optional 5--10 ms range in `constants.py`.
- Published Kp/Kd and effort limits are fixed; gain/effort randomization is disabled.
- Armature values come from the pinned public MJCF; this is model provenance,
  not evidence of measured actuator identification. Examples include 0.032 kg m^2
  for hip/knee and four shoulder joints, 0.0018 for shoulder yaw/elbow,
  0.0036 for the serial ankle coordinates, and 0.01 for waist roll.
- The model does not implement voltage-dependent torque-speed envelopes,
  measured breakaway/Stribeck friction, backlash, or temperature derating.
- The loaded MJCF reports zero `frictionloss` for every degree of freedom.
  Multiplicative friction randomization leaves that zero unchanged; it does
  not introduce nonzero gearbox friction.
- Joint-speed reward thresholds are not a torque-speed motor model.
- The training uses serial ankle coordinates, not the physical parallel linkage.

The frozen profile is `upstream_asset/control_profiles/sprite0825_serial_training_20261105.json`.
Its SHA256 is `184ec989fd24b4bc6ea5034e6fdae796b5b9ce83ef2c884dfe4fc61d8d6987dc`.
The profile's archived 200 Hz timing is explicitly overridden to the approved
500 Hz training variant. The runtime hardware specification file is not an
identified actuator model and is not the source of the training dynamics.

## Output-shaft attachment

For a joint attached directly to a geared module's output shaft, the external
transmission ratio is one (up to joint-axis sign and encoder offset). Internal
gear reduction, reflected rotor inertia, gearbox friction, backlash and drive
dynamics still exist. Output-side torque/speed data must not be multiplied by
the internal ratio a second time. Rotor-side inertia, if that is what a manual
specifies, reflects approximately as `J_output = J_rotor * N^2`; an already
output-referred inertia must not be reflected again. CAD link inertia and rotor
armature also require separate accounting to avoid double counting.

For a parallel linkage, motor coordinates and generalized joint coordinates
require a configuration-dependent Jacobian. Direct attachment to the motor
output does not remove that linkage transformation.

## Public sources reviewed

1. [DAMIAO official joint-motor catalog](https://www.mdmbot.com/index.php?c=category&id=22)
   lists the requested motor families, including multiple revisions of 4340P.
   Pin the exact revision, voltage and firmware; the short family name is insufficient.
2. [OpenArm MuJoCo](https://github.com/enactic/openarm_mujoco) provides public
   models for a robot using DAMIAO modules. A usable MJCF is not by itself a
   validated identification of our exact four actuator variants.
3. [mjlab DC actuator implementation](https://mujocolab.github.io/mjlab/main/_modules/mjlab/actuator/dc_actuator.html)
   provides velocity-dependent torque saturation. This is a reusable model
   structure, not a DAMIAO parameter set. Its directional motoring/braking
   envelope should not be replaced with an indiscriminate absolute-speed clip.
4. [Rhoban BAM](https://github.com/Rhoban/bam) supplies an identification pipeline,
   extended friction models and MuJoCo CPU/Warp integration. Its README lists
   identified Dynamixel, eRob and Feetech servos, not these four DAMIAO variants.
5. [MABEL](https://github.com/robotmabel/open_mabel) documents DAMIAO friction/inertia
   calibration. This is a possible workflow lead; the exact parameter artifacts
   and validation for our motor revisions have not been verified.
6. [DAMIAO friction GUI](https://github.com/cdssywc/qt5_damiao_motor_friction_detection)
   describes static/Coulomb friction identification. Its fetched README contains
   conflicting merge markers and noncommercial license language. Do not vendor
   it as an unrestricted dependency or treat it as a validated four-model library.

The search did not establish a ready-to-use, experimentally validated model
library covering DM-J4340P, DM-J4310P, DM-J3507 and DM-J6248P in our revisions.
This is a bounded search result, not proof that no such model exists.

## Proposed next model experiment

Preserve the current training comparisons. In a separate versioned experiment,
start with output-side PD, motor-specific torque-speed/current/voltage envelopes,
reflected inertia, friction and measured command/response delay. Match the actual
supply voltage; the runtime source records nominal 38 V, while several catalog
parameters are explicitly for 48 V. Keep peak, continuous, communication-codec
range and runtime safety limits distinct.

Use existing logs to check units, signs, saturation and delay first. Walking
logs alone generally cannot uniquely separate friction, inertia, external load
and torque-calibration errors. Later, representative samples of each exact
motor variant should be tested with controlled loads and held-out trajectories.
Built-in current-derived torque feedback is not an independent output torque
sensor. No powered identification experiment is authorized by this document.

Fit only parameters supported by measurements; randomize residual uncertainty.
A learned actuator network is optional, not a prerequisite. A world model
trained on simulator rollouts learns the simulated actuator response and cannot
discover an omitted real gearbox friction law without real evidence.

## Follow-up: actual artifacts found in other projects

The follow-up search found concrete published identification outputs, beyond
the initial README-level leads. They narrow the gap but do not establish a
validated, revision-matched library for all four Open Sprite motor types.

### MABEL identification outputs

[DM-J4340P params.json](https://github.com/robotmabel/open_mabel/blob/main/calibration/motor_calibration/results/DM-J4340P/params.json)
reports Stribeck fitting and labels the following values as measured:

| Output-side quantity | Published value |
| --- | ---: |
| Positive/negative Coulomb friction | 0.33308 / 0.34538 N m |
| Positive/negative static friction | 0.34988 / 0.38229 N m |
| Viscous coefficient | 0.183973 N m s/rad |
| Stribeck velocity | 0.087989 rad/s |
| Reflected inertia | 0.02590013 kg m^2 |

The same result explicitly marks the stall torque as unmeasured and substitutes
the 27 N m datasheet value, with 9 N m continuous torque. The published inertia
fit RMS residual is 0.4235 N m. The result does not by itself pin the exact
firmware, supply voltage and our V1.1 hardware revision. The inspected results
tree contains parameter files, not raw independent validation trajectories.
Treat these as author-reported identification results and candidate priors.

[DM-J4310 params.json](https://github.com/robotmabel/open_mabel/blob/main/calibration/motor_calibration/results/DM-J4310/params.json)
reports Coulomb friction approximately 0.082864 N m and viscous coefficient
0.01976 N m s/rad. It is J4310, not our J4310P; its `J: 0.0` is not evidence
that physical reflected inertia is zero. The inspected result tree has no
identified J3507 or J6248P parameter file.

The [generated Isaac Lab example](https://github.com/robotmabel/open_mabel/blob/main/calibration/motor_calibration/results/DM-J4340P/isaaclab_actuator.py)
must not be copied directly: its assignment name contains a hyphen (invalid
Python), and it fills actuator `damping` with physical viscous friction despite
acknowledging that this field is the PD control gain. Physical friction belongs
in the plant model, separately from the approved Kd. No code was imported.

### OpenArm model

[v1/openarm.xml](https://github.com/enactic/openarm_mujoco/blob/1daab06716b779376dbe7f01fb0548f8b539bb35/v1/openarm.xml)
sets DM4340 armature/frictionloss/damping to 0.030/0.1/0.9 and DM4310 to
0.002/0.1/0.9. Its own comments call rotor-derived damping reference-only and
state that gearbox contributions are unconfirmed. This is useful implementation
and plausibility evidence, not a reason to replace Open Sprite parameters.

### DM-Dog-RL and RoboParty

[DM-Dog-RL actuator config](https://github.com/Maverick220141/DM-Dog-RL/blob/5be96e8a45b94e536659c45047b79d8170ef96ed/train/robolab/robolab/assets/robots/anymal_c.py)
uses `DelayedPDActuatorCfg` for J6248P, with base constants 97 N m, 6.283 rad/s
at its stated 24 V condition, and armature 0.01 kg m^2. Task overrides use
85 N m, 5.5 rad/s and delays 1--3 ticks. Its
[speed-torque function](https://github.com/Maverick220141/DM-Dog-RL/blob/5be96e8a45b94e536659c45047b79d8170ef96ed/train/robolab/robolab/tasks/anymal_c/anymal_c_mdp.py)
is a reward penalty on torque exceeding a linear absolute-speed envelope.
The inspected actuator implementation is not a hard torque-speed saturation
model. The distinction matters when adapting ideas to SuperMimic's world model.

[RoboParty's humanoid actuator config](https://github.com/Roboparty/roboto_origin/blob/d94afba45ef029c5f77a268751ee638cc64c84d0/modules/roboparty_train/robolab/robolab/assets/robots/roboparty.py)
also uses delayed PD, with 0.01 kg m^2 armature and 0--2 delay ticks for the
inspected groups. Its BOM includes 48 V DM4340P modules. This demonstrates a
practical approximate-actuator training approach, not a published learned
DAMIAO actuator network or measured family-specific friction model.

## Recommended training sequence

1. Preserve both completed 6,030-update PM01 comparisons and their artifacts.
2. Build a coverage-aware PM01 plus reviewed Neutral motion bank. Use PM01 as
   the established ordinary-walking anchor; add qualified standing, forward,
   turning and lateral clips. Audit forward speed, signed turn rate and signed
   lateral speed, rather than trusting clip names or raw duration. Prefer
   balanced source selection; any additional sampling weights must be explicit
   and must preserve SuperMimic's return-based sampler. Keep held-out source
   intervals excluded from training. A yaw rotation does not reverse turn sign;
   mirroring requires full kinematic/dynamic QA and is not currently implemented.
3. Run a fixed-dynamics data comparison at approximately 6,000 updates, then
   continue a promising candidate toward 12,000--18,000 updates. Preserve the
   full joint SuperMimic interfaces, encoders/shared decoder/world model,
   critics and latent alignment. An additional sole-height reward remains an
   explicitly versioned extension, not an undocumented paper default.
4. In parallel with data preparation, evaluate existing policies under separate
   actuator sensitivity sweeps. Use external parameters as labeled priors;
   measure static friction, velocity-dependent friction, inertia, transport
   delay and voltage-dependent torque capability separately before combining.
   Do not substitute plant friction for control Kd, or use codec limits as
   physical capability. Model parallel-joint mappings where needed.
5. Once the physical-model candidate is supported, train a paired comparison
   using the same motion bank, seed, command sampler and reward. Changing the
   plant changes the world-model target; retrain or explicitly adapt the world
   model/replay buffer rather than silently loading the old model as if the
   dynamics were unchanged.
6. Select by sustained zero-command station keeping, bilateral swing-event
   clearance distributions, toe direction, loaded slip, forward/reverse/turn/
   lateral command tracking, and graded force-disturbance recovery. Report
   training-seed replication separately from evaluation-seed repeats. Maximum
   clearance and survival alone are insufficient.

Measured PM01 throughput suggests 6,000 updates take about 2--3 hours and
12,000--18,000 updates about 5--8 hours on this L40S, before evaluation and
allowing for changed motion/model cost. This remains an estimate, not a claim
that the candidate will meet the gait gates in that time. The subsequently
authorized mixed-reference run launched at 12:49 China time on November 13;
see [its run record](MIXED_REFERENCE_RUN_20261113.md) for the frozen configuration,
parallel actuator sensitivity matrix and evaluation/backup setup.
