# OpenSprite-SuperMimic

Joint motion tracking and velocity-command learning for **Open Sprite v5**, a
31-joint humanoid with four proximal shoulder J4340P motors. The project carries
SuperMimic from native robot reference motions through training, CPU deployment,
actuator-level validation, and supervised hardware experiments.

**License: [AGPL-3.0](LICENSE).** External code, assets, and motion datasets retain
their own terms; see [Data and licenses](DATA_AND_LICENSES.md).

## Current progress — November 19, 2026

**Sprite SM-Walk R1 is the current supervised hardware candidate.** It has
walked under Xbox commands on Open Sprite v5. The operator reported quieter
foot contact and useful walking/turning; torso roll, slow initiation and gait
asymmetry remain areas for improvement. Commanded speed is not a measured
hardware speed benchmark, and the robot is not qualified for unrestricted use.

R1 combines reviewed **PM01/G71 walking and 100STYLE Neutral standing** with
80/20 reset sampling. It retains the full joint SuperMimic method: motion and
velocity interfaces, shared decoder/world model, separate critics and latent
alignment. The world model learns simulator transitions. Deployment needs the
velocity actor and real feedback, not reference motions or an AMP discriminator.

The selected runtime uses **50 Hz policy inference, 50 Hz nominal direct-joint
sends and 500 Hz parallel-joint refresh**, with coherent new-action commits.
The all-joint 500 Hz experiment remains separate. All 31 joints use training
Kp/Kd; raw previous policy actions enter history. The current Xbox window is
**300 seconds**, preceded by supported standing and followed by braking and
31-motor disable verification. Its complete no-enable validation has passed;
a five-minute powered walking result is not claimed.

| Evidence | Result | Scope |
|---|---|---|
| Standing-reference continuation | Standing knees approximately 10.7/12.6°, torso backward pitch 0.51° | Simulation, selected seed; parent and matched control retained |
| Forward walking | 0.329 m/s measured for 0.50 m/s command; swing peaks 30.4/18.6 mm | Simulation; speed bias and asymmetry remain |
| Bounded robustness | 14/14 initial-tilt, 12/12 disturbance, 14/14 delay and 4/4 additional 60 s cases survived | Specified simulator tests, not arbitrary pushes |
| Supervised hardware | R1 Xbox walking reported; first faster-start 120 s trial completed and 31 motors disabled | Operator observations and controller logs |
| Five-minute no-enable integration | Full 326 s schedule, late 20 ms fault injection and Ctrl+C shutdown passed | Zero enables; 31/31 disabled after each |
| CPU inference / transport | Median inference 2.788 ms, max 5.789 ms; max oldest-feedback-to-all-TX-receipt 13.864 ms | Final normal no-enable run; hardware TX is not motor execution |

**Known failures stay visible:** right-foot clearance remains lower; the selected
candidate failed a relative left-turn spacing-retention check; earlier live
attempts hit the unchanged 20 ms guard after an approximately 11.5 ms receipt
delay whose scheduler cause remains unresolved. Passing later runs does not
remove that uncertainty. The four-architecture experiment is ongoing separately
and has not replaced R1.

Start with the [R1 model card and reproduction notes](docs/SPRITE_SM_WALK_R1.md),
[matched training results](docs/REFERENCE_FUSION_RESULTS_20261119.md),
[five-minute runtime evidence](docs/JETSON_SM_WALK_R1_XBOX300_20261119.md), and
[thread/driver corrections](docs/R1_LIVE_THREAD_LAYOUT_FIX_20261119.md).
[Download R1 policy, checkpoint and simulation video](https://github.com/BeijingDynamics/OpenSprite-SuperMimic/releases/tag/sprite-sm-walk-r1-20261119).

| Current R1 artifact | SHA256 |
|---|---|
| Checkpoint | `70cf3e70354c8b574777709f56f24f83864329d589c256cfa97d6a1236afdedd` |
| ONNX actor | `5695b6980746534a11d075befc29bf5f7fe0ef2a7d81e3dadf3147c50f4d6930` |

## Why the deployment contract matters

- The previous-action observation contains the **raw previous policy action**.
  Joint positions and velocities come from real feedback. A protected or scaled
  target must not be substituted for the policy's previous action.
- Warmup is separate from control. Direct takeover resets history from fresh
  feedback and immediately uses all 31 training Kp/Kd values. There is no
  zero-gain policy pre-roll or gain/position ramp in this mode.
- The PD layer preserves Kp/Kd and limits estimated motor torque by adjusting
  the position target. Parallel ankles retain their motor-space mapping and
  constraints. No extra feedforward torque is added.
- Training PD is **500 Hz** and the policy is **50 Hz**. These are distinct from
  CAN refresh rates and from the motor's internal control loop.
- Hardware time, driver handoff time, and policy timing use different clocks.
  The transport audit records their identities and bounds rather than
  subtracting unrelated timestamps.

## Training setup and the original run002 baseline

For the current R1 continuation, use [its model card](docs/SPRITE_SM_WALK_R1.md)
and [recorded configuration](configs/sprite-sm-walk-r1.json). The commands below
reproduce the older run002 baseline and do not train R1 unchanged.

The recorded training environment used Linux, Python 3.12, and CUDA 12.8.
Install the pinned dependencies and robot meshes:

```bash
git clone https://github.com/BeijingDynamics/OpenSprite-SuperMimic.git
cd OpenSprite-SuperMimic
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-training.txt
python -m pip install -e .
python scripts/fetch_robot_assets.py
```

Prepare authorized motion data locally using the
[reference-motion procedure](docs/REFERENCE_MOTIONS.md). The mesh downloader
checks all 33 STL hashes against the pinned manifest. It does not substitute
the latest upstream robot for the trained asset.

```bash
export MUJOCO_GL=egl
python scripts/preflight_run002.py
python scripts/test_joint_gradient_routing.py
python scripts/check_reference_fk_precision.py
python scripts/train_joint.py --config configs/run002.json \
  --motion data/motions/contact_v5_v11/v5_locomotion.npz \
  --quality data/motions/contact_v5_v11/v5_locomotion.qa.json \
  --output outputs/smoke --smoke
```

For the full run, choose a new output directory and omit `--smoke`. The recorded
12-hour budget ended at 15,398 updates; the configuration's 100,000-update limit
was not reached. See [reproduction details](docs/REPRODUCE.md),
[runtime integration](docs/RUNTIME.md), and [results and lessons](docs/RESULTS_AND_LESSONS.md)
for the original experiment records.

[Release assets](https://github.com/BeijingDynamics/OpenSprite-SuperMimic/releases)
include checkpoints, the ONNX actor, scenes, and validation results with hashes.
The exported CPU velocity interface does not require reference motion data.
Retraining and motion-tracking evaluation require separately prepared references.

## Original run002 model and training configuration

- Robot: `sprite0825_v5_4340_shoulders_waist20261105`; waist yaw link mass 3.7391 kg.
- Published Kp/Kd, effort limits, and default pose are matched by joint name.
  The experiment explicitly overrides the source control metadata to
  **500 Hz PD / 50 Hz policy / 10 substeps**.
- Full joint method: separate motion/velocity encoders and critics, shared
  decoder and world model, 50/50 environment split, 10 history frames,
  10 future reference frames, and latent alignment.
- Configuration: [configs/run002.json](configs/run002.json).
  Asset, code, and dependency identities: [provenance/](provenance/).

| Artifact | SHA256 |
|---|---|
| `model_015398.pt` | `094b52267159d26f4d5bfb41edc5823a9becc32f706c87df05f8aa36b4016282` |
| `policy.onnx` | `f3529c927a44f970908d48e5694e63302d0fc838121aba2fa87c0a0863968bba` |
| Published control snapshot | `184ec989fd24b4bc6ea5034e6fdae796b5b9ce83ef2c884dfe4fc61d8d6987dc` |

## Repository layout

```text
opensprite_tracker/supermimic/  World model, joint interfaces, BPTT, replay, simulation
scripts/                       Training, evaluation, export, reference processing, QA
configs/                       Experiment, evaluation, retargeting, actuator simulation
upstream_asset/                Pinned URDF/MJCF/control snapshots; meshes fetched by hash
retarget/ + third_party/umr/   Retargeting and contact optimization; original licenses
data/                          Provenance, clip selection, QA metadata; no motion sequences
runtime/                       Native runtime sources, commissioning tools, tests
reports/                       Numerical results and validation summaries
docs/                          Reproduction notes and dated development records
```

The [robot model](https://github.com/BeijingDynamics/open_sprite) and
[hardware runtime](https://github.com/BeijingDynamics/open_sprite_runtime) are
separate repositories. The runtime snapshot here contains experiment-specific
commissioning tools and pinned hardware configuration checks; installing the
Python package does not deploy or arm a robot. Review the native replay,
[SPI power-management requirement](docs/SPI_POWER_MANAGEMENT.md), actuator
mapping, scheduling, and shutdown checks for the exact target machine.

## Contributing and attribution

The primary README, first-party code comments, terminal messages, operator
prompts, and generated labels are **English** so contributors worldwide can
use and review the project. Preserve original third-party notices and historical
measurement records. Run `python scripts/check_language_policy.py` before publishing.

The original training baseline builds on a pinned version of
[lzjever/opensprite-tracker](https://github.com/lzjever/opensprite-tracker) and its
[SuperMimic](https://arxiv.org/abs/2610.28480) implementation, with Open Sprite v5
references and actuator/deployment work. The velocity-v2/R1 changes follow the
released [official SuperMimic](https://github.com/HybridRobotics/supermimic) and
[C. Zhang implementation](https://github.com/zita-ch/supermimic_vibed) comparison
recorded in [the v2 design audit](docs/VELOCITY_V2_RETRAIN.md); this is not a claim
of reproducing the paper's G1 benchmark. Retargeting uses
[UMR](https://hanyang9.github.io/UMR/); source motions include
[LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset).

Neither this repository nor its releases distribute **LAFAN1 data, retargeted
motion sequences, or reference-motion videos**. Keep those local and follow
their original terms. This project does not claim the paper's G1 results.
