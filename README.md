# OpenSprite-SuperMimic

Joint motion tracking and velocity-command learning for **Open Sprite v5**, a
31-joint humanoid with four proximal shoulder J4340P motors. The project carries
SuperMimic from native robot reference motions through training, CPU deployment,
actuator-level validation, and supervised hardware experiments.

**License: [AGPL-3.0](LICENSE).** External code, assets, and motion datasets retain
their own terms; see [Data and licenses](DATA_AND_LICENSES.md).

## Current progress — November 10, 2026

**The robot has walked under the SuperMimic policy in supervised hardware tests.**
Two direct-start trials completed using a **0.25 m/s forward command**, full
training gains on all 31 joints, and fall protection. That value is the command,
not an externally measured walking speed. The operator reported intermittent
steps, right-foot dragging, outward-facing feet, and startup yaw. Reliable gait
quality and unrestricted operation remain open work.

The newest transport change commits each policy action to all 31 motors as one
group, avoiding an extra low-rate send cycle for hips and knees. It has passed
**disabled-motor validation and a deliberate 20 ms timeout test**; it has not yet
been evaluated during powered walking.

| Stage | Recorded result | Scope |
|---|---|---|
| Joint training, run002 | L40S 48 GB, 12 hours, 15,398 updates from random weights | 4,096 environments; FP32; TF32 disabled |
| Training-environment velocity evaluation | 60/60 trials of 30 s; 4/4 continuous sequences of 62 s | Serial-joint training actuators; simulation |
| Motion tracking | 15/15 trials of approximately 7.98 s | Five native reference clips, three seeds |
| Native policy interface | 10 history frames, 993 inputs, 31 outputs, 50 Hz | C++/ONNX Runtime CPU inference |
| Supervised hardware walking | Two direct-start trials completed; all 31 motors disabled afterward | Existing send schedule; remaining gait issues above |
| Coherent action sending | 2,475 actions × 31 motors = 76,725 matched hardware TX records | Motors remained disabled throughout |
| Transport timing | Maximum per-bus physical group span **0.646 ms**; target-to-TX-completion upper bound **3.109 ms** | Adapter timestamps and host receipt bounds; not motor execution timing |
| Timeout verification | Intentional policy pause triggered the original **20 ms** guard | Zero enables; **31/31 disabled** on both final validation runs |

Read the [action-history contract](docs/ACTION_HISTORY_CONTRACT_20261110.md),
[direct-start validation](docs/DIRECT_POLICY_START_20261110.md), and
[coherent-send results](docs/COHERENT_ACTION_SEND_20261110.md). Dated historical
reports describe earlier stages and may retain their original language.

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

## Reproduce the training experiment

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

## Frozen model and training configuration

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

Training builds on a pinned version of
[lzjever/opensprite-tracker](https://github.com/lzjever/opensprite-tracker) and its
[SuperMimic](https://arxiv.org/abs/2610.28480) implementation, with Open Sprite v5
references and actuator/deployment work. Retargeting uses
[UMR](https://hanyang9.github.io/UMR/); source motions include
[LAFAN1](https://github.com/ubisoft/ubisoft-laforge-animation-dataset).

Neither this repository nor its releases distribute **LAFAN1 data, retargeted
motion sequences, or reference-motion videos**. Keep those local and follow
their original terms. This project does not claim the paper's G1 results.
