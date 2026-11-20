# Sprite SM-Walk R1

R1 is the current supervised hardware candidate, frozen on November 19, 2026.
It has walked under Xbox commands on Open Sprite v5. Operator observations
include quieter foot contact and usable walking/turning, with some torso roll
and slow initiation. These observations are not an externally measured speed
benchmark or an unrestricted-operation qualification.

## Model and references

| Item | Identity |
|---|---|
| Training run | `reference-fusion-walk-stand80`, continuation update 6000 |
| Checkpoint SHA256 | `70cf3e70354c8b574777709f56f24f83864329d589c256cfa97d6a1236afdedd` |
| ONNX SHA256 | `5695b6980746534a11d075befc29bf5f7fe0ef2a7d81e3dadf3147c50f4d6930` |
| Parent checkpoint SHA256 | `884946b4f03a03d53057208bfdafe3b0c116f91fec06c493ec7307a49a9c5e4c` |
| Reference bank SHA256 | `edd4ddd6f54e8a8a710481d11340ac6e38d6b944e75613c48e318fc3b32d296f` |
| Contract | 10 history frames, 993 inputs, 31 position actions, 50 Hz |

The bank combines the reviewed clearance-corrected PM01/G71 walk with one
100STYLE Neutral standing clip. Reset clip probabilities are 80% walking and
20% standing; this is not a duration ratio. The two clips contain 872 frames
and 17.40 seconds of within-clip time. Neutral walking, dance and BONES-SEED
are not in this continuation. The world model learns simulated transitions,
not merely a prediction task on these reference frames.

The method retains motion and velocity encoders, a shared decoder/world model,
separate critics, equal motion/velocity environments and latent alignment.
It is not an AMP discriminator or a velocity-only PPO substitution. The
velocity interface takes the operator command; it needs no motion library at
deployment. MLP training uses BF16 while physical integration, kinematics and
rewards remain FP32. Global TF32 remains disabled.

## Simulation findings

The matched 6000-update continuation improved standing knees to approximately
10.7/12.6 degrees and backward torso pitch to 0.51 degrees. At a 0.50 m/s command,
measured simulated forward speed was 0.329 m/s; swing peaks were 30.4/18.6 mm.
The right-turn yaw rate was -0.177 rad/s for a -0.20 rad/s command. This remains
imperfect command tracking and asymmetric clearance.

Seven nominal cases, 14 initial-tilt cases, 12 bounded disturbance cases and
14 fixed-delay diagnostic cases survived. Four additional 60-second scenarios
also survived. A left-turn foot-spacing retention check failed despite passing
the absolute spacing threshold; this failure is retained. See the complete
[matched comparison](REFERENCE_FUSION_RESULTS_20261119.md) and
[reference-selection rationale](REFERENCE_FUSION_CAMPAIGN_20261118.md).

## Hardware execution and qualification scope

- Full training Kp/Kd on all 31 joints. Raw previous policy output enters action
  history; joint positions and velocities are real feedback.
- Warmup and control are separate. Direct takeover resets history and does not
  pre-run policy at zero gain or ramp policy gains/targets.
- Policy is 50 Hz. Direct joints retain 50 Hz nominal sends; parallel joints
  refresh at 500 Hz. Coherent new-action commits refresh every motor's cached
  target from the same action. The all-joint 500 Hz experiment is not selected.
- CAN runs on CPU 2 with FIFO 50. Inference main/ORT worker use CPUs 4/5 and
  receive threads CPU 1. ORT idle spinning is disabled. CPUs 4/5 stay at
  1.344 GHz during inference and restore the prior floor afterward.
- The PD layer saturates estimated command torque by adjusting position
  targets, preserving Kp/Kd and adding no feedforward torque. Instantaneous
  feedback-torque excursions are recorded rather than standalone stop triggers
  in this explicitly selected profile. Motor faults, position, velocity,
  temperature and timing guards remain active.
- Xbox limits are +0.40/-0.05 m/s forward/reverse, +/-0.20 rad/s yaw and zero
  lateral command. The current window is 300 seconds, within a 326-second
  schedule that includes supported standing and final braking/shutdown.

The final five-minute build passed a full no-enable run, late-window 20 ms
fault injection and terminal SIGINT shutdown. Each confirmed 31 disabled motor
readbacks. The normal run's inference median/max was 2.788/5.789 ms; its oldest
feedback to all TX-completion receipts peaked at 13.864 ms. Across the full and
fault runs, all 3,123,445 TX records had hardware timestamps. These are
no-enable transport results, not a five-minute powered walking trial.

Two earlier attempts stopped on the unchanged 20 ms guard after about 11.5 ms
of pre-inference receipt delay. Its scheduler cause remains unresolved. A
post-reboot attempt also stopped at the startup pose limit; the failing joint
was not captured. A subsequent zero-gain sample was in range and the interrupt
test passed. Do not remove these guards or claim the failures were fixed by
passing subsequent trials.

See [five-minute deployment evidence](JETSON_SM_WALK_R1_XBOX300_20261119.md) and
[thread/driver corrections](R1_LIVE_THREAD_LAYOUT_FIX_20261119.md). The source
under `runtime/` matches the qualified deployment; site-specific launchers in
`runtime/examples/r1-xbox300-jetson253/` document the original installation and
are not portable one-command robot installers. No command in this document
enables motors.

## Training and artifact use

Download the R1 release assets and verify `SHA256SUMS`. The policy package has
the ONNX, contract and training-control profile; the full checkpoint is a
separate release asset. The supplied video is a velocity-policy physics rollout,
not a reference-animation playback. Simulator results are separate from the
reported supervised hardware observations.

`configs/sprite-sm-walk-r1.json` preserves the exact recorded continuation
configuration, including the parent/reference transfer identities. For an exact
continuation, obtain the named parent checkpoint and the reviewed motion bank
identified below. The parent checkpoint and source motions are not included in
this R1 release. ONNX actor inference does not require reference motions, but
the current training and `scripts/evaluate_joint.py` utilities load the bank.
Do not describe a fresh run with a different parent or bank as an exact reproduction.

### Missing `walk_stand.npz` on a fresh clone

The historical local path is:

```text
outputs/reference-fusion-20261118/reference/walk_stand.npz
```

This file is **not included in Git or the R1 release**. It is the reviewed
PM01/G71 walk plus 100STYLE Neutral standing bank, not a downloaded LAFAN1
sequence. Its exact file identity is:

```text
Size: 1528892 bytes
SHA-256: edd4ddd6f54e8a8a710481d11340ac6e38d6b944e75613c48e318fc3b32d296f
```

Obtain the reviewed file from your authorized project archive or the project
maintainer. From the repository root, install the supplied file and verify it:

```bash
mkdir -p outputs/reference-fusion-20261118/reference
cp /path/to/received/walk_stand.npz outputs/reference-fusion-20261118/reference/
python -c "from pathlib import Path; import hashlib; p=Path('outputs/reference-fusion-20261118/reference/walk_stand.npz'); actual=hashlib.sha256(p.read_bytes()).hexdigest(); assert actual=='edd4ddd6f54e8a8a710481d11340ac6e38d6b944e75613c48e318fc3b32d296f', actual; print('Verified:', p)"
```

For training, also obtain the matching `walk_stand.qa.json` and place it beside
the NPZ. Its `motion_bank_sha256` must match the verified file. Do not manufacture
a passing QA record or bypass the checkpoint's reference-hash check.

The assembly script is `scripts/prepare_stand_walk_fusion.py`. It combines an
already reviewed clearance-corrected PM01 bank with the specifically selected
Neutral standing clip. **It is not a standalone downloader or a complete
fresh-clone regeneration pipeline:** its two prepared source banks and their QA
records are external inputs at the dated paths in that script. The
[reference-selection record](REFERENCE_FUSION_CAMPAIGN_20261118.md) explains the
selection; having only the public repository does not supply those inputs.
A newly retargeted or corrected bank is a new reference version unless its bytes
match the recorded SHA-256. Preserve source attribution and data terms from
[Data and licenses](../DATA_AND_LICENSES.md).

The campaign scripts preserve dated paths and deadlines as experiment records.
For a new run, use `scripts/train_joint.py --help` and provide new output paths,
the prepared bank and QA, and the explicit transfer source; do not launch the
dated campaign supervisor unchanged. The four-architecture experiment remains
separate from R1 and is not promoted by this release.

Code remains AGPL-3.0. Motion and character terms remain separate. Credit the
PM01/G71 source distributed through `BeijingDynamics/open_sprite_rl` and
**The 100STYLE Dataset — Ian Mason** (CC BY 4.0), with its original citation.
This release distributes neither source/retargeted motion sequences nor Geno
character data. It does not assert commercial clearance for external datasets
or silently relicense pretrained weights' data provenance. See
[data and licenses](../DATA_AND_LICENSES.md).
