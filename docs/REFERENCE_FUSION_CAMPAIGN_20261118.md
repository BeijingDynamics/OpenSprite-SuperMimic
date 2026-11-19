# Reference inventory and bounded overnight improvement campaign

## Objective and deadlines

The user requested a useful hardware-test candidate within sixteen hours of
approximately 19:05 on November 18, Asia/Shanghai. The server rental ends at
15:00 on November 19. The campaign stops initiating long training by 08:30
and reserves validation/export time until 10:30. Target review/delivery is
around 11:00, with additional rental time available for corrections and final
backup. Simulation success does not guarantee hardware performance.

The current priorities are natural standing, useful teleoperated forward
walking, bilateral clearance, preserved foot spacing, turning and disturbance
recovery. Failure to walk at 0.15 m/s alone is no longer a selection veto.
Do not weaken hardware protection or modify gains to satisfy these goals.

## Available data and selection

| Source | Available evidence | Value and current decision |
| --- | --- | --- |
| PM01/G71 original, adapted to current v5 | 11.98-second whole-body walk; operator-approved style | Useful baseline and source control; not independent of its corrected version. |
| PM01 clearance-corrected | 11.42 seconds, 14 swings per foot, approximately 25 mm reference peaks | Main walking anchor; retain exact samples and prior source QA. |
| 100STYLE Neutral standing | 5.98-second training clip, knees approximately +4.36/-4.14 degrees | Directly fills the absent standing demonstration; newly reviewed for a small targeted mixture. |
| Other Neutral training candidates | Short walks, a long curved path and rightward lateral motion | Previously rejected walking style and failed six-clip training are retained. Do not reintroduce the whole bank. |
| Neutral validation clips | Separate standing/walking/lateral source intervals | Keep held out; no transfer into training. |
| Previous LAFAN run002 library | Five 7.98-second straight/left/right/start/stop clips on current asset | Possible transition/turn research; mean planar speeds about 0.45-0.66 m/s and some strongly bent legs make unreviewed mixing inappropriate. |
| Tracker dance | 11,284 frames at 50 Hz, about 225.66 seconds; no current asset revision declared | Not directly qualified for the updated v5 plant. No dance in this campaign. |
| BONES-SEED | No qualified local reference bank located | Do not count unavailable data as part of the training library. |

The original PM01 and its clearance correction are variants of one source,
not two independent demonstrations. The tracker packed dance and per-motion
file also do not count as independent examples merely because there are two
files. Generic downloadable data is not copied into the publication repository.
All inherited licenses and source notices remain applicable.

The previous six-clip mixed experiment initially assigned equal mass to clips,
leaving only one sixth of reset probability to PM01; it later lost PM01 motion
tracking as well as velocity foot clearance. This does not uniquely identify
sampling dilution as its cause, but motivates an explicit controlled mixture.
The new experiment does not assume that raw kinematic dataset duration trains
a better world model. World-model transitions still come from simulator
experience, with the existing equal motion/velocity replay allocation.

## New bank and source qualification

The new bank contains exact samples from corrected PM01 and the previously
qualified Neutral training standing clip, with independent clip boundaries.
It has 872 frames across two clips, totaling 17.40 seconds of within-clip time.
SHA-256: `edd4ddd6f54e8a8a710481d11340ac6e38d6b944e75613c48e318fc3b32d296f`.

Packing verifies source QA hashes, matching model/joint/body identities,
training versus held-out status, and exact preservation of every trajectory
array. Current-model sole minima are 0.18 mm for PM01 and 0.46 mm for standing.
Four front/side sampled frames per clip were reviewed anew; the original source
qualification remains bound in the QA record. This sampled review is not
represented as a new full-length video review or dynamic-feasibility proof.
No arm splicing, looping across boundaries, or root-trajectory fabrication is
performed. Local artifacts: `outputs/reference-fusion-20261118/reference/`.

## Matched training and validation

Both arms transfer from upright-clearance3000, checkpoint
`884946b4f03a03d53057208bfdafe3b0c116f91fec06c493ec7307a49a9c5e4c`.
They use seed 49, the same posture/width/clearance rewards and recovery schedule,
and unchanged hardware model/gains/rates. The control keeps PM01 only. The
treatment gives PM01 and standing reset clip masses 0.8 and 0.2. Return-based
adaptation is restricted within each clip and cannot overwrite these masses.
These are reset probabilities, not a claim of exactly 80/20 replay occupancy.

Both critics and all optimizers reset in both arms for a matched comparison;
the controller, normalizers and world model are inherited. The reference
change requires an exact source/destination hash declaration. Ordinary resume
still rejects a changed reference or sampling distribution. Training remains
full joint SuperMimic, with shared decoder/world model, separate critics and
50/50 motion/velocity environments. Eleven focused tests and a three-update
GPU smoke with actual actor updates passed before scheduling the long run.

The bounded coordinator waits for the existing standing-cost pair to finish,
then evaluates the common parent and runs interleaved 1000/3000/6000 stages.
Severe physical failure stops an arm. Each stage saves per-clip motion tracking,
physical velocity rollouts, posture, foot geometry and video. Common primary
evaluation uses the controller-default initial pose so the mixed bank's quiet
standing frame cannot silently create an easier initial state than the control.
Commands include zero, 0.25, 0.5 m/s, 0.35 m/s curved walking, start/stop and
diagnostic 0.15 m/s. The original lower-speed reports remain preserved.

The two best completed candidates receive independent initial-tilt probes,
four-direction torso forces and lateral sustained pulls, plus separately
labeled 10/20 ms action-delay sensitivity tests. Delay profiles are diagnostic
plant overrides, not claims of measured actuator identification. Both are
exported with the unchanged 993-dimensional native interface, ONNX parity,
and replay of recorded histories against recorded policy actions. Hardware
admission remains false until runtime qualification and an authorized test.

## Persistence

Remote isolated source: `/root/gpufree-data/supermimic_datafusion_20261118`.
Campaign output: `outputs/reference-fusion-campaign` under that root.
Local output and automatic backup: `outputs/reference-fusion-20261118`.
The original comparison and its source remain unchanged.

The frozen source archive was copied locally and its SHA-256 verified:
`70f488e762320bd6d06fe8bd7df5192e2cb3ce256a610a0419c80a530bbe2f10`.
The independent backup worker copies each completed checkpoint with hash
verification, configuration, metrics, evaluation, videos and export folders.
It requires this local computer to stay running and connected; the remote
training supervisor continues independently of the chat and local backup.
Status files record failures rather than silently treating incomplete stages
as successful. No deployment or motor enabling is part of this supervisor.

## Preflight follow-through

The complete common-initial-state evaluation was exercised on the parent
before unattended training. Seven cases survived, including diagnostic low
speed. The practical screen passed standing stability, useful forward motion,
left/right turns and settling after stop; standing knee extension remained
the failed criterion (maximum median knee flexion approximately 35.8 degrees).
This is a nominal baseline, not a new robustness or hardware qualification.

The preflight caught a NumPy boolean JSON-serialization issue in the new
selection report. It was fixed before the queued campaign began. The refreshed
supervisor accepts only an empty, still-waiting state; no active training was
restarted. Real-rollout checks verified JSON serialization, baseline self-
comparison, rejection of a clearance regression, and exclusion of 0.15 m/s
from selection vetoes. New candidates additionally retain each baseline
case's P05 spacing within 5 mm, bilateral swing peaks within 15%, and moving
pitch magnitude within 2 degrees. These are research selection checks, not
changes to hardware protection.

The parent export and native observation-order pipeline passed 880 recorded
history samples, with maximum action error 1.67e-6. A separate four-second
10 ms delay harness smoke completed; this verifies the tool path, not delayed
walking robustness. Both preflight and parent-export evidence were saved
locally. The final corrected source snapshot is preserved separately as
`source-snapshot-v2.tar.gz`, SHA-256
`1825d656f7e80d5515282ab7418e9d117d033b796c5fb1c75a910e1dcb534b7e`.
