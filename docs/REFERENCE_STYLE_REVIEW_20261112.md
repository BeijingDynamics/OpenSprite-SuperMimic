# Reference style review, November 12, 2026

## Accepted walking style and rejected candidate

The operator approved the rendered PM01/G71 walking style on the current v5
model. Use the complete G71 whole-body reference as the primary ordinary-walk
candidate. This visual approval does not assert dynamic tracking success or
complete start/stop/turn coverage.

Pinned source: `BeijingDynamics/open_sprite_rl` at
`63a82f744c78d48ee42f557fb472d7499702ee94`,
`assets/references/g71/sprite0825_pm01_g55_ik_contactclean_wbt_v1.npz`.
SHA-256: `97ec26f45460316e55845137b2adeb83958bd2676967af166085df085b509256`.
The source has 1,199 samples at 100 Hz. The current-v5 review keeps every second
sample, preserving root and joint poses while recomputing derivatives and FK.
Retain EngineAI's BSD-3-Clause notice and provenance. Do not confuse the full
body reference with the reduced AMP expert arrays, which omit root poses.

The demonstrated LAFAN/UMR clip
`walk3_subject4_009000_009360_straight` was rejected: its hands remain too far
in front of the body. The new 13-candidate LAFAN refinement batch was stopped,
with its logs and rejection record preserved. No production v2 training had
started. Its reference/contact feasibility result is not style acceptance.

The error is not attributable solely to the human source. On identical Sprite
geometry, median shoulder-to-wrist displacement along pelvis heading changed:

| Reference | Left forward distance | Right forward distance |
| --- | ---: | ---: |
| Native UMR of the rejected clip | 0.1066 m | 0.0958 m |
| Contact/torso-refined clip | 0.2247 m | 0.2093 m |
| PM01/G71 on current v5 | 0.0371 m | 0.0429 m |

These are FK measurements, not a dynamic experiment. The postprocessing changed
the relationship between root/torso and arms without preserving sufficient
upper-body semantic targets. Any future retarget repair must review the full
body after each stage; improving sole contact or torso heading alone is not
sufficient. Do not splice arbitrary arm poses onto a demonstrated lower body.

## Numerical qualification remains separate

The earlier 20 mm sole-clearance event requirement was a dataset screening
heuristic, not a universal definition of walking and not a hardware limit.
G71's measured maximum lowest-sole heights are about 15.0 and 13.9 mm. Its
current-v5 FK agrees with the stored source to below one micrometre, with no
hard joint-limit or geometry violation in the completed audit.

Neither visual approval nor historical AMP-policy success proves exact
whole-body SuperMimic tracking. The old height-only support test and V20's
height-and-speed support proxy measure different things. In particular, a
velocity-filtered support mask cannot independently prove low slip. Preserve
the original reports and resolve contact-phase interpretation, resampling and
closed-loop behavior in a separately versioned PM01 audit. Do not silently
flip the old audit's failed fields or change runtime protections.

## Optional dance data and world-model coverage

The operator asked whether a 100-second tracker dance excerpt would help the
world model. It is optional, not a prerequisite. The current tracker README
describes a 225.66-second `dance2_subject1` reference; a 100-second excerpt is
a separate subset that needs its own review on the current v5 embodiment.

In this implementation, reference frames select tracking goals and reset
states. Actual simulator rollouts populate replay with state, action, context
and next-state tuples. World-model fitting draws equally from the motion and
velocity interfaces. Raw kinematic reference frames are not supervised
action-conditioned physics transitions.

Adding dance references to the current motion bank affects the motion encoder,
shared decoder and collection distribution as well as the world model. It is
not a world-model-only intervention. A world-model-only auxiliary experiment
would require separately collected, physically simulated transitions and
explicit sampling isolation; this has not been implemented or qualified.

First establish the approved PM01 walking baseline. Prioritize relevant state
coverage: stand/walk transitions, speed changes, left/right turns, contact
switches and recovery from modest disturbances. Test any dance supplement as
a named ablation, keeping clean-walking evaluation fixed. Compare held-out
1/8/32-step prediction errors and physical command tracking, foot clearance,
heading and stability; lower average training loss alone is insufficient.

World-model coverage is not the same as reference duration: a short reference
can generate many diverse physical transitions, but it does not by itself
provide a broad movement repertoire for the shared motion decoder. Repeated
copies of a clip add sampling weight, not independent motion diversity.

## Sources

- https://github.com/BeijingDynamics/open_sprite_rl/blob/63a82f744c78d48ee42f557fb472d7499702ee94/docs/DATA_PROVENANCE.md
- https://github.com/BeijingDynamics/open_sprite_rl/blob/63a82f744c78d48ee42f557fb472d7499702ee94/docs/SPRITE0825_STABLE_WALK_HANDOFF.en.md
- https://github.com/lzjever/opensprite-tracker/blob/main/README.md
- Local implementation: `scripts/train_joint.py`, `collect`, `MixedReplay`,
  `validate_world`; `opensprite_tracker/supermimic/engine.py`, `wm_update`.
