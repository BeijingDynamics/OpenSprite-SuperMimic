# R1 source and artifact publication

This update advances the public repository from the November 11 run002
baseline to the frozen Sprite SM-Walk R1 candidate and its five-minute Xbox
runtime. It preserves the intervening velocity-v2 research history rather than
squashing failed comparisons out of the record.

The publication was assembled in an isolated worktree. The original dirty
development worktrees were not reset or overwritten. A local Git bundle and an
archive of uncommitted source/docs were saved before assembly; the separate
600,142,546-byte hardware evidence archive was already verified locally.

## Source selection

- Training modules and the R1 configuration come from the locally archived
  reference-fusion training source and its recorded 6000-update configuration.
- Native runtime and Python deployment support come from the exact qualified
  five-minute deployment snapshot. Text normalization is tracked separately
  from archived raw-file hashes in `provenance/sprite-sm-walk-r1-source.json`.
- The sole-height diagnostic merges the existing named-body/multi-clip audit
  with the training snapshot's reward factory. This repairs a source-assembly
  regression without changing training or inference math.
- Site-specific launchers are labeled as archived examples. Training campaign
  supervisors retain their original dates/paths and are historical experiment
  tools, not automatic jobs to run on a fresh installation.
- The separate actor-size comparison is not selected for this model release.

## Verification

Linux CPU-only regression: **551 tests and 55 subtests passed**. GPU visibility
was disabled, CPU thread counts bounded, and the running rental-GPU experiment
was not changed. **10/10 native tests passed** in an isolated Linux build.
The four echo-registration race-model tests and English publication check also
passed. Windows attempts exposed missing/Linux-only test-environment features;
the authoritative combined result is the complete Linux run.

Artifact SHA256 checks bind the released ONNX and full checkpoint to the R1
model card and the already qualified Jetson bundle. The public release includes
only an explicit allowlist: policy ZIP, full checkpoint, physical simulation
video and checksums. No reference sequence or raw robot log is published.

This is a **prerelease research/hardware-test candidate**. It includes supervised
hardware walking observations and qualified no-enable timing evidence, not a
claim that the five-minute powered profile was tested. Earlier natural timing
failures, startup admission failure and simulation-retention failures remain
documented. Source publication does not broaden hardware authorization or
weaken any live protection.
