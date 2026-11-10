# Project conventions

The primary README, first-party code comments/docstrings, terminal output,
operator prompts, and generated labels must be English. Preserve original
third-party notices and immutable historical evidence. Retain AGPL-3.0.

Preserve full joint SuperMimic: motion/velocity encoders, shared decoder and world model,
separate critics, 50/50 environments, 10 history and 10 future reference frames, latent alignment.
Do not replace it with PPO or a velocity-only shortcut.
Training PD is 500 Hz; policy is 50 Hz. Keep published gains, effort caps, defaults and their
exact asset/control profile hashes together. A newer upstream asset is a separate experiment.
Reference changes require numerical QA and rendered review before long training.
Simulation survival, CPU replay, hardware feedback and powered motion are separate gates.
No motor enable or hardware transmission without explicit user authorization.
Do not weaken torque, speed, position or body protection to obtain passing results.
Keep credentials, generic raw datasets, dependency caches and generated binaries out of Git.
Public code license does not relicense external motion/character assets; see DATA_AND_LICENSES.md.
