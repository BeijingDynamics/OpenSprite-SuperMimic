# Validated runtime source snapshot

Snapshot of `BeijingDynamics/open_sprite_runtime`, branch `codex/supermimic-v5`,
commit `5f8225ce6258cb38db174ee18c55162fa34d96a6`.
The snapshot includes native 10-frame/993-input replay, receive-only IPC, motor maps,
strict closed-loop simulation, tests and their dependencies. No model is hardware qualified.

Use the parent project's [CPU and closed-loop instructions](../docs/RUNTIME.md).
The recommended commands build only file-replay and test targets, and never enable motors.
Other historical commissioning tools are retained as source dependencies; they are not
the entry point for this model. No deployed controller is replaced by installing this package.
