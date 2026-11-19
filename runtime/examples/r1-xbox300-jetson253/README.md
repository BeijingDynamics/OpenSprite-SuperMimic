# Archived R1 five-minute operator entry

These English launchers describe the qualified November 19 Jetson installation.
They retain its absolute paths, sealed hardware configuration, evidence gates,
driver dependency and explicit `READY` prompt. They are not a portable install
script and must not be executed on another robot without rebuilding and
validating that robot's configuration.

The maintained C++ implementation is in `runtime/native`; Python support is in
`runtime/src` and `runtime/tools`. No native executable, motor calibration,
raw CAN/IMU recording or motion dataset is distributed in this directory.

The example requires the echo-registration-order driver correction, not merely
the earlier timestamp-ioctl correction. Its loader is temporary and reboots
restore the installed driver. `load-echo-order-driver-implementation.sh` records
the loader's lock, dependency checks and rollback; the original module and its
hash remain local. It does not send motor frames. The separate scheduler helper
checks CPU 2 / FIFO 50 and grants only the CAN executable `CAP_SYS_NICE` when
the operator provides sudo authorization.

The 300-second Xbox window follows the existing 20-second supported-standing
phase; total native control duration is 326 seconds. Release LT for command
braking. Ctrl+C requests motor disable, so retain fall protection. Individual
readback of all 31 disabled states is required; a process exit alone is not
proof of disabled motors. Read [R1 qualification](../../../docs/SPRITE_SM_WALK_R1.md)
and the known failures before adapting this example.
