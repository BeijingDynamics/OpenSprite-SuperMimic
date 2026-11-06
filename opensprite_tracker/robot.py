"""Pinned waist20261105 asset and named serial controls, with500Hz timing.

URDF limits remain immutable metadata. Applied torque caps, velocity-cost
thresholds and action offsets come from the user-approved training profile.
"""
from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET
import json
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[1]
ROBOT_XML = ROOT / "assets/opensprite/opensprite.xml"
ASSET_BUNDLE = ROOT / 'upstream_asset'
_loader = ASSET_BUNDLE / 'scripts/control_profile.py'
_spec = importlib.util.spec_from_file_location('sprite_run002_pinned_controls', _loader)
_module = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = _module
_spec.loader.exec_module(_module)
CONTROL_PROFILE = _module.load_control_profile(repository_root=ASSET_BUNDLE)
ASSET_REVISION = CONTROL_PROFILE.data['model_binding']['asset_revision']
CONTROL_PROFILE_SHA256 = CONTROL_PROFILE.sha256
URDF = ASSET_BUNDLE / CONTROL_PROFILE.data['model_binding']['asset_directory'] / 'sprite0825_float.urdf'
PHYSICS_DT_S = .002
POLICY_DT_S = .020
DECIMATION = 10
INITIAL_ROOT_HEIGHT = .46224576
FOOT_BODY_NAMES = ("left_ankle_roll_link", "right_ankle_roll_link")
FOOT_COLLISION_PATTERN = r"^(left|right)_foot_collision$"
BODY_PRIMITIVE_COLLISION_PATTERN = r"^.*_primitive_collision$"
FULL_BODY_COLLISION_PATTERN = r"^.*_collision$"

_joints = tuple(j for j in ET.parse(URDF).getroot().findall("joint")
                if j.get("type") != "fixed")
BODY_JOINT_NAMES = tuple(j.get("name") for j in _joints)
HEAD_JOINT_NAMES = tuple(n for n in BODY_JOINT_NAMES if n.startswith("head_"))
LEG_JOINT_NAMES = tuple(n for n in BODY_JOINT_NAMES
                        if any(x in n for x in ("hip_", "knee_", "ankle_")))
ARM_JOINT_NAMES = tuple(n for n in BODY_JOINT_NAMES
                        if any(x in n for x in ("shoulder_", "elbow_", "wrist_")))
# Track the actual complete Sprite tree, including both waist links, all three
# head links, intermediate hip/shoulder housings and every wrist link. Its
# ordering is the MJCF DFS order used by MuJoCo, with the floating pelvis first.
TRACKED_BODY_NAMES = tuple(body.get("name") for body in
                          ET.parse(ROBOT_XML).getroot().find("worldbody").iter("body"))


@dataclass(frozen=True)
class JointControl:
    kp: float
    kd: float
    command_effort_nm: float
    urdf_velocity_rad_s: float
    default_pos_rad: float = 0.


def _gains(name):
    control = CONTROL_PROFILE.joints[name]
    return control.kp, control.kd


JOINT_CONTROL = {}
JOINT_LIMITS = {}
_named = CONTROL_PROFILE.for_joint_names(BODY_JOINT_NAMES)
for joint in _joints:
    name = joint.get("name")
    limits = joint.find("limit")
    control = _named[name]
    # urdf_velocity_rad_s is the legacy API field name; the value is a reward
    # threshold from the selected profile, not a hard motor-speed capability.
    JOINT_CONTROL[name] = JointControl(control.kp, control.kd, control.command_effort_nm,
                                      control.velocity_limit_rad_s, control.default_pos_rad)
    JOINT_LIMITS[name] = (float(limits.get("lower")), float(limits.get("upper")))

# The public model gives no shared actuator-response groups. Each of its 31
# physical joints is independent; no grouping from another robot is retained.
TORQUE_LAG_GROUPS = tuple((name,) for name in BODY_JOINT_NAMES)
assert len(BODY_JOINT_NAMES) == 31
assert len(TRACKED_BODY_NAMES) == 32 and TRACKED_BODY_NAMES[0] == "pelvis_link"
assert sorted(n for group in TORQUE_LAG_GROUPS for n in group) == sorted(BODY_JOINT_NAMES)
