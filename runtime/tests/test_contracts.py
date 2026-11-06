import copy
from pathlib import Path
import tempfile
import unittest

from open_sprite_runtime.contracts import PolicyContract, RuntimeTiming
from open_sprite_runtime.safety import SafetyState


class RuntimeTimingTests(unittest.TestCase):
    def test_qualified_multirate_plan(self) -> None:
        timing = RuntimeTiming(policy_hz=50, state_hz=500, motor_internal_hz=1000)
        timing.validate()
        self.assertEqual(timing.state_updates_per_policy, 10)
        self.assertEqual(timing.interpolation_steps, 10)

    def test_rejects_non_integer_rate_relationship(self) -> None:
        with self.assertRaises(ValueError):
            RuntimeTiming(policy_hz=50, state_hz=400, motor_internal_hz=1000).validate()

    def test_rejects_policy_target_interpolation(self) -> None:
        with self.assertRaisesRegex(ValueError, "zero-order hold"):
            RuntimeTiming(
                policy_hz=50,
                state_hz=500,
                motor_internal_hz=1000,
                policy_target_semantics="linear_interpolation",
            ).validate()

    def test_default_safety_state_cannot_arm(self) -> None:
        with self.assertRaisesRegex(RuntimeError, "hardware arm blocked"):
            SafetyState().require_armed()


def valid_policy_data() -> dict:
    terms = []
    cursor = 0
    for name, frame_dim, history in (
        ("joint_pos", 31, 8),
        ("joint_vel", 31, 8),
        ("actions", 31, 8),
        ("base_ang_vel", 3, 8),
        ("projected_gravity", 3, 8),
        ("velocity_commands", 3, 0),
    ):
        dim = frame_dim * max(history, 1)
        terms.append(
            {
                "name": name,
                "start": cursor,
                "end": cursor + dim,
                "shape": [dim],
                "frame_dim": frame_dim,
                "history_length": history,
            }
        )
        cursor += dim
    data = {
        "policy_dt": 0.02,
        "physics_dt": 0.002,
        "decimation": 10,
        "action_dim": 31,
        "actor_observation_dim": 795,
        "observation_has_horizontal_base_velocity": False,
        "observation_has_global_position": False,
        "observation_has_global_yaw": False,
        "observation_terms": terms,
        "joint_names": [f"joint_{index}" for index in range(31)],
        "action_formula": "target_joint_pos = action_offset + action_scale * policy_action",
        "base_ang_vel_frame": "root_link_local",
        "projected_gravity_definition": "R_world_to_base @ [0, 0, -1]",
        "quaternion_order": "wxyz",
        "command_layout": ["vx", "vy", "yaw_rate"],
        "deployment_handoff_seconds": 0.04,
        "deployment_handoff_mode": "smoothstep_from_pose_equivalent_action",
        "deployment_initial_velocity_mode": "zero",
        "physical_ankle_differential": {
            "enabled": True,
            "rated_torque_nm": 3.5,
            "peak_torque_nm": 12.5,
            "rated_speed_rad_s": 12.56,
            "no_load_speed_rad_s": 36.2,
            "joint_to_motor_ratio": 1.0,
        },
    }
    for name, value in (
        ("default_joint_pos", 0.0),
        ("action_offset", 0.0),
        ("action_scale", 0.2),
        ("stiffness", 10.0),
        ("damping", 1.0),
        ("effort_limit", 12.5),
        ("velocity_limit", 9.3),
    ):
        data[name] = [value] * 31
    return data


class PolicyContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.timing = RuntimeTiming(policy_hz=50, state_hz=500, motor_internal_hz=1000)

    def contract(self, data: dict) -> PolicyContract:
        return PolicyContract(path=None, data=data)  # type: ignore[arg-type]

    def test_accepts_complete_50hz_hardware_contract(self) -> None:
        self.contract(valid_policy_data()).validate_for_hardware(self.timing)

    def test_rejects_global_or_horizontal_state(self) -> None:
        for field in (
            "observation_has_horizontal_base_velocity",
            "observation_has_global_position",
            "observation_has_global_yaw",
        ):
            with self.subTest(field=field):
                data = valid_policy_data()
                data[field] = True
                with self.assertRaises(ValueError):
                    self.contract(data).validate_for_hardware(self.timing)

    def test_rejects_observation_slice_or_order_drift(self) -> None:
        data = valid_policy_data()
        data["observation_terms"] = copy.deepcopy(data["observation_terms"])
        data["observation_terms"][1]["start"] += 1
        data["observation_terms"][3]["name"] = "base_lin_vel"
        with self.assertRaisesRegex(ValueError, "not contiguous"):
            self.contract(data).validate_for_hardware(self.timing)

    def test_rejects_joint_vector_or_ankle_contract_drift(self) -> None:
        data = valid_policy_data()
        data["action_scale"] = data["action_scale"][:-1]
        data["physical_ankle_differential"]["peak_torque_nm"] = 25.0
        with self.assertRaisesRegex(ValueError, "action_scale"):
            self.contract(data).validate_for_hardware(self.timing)


def supermimic_policy_data() -> dict:
    data = valid_policy_data()
    data["schema"] = "sprite_supermimic_joint_velocity_v1"
    data["normalization"] = "embedded_running_norm"
    data["hardware_qualified"] = False
    data["physical_ankle_differential"] = {"enabled": False}
    cursor = 0
    for term in data["observation_terms"]:
        history = 10 if term["history_length"] else 0
        dim = term["frame_dim"] * max(history, 1)
        term.update(history_length=history, start=cursor, end=cursor + dim,
                    shape=[dim], history_order="oldest_to_newest", flatten_history_dim=True,
                    scale=None, clip=None)
        cursor += dim
    data["actor_observation_dim"] = cursor
    return data


class SuperMimicReplayContractTests(unittest.TestCase):
    def contract(self, data=None):
        return PolicyContract(Path("contract.json"), supermimic_policy_data() if data is None else data)

    def test_993_profile_is_replay_only(self):
        contract = self.contract()
        contract.validate_for_replay("supermimic_v5_10f")
        self.assertEqual(contract.data["actor_observation_dim"], 993)
        self.assertEqual(contract.maximum_history_length, 10)
        with self.assertRaisesRegex(ValueError, "offline-only"):
            contract.validate_for_hardware(RuntimeTiming(50, 500, 1000))

    def test_wrong_or_unknown_profile_rejected(self):
        for profile in ("sprite_8f", "auto", "unknown"):
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                self.contract().validate_for_replay(profile)
        with self.assertRaisesRegex(ValueError, "schema"):
            self.contract(valid_policy_data()).validate_for_replay("supermimic_v5_10f")
        self.contract(valid_policy_data()).validate_for_replay("sprite_8f")

    def test_malformed_history_and_external_normalization_rejected(self):
        for field, value in (("history_length", 8), ("history_order", "newest_to_oldest"),
                             ("start", 1), ("scale", .1), ("flatten_history_dim", False)):
            with self.subTest(field=field):
                data = supermimic_policy_data()
                data["observation_terms"][0][field] = value
                with self.assertRaises(ValueError):
                    self.contract(data).validate_for_replay("supermimic_v5_10f")
        data = supermimic_policy_data()
        data["normalization"] = "external"
        with self.assertRaisesRegex(ValueError, "embedded"):
            self.contract(data).validate_for_replay("supermimic_v5_10f")

    def test_replay_does_not_relax_other_hardware_layouts(self):
        data = valid_policy_data()
        data["observation_terms"][0]["history_length"] = 10
        with self.assertRaisesRegex(ValueError, "history_length mismatch"):
            self.contract(data).validate_for_hardware(RuntimeTiming(50, 500, 1000))

    def test_fixture_is_typed_bounded_and_immutable(self):
        import numpy as np
        from open_sprite_runtime.supermimic_replay import write_fixture
        contract = self.contract()
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "input.bin"
            inputs = np.zeros((2, 71))
            write_fixture(target, contract, inputs, 2)
            self.assertEqual(target.read_bytes()[:8], b"SPMIM001")
            self.assertEqual(target.stat().st_size, 32 + 3 * 31 * 8 + 2 * 71 * 8)
            with self.assertRaises(FileExistsError):
                write_fixture(target, contract, inputs, 2)
            inputs[0, 0] = np.nan
            with self.assertRaises(ValueError):
                write_fixture(Path(folder) / "invalid.bin", contract, inputs, 2)


if __name__ == "__main__":
    unittest.main()
