"""Evaluate an explicitly perturbed simulation plant without changing training defaults.

These profiles are hypotheses, not identified Open Sprite motor models. All
joint Kp/Kd, policy actions/history, constant effort caps and policy rates remain
unchanged. Only explicitly named plant parameters differ from the checkpoint.
"""
import argparse
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
PROFILES = {
    'baseline': {},
    'delay_fixed_10ms': {'delay_ticks': 5},
    'delay_fixed_20ms': {'delay_ticks': 10},
    '4340_friction_010': {'frictionloss': .1},
    '4340_friction_0339': {'frictionloss': .339229057},
    '4340_viscous_0184': {'viscous_damping': .183972973},
    '4340_armature_070': {'armature_factor': .7},
    '4340_armature_130': {'armature_factor': 1.3},
    'torque_response_5ms': {'torque_tau_s': .005},
    'torque_response_10ms': {'torque_tau_s': .010},
    '4340_speed_curve_48v': {'speed_voltage_factor': 1.0},
    '4340_speed_curve_38v_hypothesis': {'speed_voltage_factor': 38 / 48},
}


def torque_bounds_scalar(velocity, peak, no_load, constant_cap):
    """Illustrative four-quadrant envelope; braking retains the constant cap."""
    if no_load <= 0 or peak <= 0 or constant_cap <= 0:
        raise ValueError('Torque-speed parameters must be positive')
    upper = min(constant_cap, peak * max(0., min(1., 1 - velocity / no_load)))
    lower = -min(constant_cap, peak * max(0., min(1., 1 + velocity / no_load)))
    return lower, upper


def evaluate(args):
    import numpy as np
    import torch
    import evaluate_joint as runner
    import opensprite_tracker.supermimic.env as plant
    from opensprite_tracker.robot import JOINT_CONTROL

    profile = dict(PROFILES[args.profile])
    hardware_path = ROOT / 'configs/runtime/hardware-source.json'
    hardware = json.loads(hardware_path.read_text())
    model_name = 'DM-J4340P-2EC V1.1 (48V)'
    direct = sorted(row['policy_joint'] for row in hardware['motor_map'].values()
                    if row.get('policy_joint') and row['model'] == model_name)
    if not direct or len(direct) != len(set(direct)):
        raise ValueError('Direct-drive motor mapping is absent or ambiguous')
    evidence = dict(profile=args.profile, overrides=profile, selected_direct_joints=direct,
                    hardware_map_sha256=hashlib.sha256(hardware_path.read_bytes()).hexdigest(),
                    hardware_qualified=False,
                    interpretation='Nominal checkpoint contract is checked before explicit diagnostic plant overrides. This is not a nominal-physics score.',
                    exclusions='No parallel ankle/head motor approximation; no motor enabling or runtime changes.',
                    prior_warning='4340 friction is an external older-revision bench prior. Voltage scaling and linear torque-speed envelope are unvalidated hypotheses.')
    original_cfg = plant.make_env_cfg
    original_env = runner.TrackingSim

    def make_cfg(*pos, **kw):
        cfg = original_cfg(*pos, **kw)
        entity = cfg.scene.entities['robot']
        original_spec = entity.spec_fn

        def spec_fn():
            spec = original_spec()
            records = []
            for name in direct:
                joint = spec.joint(name)
                scalar = lambda key: float(np.asarray(getattr(joint, key)).reshape(-1)[0])
                def assign(key, value):
                    current = np.asarray(getattr(joint, key))
                    if current.ndim == 0:
                        setattr(joint, key, value)
                    else:
                        updated = current.copy()
                        updated.flat[0] = value
                        setattr(joint, key, updated)
                before = {key: scalar(key) for key in ('frictionloss', 'damping', 'armature')}
                if 'frictionloss' in profile:
                    assign('frictionloss', profile['frictionloss'])
                if 'viscous_damping' in profile:
                    assign('damping', profile['viscous_damping'])
                if 'armature_factor' in profile:
                    assign('armature', scalar('armature') * profile['armature_factor'])
                records.append(dict(joint=name, before=before,
                    after={key: scalar(key) for key in before}))
            evidence['spec_overrides'] = records
            return spec

        changes = {}
        if 'delay_ticks' in profile:
            changes.update(delay_min_lag=profile['delay_ticks'], delay_max_lag=profile['delay_ticks'])
        if 'torque_tau_s' in profile:
            changes.update(torque_lag_enabled=True,
                           torque_time_constant_range=(profile['torque_tau_s'],) * 2)
        actuators = tuple(replace(a, **changes) for a in entity.articulation.actuators)
        cfg.scene.entities['robot'] = replace(entity, spec_fn=spec_fn,
            articulation=replace(entity.articulation, actuators=actuators))
        return cfg

    class DiagnosticEnv(original_env):
        def __init__(self, *pos, **kw):
            super().__init__(*pos, **kw)
            actuator = self.actuator
            names = [name.rsplit('/', 1)[-1] for name in actuator.target_names]
            for field, source in [('stiffness', 'kp'), ('damping', 'kd'), ('force_limit', 'command_effort_nm')]:
                expected = torch.tensor([getattr(JOINT_CONTROL[n], source) for n in names], device=self.device)
                torch.testing.assert_close(getattr(actuator, field), expected[None].expand(self.N, -1))
            evidence['published_kp_kd_constant_caps_verified'] = True
            model = self.sim.mj_model
            for row in evidence['spec_overrides']:
                ids = [j for j in range(model.njnt) if model.joint(j).name.rsplit('/', 1)[-1] == row['joint']]
                assert len(ids) == 1
                dof = int(model.jnt_dofadr[ids[0]])
                for field, spec_key in [('dof_frictionloss', 'frictionloss'), ('dof_damping', 'damping'), ('dof_armature', 'armature')]:
                    np.testing.assert_allclose(getattr(model, field)[dof], row['after'][spec_key], rtol=1e-6, atol=1e-9)
            evidence['compiled_physics_verified'] = True
            evidence['delay_range_ticks'] = [actuator.cfg.delay_min_lag, actuator.cfg.delay_max_lag]
            evidence['torque_response_tau_range_s'] = list(actuator.cfg.torque_time_constant_range)
            evidence['torque_response_enabled'] = actuator.cfg.torque_lag_enabled
            if 'speed_voltage_factor' in profile:
                # Explicit 4340P v1.1 datasheet hypothesis; not the older 27 Nm prior.
                peak = 40.0
                no_load = 11.72861257 * profile['speed_voltage_factor']
                ids = [names.index(name) for name in direct]
                original_compute = actuator.compute
                evidence['speed_envelope'] = dict(peak_nm=peak, no_load_rad_s=no_load,
                    braking='Constant cap retained; motoring envelope shrinks with speed')

                def compute(cmd):
                    old_peak = actuator.applied_peak.clone()
                    requested = original_compute(cmd)
                    velocity = cmd.vel[:, ids]
                    cap = actuator.force_limit[:, ids]
                    upper = torch.minimum(cap, peak * (1 - velocity / no_load).clamp(0, 1))
                    lower = -torch.minimum(cap, peak * (1 + velocity / no_load).clamp(0, 1))
                    requested[:, ids] = torch.maximum(lower, torch.minimum(upper, requested[:, ids]))
                    actuator.filtered_effort.copy_(requested)
                    actuator.applied_peak.copy_(torch.maximum(old_peak, requested.abs()))
                    return actuator.filtered_effort

                actuator.compute = compute

    plant.make_env_cfg = make_cfg
    runner.TrackingSim = DiagnosticEnv
    try:
        runner.evaluate(args)
        (args.output / 'actuator-sensitivity.json').write_text(json.dumps(evidence, indent=2) + '\n')
        summary_path = args.output / 'summary.json'
        summary = json.loads(summary_path.read_text())
        summary['diagnostic_plant_profile'] = evidence
        summary_path.write_text(json.dumps(summary, indent=2) + '\n')
    finally:
        plant.make_env_cfg = original_cfg
        runner.TrackingSim = original_env


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile', choices=list(PROFILES), required=True)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--motion', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--commands-file', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--seed', type=int, default=101)
    p.add_argument('--device', default='cuda:0')
    p.add_argument('--initial-pose', choices=['reference-quiet', 'controller-default', 'urdf-zero'], default='reference-quiet')
    p.set_defaults(interface='velocity', heldout_training_bank=None)
    evaluate(p.parse_args())
