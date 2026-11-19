"""Observe 500 Hz simulated PD torques without modifying control or dynamics."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import torch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--motion', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--commands-file', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=20.)
    parser.add_argument('--seed', type=int, default=101)
    args = parser.parse_args()
    sys.path[:0] = [str(args.source_root), str(args.source_root / 'scripts')]
    import evaluate_joint as runner
    from opensprite_tracker.robot import JOINT_CONTROL, BODY_JOINT_NAMES
    original = runner.TrackingSim
    traces = {}
    count = 0
    expected = round(args.seconds / .002)
    metadata = {}

    class ObservedEnv(original):
        def __init__(self, *pos, **kw):
            super().__init__(*pos, **kw)
            actuator = self.actuator
            ids = self.act_ids
            metadata.update(joint_names=list(BODY_JOINT_NAMES),
                controls={n: vars(JOINT_CONTROL[n]) for n in BODY_JOINT_NAMES},
                delay_ticks=[actuator.cfg.delay_min_lag, actuator.cfg.delay_max_lag],
                torque_lag_enabled=actuator.cfg.torque_lag_enabled)
            for key in ['q', 'qd', 'target', 'p_term', 'd_term', 'raw', 'applied']:
                traces[key] = torch.empty((expected, self.N, len(ids)), device=self.device)
            compute = actuator.compute

            def observed(cmd):
                nonlocal count
                if count >= expected:
                    raise RuntimeError('Unexpected actuator invocation count')
                assert not torch.count_nonzero(cmd.velocity_target)
                assert not torch.count_nonzero(cmd.effort_target)
                p = actuator.stiffness * (cmd.position_target - cmd.pos)
                d = -actuator.damping * cmd.vel
                result = compute(cmd)
                fields = dict(q=cmd.pos, qd=cmd.vel, target=cmd.position_target,
                              p_term=p, d_term=d, raw=p+d, applied=result)
                for key, value in fields.items():
                    traces[key][count].copy_(value[:, ids])
                count += 1
                return result

            actuator.compute = observed

    runner.TrackingSim = ObservedEnv
    args.interface = 'velocity'
    args.device = 'cuda:0'
    args.initial_pose = 'reference-quiet'
    args.heldout_training_bank = None
    runner.evaluate(args)
    assert count == expected, (count, expected)
    arrays = {key: value.cpu().numpy() for key, value in traces.items()}
    with np.load(args.output / 'rollout.npz') as rollout:
        for key, field in [('raw', 'raw_torque_peak'), ('applied', 'applied_torque_peak')]:
            peak = abs(arrays[key]).reshape(-1, 10, *arrays[key].shape[1:]).max(1)
            np.testing.assert_allclose(peak, rollout[field], atol=1e-6, rtol=1e-5)
        cap = np.array([JOINT_CONTROL[n].command_effort_nm for n in BODY_JOINT_NAMES])
        np.testing.assert_allclose(arrays['applied'], np.clip(arrays['raw'], -cap, cap), atol=2e-6)
    np.savez_compressed(args.output / 'torque-500hz.npz', **arrays)
    metadata.update(samples=count,physics_dt=.002,policy_dt=.02,
        checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
        peak_log_consistency_passed=True,clipped_pd_consistency_passed=True,
        hardware_qualified=False,observer_only=True,
        limitations='Simulated motor-input torque, not measured shaft torque. No friction is applied by this observer.')
    (args.output / 'torque-contract.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print('TORQUE_AUDIT_COMPLETE', count, flush=True)


if __name__ == '__main__':
    main()
