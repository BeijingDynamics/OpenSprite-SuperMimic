"""Opt-in Sprite sole geometry experiment; the paper reward remains the default."""
import math
import torch

from .joint_interfaces import MotionTrackingMDP
from .mdp import CONTROL_DT

REVISION = 'sprite_motion_sole_height_v1'


def sole_height_penalty(actual, target, *, std_m, weight):
    """Bounded nonpositive term with no invented contact force or gait clock."""
    return weight * torch.expm1(-((actual-target)/std_m).square()).mean(-1)


class SoleTrackingMDP(MotionTrackingMDP):
    """The full paper motion interface plus a separately identified geometry term."""
    def __init__(self, *args, sole_std_m, sole_weight, **kwargs):
        super().__init__(*args, **kwargs)
        self.sole_std_m = sole_std_m
        self.sole_weight = sole_weight

    def reward(self, S, f, a, a_prev, terms=False):
        original = super().reward(S, f, a, a_prev, terms=terms)
        base, detail = original if terms else (original, None)
        bp, bR, _, _, target_p, target_R = self._bodies_and_target(S, f)
        actual = self.kin.sole_bottom_z(bp[..., self.feet, :], bR[..., self.feet, :, :])
        target = self.kin.sole_bottom_z(target_p[..., self.feet, :], target_R[..., self.feet, :, :])
        modifier = sole_height_penalty(actual, target, std_m=self.sole_std_m, weight=self.sole_weight)
        reward = base + CONTROL_DT*modifier
        if not terms:
            return reward
        detail = dict(detail)
        detail['per_reward/penalties'] = detail['per_reward/penalties'] + modifier
        detail['per_reward/sole_height'] = modifier
        detail['sole_height_rmse_m'] = (actual-target).square().mean(-1).sqrt()
        return reward, detail


def make_motion_mdp(config, *args, **kwargs):
    """Keep old checkpoints exact; reject unknown or incomplete reward settings."""
    extension = config.get('motion_reward_extension')
    if extension is None:
        return MotionTrackingMDP(*args, **kwargs)
    if set(extension) != {'revision', 'std_m', 'weight'} or extension['revision'] != REVISION:
        raise ValueError('Unknown or incomplete motion reward extension')
    std, weight = float(extension['std_m']), float(extension['weight'])
    if not math.isfinite(std) or not .005 <= std <= .05:
        raise ValueError('Sole geometry tolerance must be between5mm and50mm')
    if not math.isfinite(weight) or not 0 < weight <= 1:
        raise ValueError('Sole geometry weight must be positive and at most1')
    return SoleTrackingMDP(*args, sole_std_m=std, sole_weight=weight, **kwargs)
