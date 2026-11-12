# SPDX-License-Identifier: MIT
# MIT-licensed algorithm utility; see THIRD_PARTY_LICENSE.txt.
# Copyright (c) 2026 ETH Zurich, Chong Zhang
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Configurable SuperMimic dynamics, deterministic actor and value networks.

Freeze WM/critic *parameters* for actor BPTT, not their forward computations:
the action/state, survival, and terminal-value paths must remain differentiable.
Running statistics update only through explicit update() calls.
"""

from collections.abc import Callable, Sequence

import torch
from torch import nn

from .rotations import so3_exp, yaw_mat
from .state import NUM_J, invariant_features, state_delta

AMP_DTYPE: torch.dtype | None = None


def mlp(inp, hidden, out, act=nn.ELU, layer_norm=False):
    layers, d = [], inp
    for h in hidden:
        layers.append(nn.Linear(d, h))
        if layer_norm:
            layers.append(nn.LayerNorm(h))
        layers.append(act())
        d = h
    layers.append(nn.Linear(d, out))
    return nn.Sequential(*layers)


class RunningNorm(nn.Module):
    def __init__(self, dim, max_count=2e8, eps=1e-2):
        super().__init__()
        self.register_buffer("mean", torch.zeros(dim))
        self.register_buffer("var", torch.ones(dim))
        self.register_buffer("count", torch.zeros(()))
        self.max_count, self.eps = max_count, eps

    @torch.no_grad()
    def update(self, x):
        if self.count >= self.max_count:
            return
        x = x.reshape(-1, self.mean.numel()).to(self.mean.dtype)
        if x.shape[0] == 0:
            return
        n = x.shape[0]
        mean, var = x.mean(0), x.var(0, unbiased=False)
        total = self.count + n
        delta = mean - self.mean
        new_var = (
            self.var * self.count + var * n + delta.square() * self.count * n / total
        ) / total
        self.mean.add_(delta * n / total)
        self.var.copy_(new_var)
        self.count.copy_(total)

    def std(self):
        return (self.var + self.eps**2).sqrt()

    def forward(self, x):
        return (x - self.mean) / self.std()


def run_mlp(net, x):
    # Integration and analytic heads remain in fp32; CPU tests use normal dtype.
    if AMP_DTYPE is None or x.device.type != "cuda":
        return net(x)
    with torch.autocast(device_type="cuda", dtype=AMP_DTYPE):
        return net(x).float()


class WorldModel(nn.Module):
    """Deterministic local increments conditioned on actual normalized context.

    h is always an input when memory_dim > 0. Normally its delta is predicted.
    For deterministic memory, set predict_memory=False and pass a differentiable
    memory_step(S, action, xi_normalized), or provide memory_next to every step.
    The action argument must match the command actually logged by the adapter.
    """

    def __init__(
        self,
        hidden: Sequence[int] = (2048, 2048, 2048),
        *,
        num_joints: int = NUM_J,
        action_dim: int | None = None,
        context_dim: int = 0,
        memory_dim: int = 0,
        predict_memory: bool = True,
        memory_step: Callable | None = None,
        dynamics_mode: str = "delta_v1",
        dt: float = 0.02,
        substeps: int = 10,
    ):
        super().__init__()
        if num_joints <= 0 or context_dim < 0 or memory_dim < 0:
            raise ValueError("invalid joint/context/memory dimensions")
        self.num_joints = num_joints
        self.action_dim = num_joints if action_dim is None else action_dim
        self.context_dim, self.memory_dim = context_dim, memory_dim
        self.predict_memory = predict_memory
        self.predict_memory_dim = memory_dim if predict_memory else 0
        self.memory_step = memory_step
        if dynamics_mode not in ("delta_v1", "acceleration_v2") or dt <= 0 or substeps < 1:
            raise ValueError("invalid dynamics integration configuration")
        self.dynamics_mode = dynamics_mode
        self.dt, self.substep_coeff = dt, (substeps + 1) / (2 * substeps)
        self.state_dim = 13 + 2 * num_joints + memory_dim
        self.inv_dim = 13 + 2 * num_joints + memory_dim
        self.in_dim = self.inv_dim + self.action_dim + context_dim
        self.physical_output_dim = (6 + num_joints if dynamics_mode == "acceleration_v2"
                                    else 12 + 2 * num_joints)
        self.out_dim = self.physical_output_dim + self.predict_memory_dim
        self.net = mlp(self.in_dim, hidden, self.out_dim,
                       act=nn.GELU if dynamics_mode == "acceleration_v2" else nn.SiLU,
                       layer_norm=True)
        nn.init.zeros_(self.net[-1].weight)
        nn.init.zeros_(self.net[-1].bias)
        self.in_norm = RunningNorm(self.in_dim)
        self.out_norm = RunningNorm(self.out_dim, eps=1e-4)

    def _check_state(self, S):
        if (
            len(S) not in (6, 7)
            or S[4].shape[-1] != self.num_joints
            or S[5].shape[-1] != self.num_joints
        ):
            raise ValueError("state joint layout does not match world model")
        actual = S[6].shape[-1] if len(S) == 7 else 0
        if actual != self.memory_dim:
            raise ValueError("state h dimension does not match memory_dim")

    def inputs(self, S, a, xi_n=None):
        self._check_state(S)
        if a.shape[-1] != self.action_dim:
            raise ValueError("action dimension does not match world model")
        if xi_n is None:
            if self.context_dim:
                raise ValueError("recorded normalized context is required")
            xi_n = a.new_empty(a.shape[:-1] + (0,))
        if xi_n.shape[-1] != self.context_dim:
            raise ValueError("context dimension does not match world model")
        return torch.cat((invariant_features(S), a, xi_n), -1)

    def targets(self, S, S1):
        self._check_state(S)
        self._check_state(S1)
        if self.dynamics_mode == "delta_v1":
            return state_delta(S, S1, include_memory=self.predict_memory)
        Rt = S[1].transpose(-1, -2)
        parts = [(Rt @ (S1[2] - S[2])[..., None]).squeeze(-1),
                 (Rt @ (S1[3] - S[3])[..., None]).squeeze(-1), S1[5] - S[5]]
        if self.predict_memory_dim:
            parts.append(S1[6] - S[6])
        return torch.cat(parts, -1)

    def nominal_state_scale(self):
        """Physical tolerances plus explicit normalized-memory tolerances."""
        return self.out_norm.mean.new_tensor(
            [0.01] * 3 + [0.02] * 3 + [0.1] * 3 + [0.3] * 3
            + [0.1] * self.num_joints + [3.14] * self.num_joints
            + [0.1] * self.predict_memory_dim)

    def weighted_sq_error(self, predicted, truth):
        """C. Zhang v2 physical loss, extended to Sprite's normalized memory.

        Root velocities are supervised explicitly. Memory retains the recorded
        v1 state semantics; it is not silently discarded by the smaller output.
        """
        self._check_state(predicted)
        self._check_state(truth)
        parts = [((predicted[0] - truth[0]) / 0.01).square(),
                 ((predicted[1] - truth[1]).flatten(-2) * (0.5 ** 0.5) / 0.02).square(),
                 ((predicted[2] - truth[2]) / 0.1).square(),
                 ((predicted[3] - truth[3]) / 0.3).square(),
                 ((predicted[4] - truth[4]) / 0.1).square(),
                 ((predicted[5] - truth[5]) / 3.14).square()]
        if self.predict_memory_dim:
            parts.append(((predicted[6] - truth[6]) / 0.1).square())
        return torch.cat(parts, -1)

    def state_error(self, S_hat, S):
        self._check_state(S_hat)
        self._check_state(S)
        return state_delta(S, S_hat, include_memory=self.predict_memory)

    def step(self, S, a, xi_n=None, *, memory_next=None):
        y = run_mlp(self.net, self.in_norm(self.inputs(S, a, xi_n)))
        d = y * self.out_norm.std() + self.out_norm.mean
        p, R, v, w, q, qd = S[:6]
        if self.dynamics_mode == "acceleration_v2":
            dv_b, dw_b, dqd = d[..., :self.physical_output_dim].split((3, 3, self.num_joints), -1)
            dv = (R @ dv_b[..., None]).squeeze(-1)
            dw = (R @ dw_b[..., None]).squeeze(-1)
            c, dt = self.substep_coeff, self.dt
            S1 = (p + (v + c * dv) * dt, so3_exp((w + c * dw) * dt) @ R,
                  v + dv, w + dw, q + (qd + c * dqd) * dt, qd + dqd)
        else:
            sizes = (3, 3, 3, 3, self.num_joints, self.num_joints)
            dp, dth, dv, dw, dq, dqd = d[..., :self.physical_output_dim].split(sizes, -1)
            S1 = (p + (yaw_mat(R) @ dp.unsqueeze(-1)).squeeze(-1), R @ so3_exp(dth),
                  v + (R @ dv.unsqueeze(-1)).squeeze(-1),
                  w + (R @ dw.unsqueeze(-1)).squeeze(-1), q + dq, qd + dqd)
        if self.memory_dim:
            if self.predict_memory:
                if memory_next is not None:
                    raise ValueError("memory_next conflicts with predicted memory")
                h1 = S[6] + d[..., -self.memory_dim :]
            else:
                if memory_next is None:
                    if self.memory_step is None:
                        raise ValueError(
                            "deterministic h needs memory_step or memory_next"
                        )
                    memory_next = self.memory_step(S, a, xi_n)
                h1 = memory_next
                if h1.shape != S[6].shape:
                    raise ValueError("next memory shape does not match h")
            S1 += (h1,)
        elif len(S) == 7:
            S1 += (S[6],)  # preserve an explicitly supplied empty h
        return S1

    def forward(self, S, a, xi_n=None, **kwargs):
        return self.step(S, a, xi_n, **kwargs)


class Policy(nn.Module):
    def __init__(
        self,
        obs_dim=105,
        goal_dim=770,
        hist=10,
        z_dim=32,
        *,
        action_dim=NUM_J,
        hidden=(2048, 1024, 512, 256),
        encoder_hidden=None,
        decoder_hidden=None,
    ):
        super().__init__()
        self.hist, self.obs_dim, self.goal_dim, self.action_dim = (
            hist,
            obs_dim,
            goal_dim,
            action_dim,
        )
        self.enc = mlp(
            goal_dim, hidden if encoder_hidden is None else encoder_hidden, z_dim
        )
        self.dec = mlp(
            z_dim + obs_dim * hist,
            hidden if decoder_hidden is None else decoder_hidden,
            action_dim,
        )
        with torch.no_grad():
            self.dec[-1].weight.mul_(0.01)
            self.dec[-1].bias.zero_()
        self.obs_norm, self.goal_norm = RunningNorm(obs_dim), RunningNorm(goal_dim)

    def forward(self, obs_hist, goal):
        z = torch.tanh(run_mlp(self.enc, self.goal_norm(goal)))
        o = self.obs_norm(obs_hist).flatten(-2)
        return run_mlp(self.dec, torch.cat((z, o), -1))


class Critic(nn.Module):
    def __init__(self, in_dim, hidden=(256, 128), *, normalized_gelu=False):
        super().__init__()
        self.net = mlp(in_dim, hidden, 1,
                       act=nn.GELU if normalized_gelu else nn.ELU,
                       layer_norm=normalized_gelu)
        self.norm = RunningNorm(in_dim)

    def forward(self, x):
        return run_mlp(self.net, self.norm(x)).squeeze(-1)
