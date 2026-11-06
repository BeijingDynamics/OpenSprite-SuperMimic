"""Tests the paper's central gradient routing, beyond matching network dimensions."""
import sys,json
from pathlib import Path
import torch
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.joint_interfaces import SharedController,MotionActor,VelocityActor

def test():
    torch.manual_seed(1)
    shared=SharedController(hidden=(64,32));motion=MotionActor(shared);velocity=VelocityActor(shared)
    h=torch.randn(4,10,99,requires_grad=True);goal=torch.randn(4,770);cmd=torch.randn(4,3)
    motion(h,goal).square().sum().backward()
    assert any(p.grad is not None and p.grad.abs().sum()>0 for p in shared.motion_encoder.parameters())
    assert any(p.grad is not None and p.grad.abs().sum()>0 for p in shared.decoder.parameters())
    assert all(p.grad is None for p in shared.velocity_encoder.parameters())
    shared.zero_grad(set_to_none=True);h.grad=None
    for p in shared.decoder.parameters():p.requires_grad_(False)
    action=velocity(h,cmd)
    with torch.no_grad():target=shared.motion_latent(goal)
    latent=shared.velocity_latent(h,cmd)
    loss=action.square().sum()+.1*(latent-target).square().mean()
    loss.backward()
    assert all(p.grad is None for p in shared.decoder.parameters())
    assert all(p.grad is None for p in shared.motion_encoder.parameters())
    assert any(p.grad is not None and p.grad.abs().sum()>0 for p in shared.velocity_encoder.parameters())
    assert h.grad is not None and torch.isfinite(h.grad).all() and h.grad.abs().sum()>0
    assert not ({id(p) for p in velocity.parameters()} & {id(p) for p in shared.decoder.parameters()})
    assert motion.dec is shared.decoder and velocity.shared.decoder is shared.decoder
    print(json.dumps(dict(passed=True,motion_updates_encoder_decoder=True,velocity_updates_only_velocity_encoder=True,
                         input_gradient_through_frozen_decoder=True,reference_latent_target_detached=True,history=10)))
if __name__=='__main__':test()
