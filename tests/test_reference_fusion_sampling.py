"""Check weighted clip mass and explicit reference-transfer boundaries."""
from types import SimpleNamespace
import pytest
import torch
from opensprite_tracker.supermimic.sampling import ReturnSampler
from opensprite_tracker.supermimic.reference_transfer import validate_reference_transfer

def motion():
    return SimpleNamespace(num_bins=5,root_p=torch.zeros(1,3),clip_end=torch.tensor([119,149]),
        bin_clip=torch.tensor([0,0,0,1,1]),bin_start=torch.tensor([0,50,100,120,140]),bin_end=torch.tensor([50,100,119,140,149]))

def test_weighted_mass_survives_adaptive_returns_and_past():
    s=ReturnSampler(motion(),clip_weights=[.8,.2]);s.returns.copy_(torch.tensor([0.,99.,-99.,60.,0.]));s.past.copy_(torch.tensor([.01,.01,.01,.1,.87]))
    p=s.probabilities()
    assert torch.allclose(p[:3].sum(),torch.tensor(.8))
    assert torch.allclose(p[3:].sum(),torch.tensor(.2))
    torch.manual_seed(42);frames,bins=s(20000)
    assert .78<float((bins<3).float().mean())<.82
    assert (frames<s.motion.bin_end[bins]).all()

def test_default_preserves_equal_clip_mass():
    s=ReturnSampler(motion());p=s.probabilities()
    assert torch.allclose(p[:3].sum(),torch.tensor(.5))
    assert torch.allclose(p[3:].sum(),torch.tensor(.5))

@pytest.mark.parametrize('weights',[[0,1],[-1,2],[1],[1,float('nan')],[1,float('inf')]])
def test_invalid_weights_fail(weights):
    with pytest.raises(ValueError):ReturnSampler(motion(),clip_weights=weights)

def test_bank_change_requires_exact_declaration():
    assert validate_reference_transfer('a','a',None) is False
    with pytest.raises(ValueError):validate_reference_transfer('a','b',None)
    declaration=dict(revision='reviewed_reference_transfer_v1',source_motion_sha256='a',target_motion_sha256='b',reset_both_critics=True)
    assert validate_reference_transfer('a','b',declaration)
    with pytest.raises(ValueError):validate_reference_transfer('a','c',declaration)
    declaration['reset_both_critics']=False
    with pytest.raises(ValueError):validate_reference_transfer('a','b',declaration)
