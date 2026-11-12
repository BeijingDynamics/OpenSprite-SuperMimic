"""External impulses must not become fictitious autonomous WM dynamics."""
import torch
import pytest
from opensprite_tracker.supermimic.buffer import ReplayBuffer
from opensprite_tracker.supermimic.recovery import force_schedule,validate_force_cases


def test_impulse_boundary_masks_wm_but_preserves_post_impulse_samples_and_actor_history():
    b=ReplayBuffer(6,1,'cpu',state_dim=1,action_dim=1,
                   extra_fields={'wm_discontinuity':((),torch.bool)})
    for t in range(6):
        x=t+(100 if t>=3 else 0)
        b.add(dict(s=torch.tensor([[float(x)]]),s1=torch.tensor([[float(x+1)]]),
                   a=torch.tensor([[float(t)]]),a_prev=torch.tensor([[float(t-1)]]),
                   xi=torch.empty(1,0),r=torch.zeros(1),f=torch.tensor([t]),
                   ep_step=torch.tensor([t],dtype=torch.int32),done=torch.tensor([False]),
                   wm_discontinuity=torch.tensor([t==3])))
    window=b.sample_windows(8,6)
    assert window['mask'].tolist()==[[1,1,1,0,0,0]]*8
    window=b.sample_windows(512,3)
    after=window['s'][:,0,0]==103
    assert after.any() and (window['mask'][after]==1).all()
    starts,history=b.sample_starts(512,4)
    after=starts['f']==3
    assert after.any()
    assert (history['f'][after]==torch.tensor([0,1,2,3])).all()
    assert (history['a_prev'][after,:,0]==torch.tensor([-1.,0.,1.,2.])).all()


def test_force_schedule_releases_at_exact_end_and_does_not_touch_control_cases():
    cases=[dict(forces=[dict(at_seconds=3.,duration_seconds=.2,force_world_n=[0,8,0])]),dict()]
    validate_force_cases(cases,10.)
    assert force_schedule(cases,149,'cpu').abs().sum()==0
    assert force_schedule(cases,150,'cpu')[0,0,1]==8
    assert force_schedule(cases,159,'cpu')[0,0,1]==8
    assert force_schedule(cases,160,'cpu').abs().sum()==0
    assert force_schedule(cases,155,'cpu')[1].abs().sum()==0
    with pytest.raises(ValueError):validate_force_cases(cases,3.1)
