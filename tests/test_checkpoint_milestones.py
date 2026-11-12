"""Matched-update comparisons must survive retention and latest-only polling."""
from pathlib import Path
import pytest
from opensprite_tracker.supermimic.checkpoints import checkpoint_milestones,prunable_checkpoints,evaluation_checkpoint


def paths(*iterations):return [Path(f'model_{i:06d}.pt') for i in iterations]


def test_milestone_survives_recent_checkpoint_pruning():
    candidates=paths(1,5000,6030,6500,7000,7500,8000,8500)
    assert prunable_checkpoints(candidates,{6030})==paths(5000,6500)
    assert prunable_checkpoints(candidates,set())==candidates[1:-4]


def test_required_update_is_selected_even_after_newer_checkpoints_arrive():
    candidates=paths(6030,6200)
    assert evaluation_checkpoint(candidates,set(),{6030},alive=True,regular_due=False)==candidates[0]
    assert evaluation_checkpoint(candidates,{6030},{6030},alive=True,regular_due=False) is None


def test_final_run_drains_available_milestones_then_latest():
    candidates=paths(6030,10000,12000);done=set()
    for expected in candidates:
        found=evaluation_checkpoint(candidates,done,{6030,10000,18000},alive=False,regular_due=True)
        assert found==expected;done.add(int(found.stem.split('_')[-1]))
    assert evaluation_checkpoint(candidates,done,{6030,10000,18000},alive=False,regular_due=True) is None


@pytest.mark.parametrize('invalid',[[0],[-1],[1.5],[True],'6030'])
def test_invalid_milestones_are_rejected(invalid):
    with pytest.raises(ValueError):checkpoint_milestones(invalid)
