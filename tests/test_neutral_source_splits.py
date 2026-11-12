"""Source time scaling and alternative solves must not create split leakage."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pack_reviewed_neutral import validate_source_intervals


def row(begin,end,split='candidate_training',source='a'):
    return dict(begin=begin,end=end,split=split,source_sha256=source,fps=60)


def test_adjacent_training_clips_keep_disjoint_source_time():
    validate_source_intervals([row(100,400),row(400,700)],[row(900,1200,'validation')],'candidate_training')


def test_retimed_or_retargeted_alternative_is_not_an_independent_clip():
    with pytest.raises(ValueError,match='Overlapping'):
        validate_source_intervals([row(100,400),row(200,500)],[],'candidate_training')


def test_opposite_split_keeps_one_second_of_source_margin():
    with pytest.raises(ValueError,match='opposite split'):
        validate_source_intervals([row(100,400)],[row(450,700,'validation')],'candidate_training')
    validate_source_intervals([row(100,400)],[row(460,700,'validation')],'candidate_training')


def test_different_sources_do_not_share_a_time_axis():
    validate_source_intervals([row(100,400)],[row(100,400,'validation','b')],'candidate_training')


def test_validation_candidate_cannot_be_selected_as_training():
    with pytest.raises(ValueError,match='different split'):
        validate_source_intervals([row(100,400,'validation')],[],'candidate_training')
