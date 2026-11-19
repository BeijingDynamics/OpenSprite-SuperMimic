"""Explicit bank changes require identified source and destination references."""

def validate_reference_transfer(old_digest, new_digest, declaration):
    if old_digest == new_digest:
        if declaration is not None:
            raise ValueError('Reference-change declaration supplied for an unchanged bank')
        return False
    expected = {'revision':'reviewed_reference_transfer_v1',
                'source_motion_sha256':old_digest, 'target_motion_sha256':new_digest,
                'reset_both_critics':True}
    if declaration != expected:
        raise ValueError('Changed reference needs an exact, explicit transfer declaration')
    return True
