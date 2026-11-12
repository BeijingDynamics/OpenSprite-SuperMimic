"""Retain and evaluate explicit update milestones without changing optimization."""


def checkpoint_milestones(values):
    if not isinstance(values,(list,tuple)) or any(type(v) is not int or v<=0 for v in values):
        raise ValueError('Checkpoint milestones must be positive integer updates')
    return frozenset(values)


def prunable_checkpoints(paths,milestones,keep_recent=4):
    ordered=sorted(paths)
    return [path for path in ordered[1:-keep_recent]
            if int(path.stem.split('_')[-1]) not in milestones]


def evaluation_checkpoint(paths,done,required,*,alive,regular_due):
    ordered=sorted(paths)
    for path in ordered:
        iteration=int(path.stem.split('_')[-1])
        if iteration in required and iteration not in done:return path
    if not ordered:return None
    latest=ordered[-1];iteration=int(latest.stem.split('_')[-1])
    if iteration not in done and (not alive or (iteration>=1000 and regular_due)):return latest
    return None
