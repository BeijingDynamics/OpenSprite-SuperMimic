"""Versioned contact-aware UMR solver tuning; reference QA thresholds unchanged."""
import json
from pathlib import Path
root=Path(__file__).resolve().parents[1];path=root/'configs/umr/opensprite.json'
config=json.loads(path.read_text());config['solver'].update(
    iters=8,pose_init_iters=240,
    ground_contact_map_cost=150.,ground_contact_anchor_cost=2000.,
    ground_contact_map_threshold=.025,ground_contact_map_snap_threshold=.008,
    robot_self_penetration_hard_slack=False,
)
path.write_text(json.dumps(config,indent=2))
print(json.dumps(config['solver'],indent=2))
