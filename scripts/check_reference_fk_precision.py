import json,sys,torch
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.supermimic.robot import make_kinematics
from opensprite_tracker.supermimic.motion import MotionLib
torch.set_num_threads(4)
for precision in ['highest','high']:
    torch.set_float32_matmul_precision(precision)
    for device in ['cpu','cuda:0']:
        motion=MotionLib(ROOT/'data/motions/contact_v5_v11/v5_locomotion.npz',make_kinematics(device,True),device)
        print(json.dumps(dict(precision=precision,device=device,errors=motion.source_fk_error)),flush=True)
