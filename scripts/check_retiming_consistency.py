import json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from retarget.processing import robot_contract,geometry_quality
model,_,_,_=robot_contract()
old=np.load(ROOT/'data/motions/contact_v5_v11/v5_straight.npz')['qpos']
new=np.load(ROOT/'data/motions/contact_v5_v11_slow2/v5_straight.npz')['qpos']
print(json.dumps(dict(joint_difference=float(abs(old[:,7:]-new[::2,7:]).max()),position_difference=float(abs(old[:,:3]-new[::2,:3]).max()),
    minimum_quaternion_abs_dot=float(abs(np.sum(old[:,3:7]*new[::2,3:7],axis=-1)).min()),
    original_geometry=geometry_quality(model,old),retimed_even_geometry=geometry_quality(model,new[::2]),retimed_all_geometry=geometry_quality(model,new))),flush=True)
