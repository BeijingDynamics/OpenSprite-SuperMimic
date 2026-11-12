"""Verify raw BVH coordinates against the actual native Geno FBX source path."""
import argparse,hashlib,json,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT),str(ROOT/'third_party/umr/scripts')]
import nr_source
from retarget.bvh_tools.bvh_adapters import read_transforms,inspect_bvh,validate_geno_skeleton,bvh_skeleton_signature

NATIVE_Y_UP_TO_Z_UP=np.array([[1.,0,0],[0,0,-1],[0,1,0]])


def verify_source_frame(path):
    path=Path(path);info=inspect_bvh(path)
    validate_geno_skeleton(info,bvh_skeleton_signature(path).local_rest_offsets_m)
    fbx=ROOT/'assets/geno/Geno.fbx';asset=nr_source.load_fbx_asset(fbx)
    asset_names=asset['bone_match_names'].tolist()
    _,names,_,_,_,positions,rotations=read_transforms(path)
    ids=np.arange(len(positions));worlds=nr_source._bone_world_matrices(asset,dict(nr_bvh_path=str(path)),ids)
    common=[n for n in asset_names if n in names];a=[asset_names.index(n) for n in common];b=[names.index(n) for n in common]
    native=worlds[:,:,:3,3][:,a]*nr_source.NR_UNIT_SCALE;direct=positions[:,b]/100
    difference=np.linalg.norm(native-direct,axis=-1);hip=common.index('Hips')
    rotation=worlds[:,a[hip],:3,:3];rotation=rotation/np.linalg.norm(rotation,axis=-2,keepdims=True)
    report=dict(source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),fbx_sha256=hashlib.sha256(fbx.read_bytes()).hexdigest(),
                frames=len(ids),bones=len(common),root_position_max_error_m=float(difference[:,hip].max()),
                all_bone_position_max_error_m=float(difference.max()),all_bone_position_rms_m=float(np.sqrt(np.mean(difference**2))),
                root_orientation_matrix_max_error=float(abs(rotation-rotations[:,names.index('Hips')]).max()),
                native_source_y_up_to_z_up_matrix=NATIVE_Y_UP_TO_Z_UP.tolist(),
                scope='All source frames before robot IK; non-root FBX bind offsets can differ slightly from BVH position channels.')
    report['passed']=(report['root_position_max_error_m']<1e-6 and report['root_orientation_matrix_max_error']<1e-6 and
                      report['all_bone_position_rms_m']<.001 and report['all_bone_position_max_error_m']<.005)
    if not report['passed']:raise ValueError('Source BVH/native FBX frame verification failed: '+json.dumps(report))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=verify_source_frame(a.source);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
