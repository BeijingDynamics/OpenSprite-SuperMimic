"""Admit a pinned, visually reviewed PM01 baseline for simulator research only.

This is a distinct qualification from the LAFAN contact-refinement protocol.
Preserve all original audits; report low clearance and uncertain contact labels.
Never modify the reference pose to satisfy a heuristic clearance threshold.
"""
import argparse,hashlib,json,shutil,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from opensprite_tracker.robot import ASSET_REVISION,JOINT_CONTROL
from opensprite_tracker.supermimic.robot import make_kinematics
from opensprite_tracker.supermimic.motion import MotionLib
from scripts.audit_g71_reference import EXPECTED


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--audit-dir',type=Path,required=True)
    p.add_argument('--visual-review',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    bank=a.audit_dir/'g71_walk_v5_50hz.npz'
    audit=json.loads((a.audit_dir/'audit.json').read_text())
    review=json.loads(a.visual_review.read_text())
    sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    digest=sha(bank)
    if audit['source_sha256']!=EXPECTED or audit['motion_bank_sha256']!=digest:
        raise ValueError('Pinned PM01 audit/source identity mismatch')
    if review.get('motion_bank_sha256')!=digest or review.get('approved_style') is not True:
        raise ValueError('A hash-bound operator style review is required')
    if audit['asset_revision']!=ASSET_REVISION:raise ValueError('Wrong robot revision')
    kin=make_kinematics('cpu',True);motion=MotionLib(bank,kin,'cpu',goal_mode='robot_relative_v2',goal_stride=5)
    errors=[]
    if max(motion.source_fk_error[k]['max_abs'] for k in ('body_p','body_R'))>1e-4:
        errors.append('Current-model FK mismatch')
    for key in ('q','qd','root_p','root_R','root_v','root_w'):
        if not np.isfinite(getattr(motion,key).numpy()).all():errors.append('Nonfinite '+key)
    limit=np.array([JOINT_CONTROL[n].urdf_velocity_rad_s for n in kin.joint_names])
    ratio=float(np.max(np.abs(motion.qd.numpy())/limit))
    if ratio>1.01:errors.append('Joint speed exceeds current official limits')
    if audit['max_hard_joint_limit_excess_rad']>1e-6:errors.append('Joint hard limit violation')
    geom=audit['final_geometry']
    if geom['minimum_collision_z_m']<-.002:errors.append('Ground penetration')
    if geom['self_minimum_signed_distance_m']<-.01:errors.append('Self penetration')
    if errors:raise ValueError(errors)
    a.output.mkdir(parents=True,exist_ok=False)
    target=a.output/'pm01_g71_v5_50hz.npz';shutil.copy2(bank,target)
    shutil.copy2(a.audit_dir/'audit.json',a.output/'original-audit.json')
    shutil.copy2(a.visual_review,a.output/'visual-review.json')
    report=dict(accepted_for_training=True,visual_review_passed=True,
        qualification='pm01_g71_simulator_baseline_v1',asset_revision=ASSET_REVISION,
        motion_bank_sha256=digest,source_sha256=EXPECTED,visual_review_sha256=sha(a.visual_review),
        source_fk_error=motion.source_fk_error,max_joint_velocity_ratio=ratio,
        final_geometry=geom,frames=motion.num_frames,duration_s=audit['duration_s'],
        clip_names=motion.clip_names,independent_motion_count=1,
        original_audit_preserved=True,original_heuristic_errors=audit['errors'],
        low_clearance_m=audit['max_sole_height_m'],height_proxy_rms_speed_m_s=audit['stance_foot_rms_speed_m_s'],
        admission_reason='Operator accepted the preserved PM01 whole-body style. Joint/FK/geometry checks pass. A universal 20mm swing threshold and height-only support labels are not hard validity gates for this low-clearance reference.',
        limitations=['Kinematic reference admission for a simulator learning baseline, not exact dynamic feasibility.',
                    'Only one steady-walk clip; no independent source holdout or claimed reference start/turn coverage.',
                    'Low clearance and contact-phase ambiguity remain explicit physical-evaluation risks.',
                    'No hardware admission; no runtime gain or protection change.'],errors=[])
    target.with_suffix('.qa.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
