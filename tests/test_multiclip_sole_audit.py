"""A short clip must not inherit later motion or failure from shared rollouts."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import audit_sole_tracking as audit


def test_packed_bank_reads_named_pelvis_and_named_joints():
    names=list(reversed(audit.BODY_JOINT_NAMES));joint=np.arange(31)[None,:]
    ref=dict(joint_names=np.array(names),joint_pos=joint,body_names=np.array(['head_yaw_link','pelvis_link']),
             body_pos_w=np.array([[[9.,9.,9.],[1.,2.,3.]]]),body_quat_w=np.array([[[0.,0.,0.,1.],[1.,0.,0.,0.]]]))
    pose=audit.reference_poses(ref)
    np.testing.assert_array_equal(pose[0,:7],[1.,2.,3.,1.,0.,0.,0.])
    np.testing.assert_array_equal(pose[0,7:],np.arange(30,-1,-1))


def test_short_motion_clip_excludes_later_clearance_and_failure(tmp_path,monkeypatch):
    poses=np.zeros((10,1,38));poses[5:,:,2]=.2
    failed=np.zeros((10,1),bool);failed[7:]=True
    path=tmp_path/'rollout.npz'
    np.savez(path,qpos=poses,failed=failed,fps=50.,contact_force_mean_n=np.ones((10,1,2))*20)
    def geometry(poses):
        points=np.zeros((len(poses),2,8,3));points[...,2]=poses[:,2,None,None]
        return points
    monkeypatch.setattr(audit,'geometry',geometry)
    full=audit.physical_trace(path,0,0)
    short=audit.physical_trace(path,0,0,active_steps=5)
    assert full['sole_height_max_m']==[.2,.2] and not full['survived']
    assert short['sole_height_max_m']==[0.,0.] and short['survived']
    assert short['samples']==5 and short['scoring_end_seconds']==.1
