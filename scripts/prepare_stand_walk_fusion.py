"""Select immutable reviewed walk/stand samples and recheck current-model geometry."""
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
import mujoco
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from opensprite_tracker.robot import ROBOT_XML, ASSET_REVISION

FIELDS = ('joint_pos','joint_vel','body_pos_w','body_quat_w','body_lin_vel_w','body_ang_vel_w')
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    dest = ROOT/'outputs/reference-fusion-20261118/reference'
    dest.mkdir(exist_ok=False)
    sources = [
        (ROOT/'outputs/retrain-20261114/seed45-lr-continuation/reference/pm01_clearance_v1.npz', None, 'pm01_clearance_walk'),
        (ROOT/'outputs/retrain-20261112/neutral-research/outputs/neutral-reviewed-library-20261112/candidate_training/neutral_locomotion.npz', 'Neutral_ID_001135_001495_stand', 'Neutral_ID_001135_001495_stand'),
    ]
    chunks={k:[] for k in FIELDS}; evidence=[]; lengths=[]; names=[]
    first=None
    for path, select, label in sources:
        qa=json.loads(path.with_suffix('.qa.json').read_text())
        if not qa['accepted_for_training'] or not qa['visual_review_passed'] or qa['motion_bank_sha256']!=sha(path):
            raise ValueError('Unqualified or changed source')
        if qa.get('split')=='validation': raise ValueError('Held-out source cannot enter training')
        with np.load(path,allow_pickle=False) as data: z={k:data[k] for k in data.files}
        if first is None:first=z
        for k in ('fps','joint_names','body_names','asset_revision'):
            if not np.array_equal(z[k],first[k]):raise ValueError('Source identity mismatch: '+k)
        start,n=0,len(z['joint_pos'])
        if select:
            i=z['motion_names'].tolist().index(select); start=int(z['motion_starts'][i]);n=int(z['motion_lengths'][i])
            if not next(r for r in qa['references'] if r['name']==select)['accepted']:raise ValueError('Rejected clip')
        for k in FIELDS:chunks[k].append(z[k][start:start+n].copy())
        evidence.append(dict(path=str(path),sha256=sha(path),qa=qa,selected_clip=select,start=start,length=n))
        lengths.append(n);names.append(label)
    payload={k:np.concatenate(v) for k,v in chunks.items()}
    payload.update({k:first[k] for k in ('fps','joint_names','body_names','asset_revision')})
    payload.update(motion_names=np.array(names),motion_starts=np.r_[0,np.cumsum(lengths)[:-1]],motion_lengths=np.array(lengths),
                   retarget_method=np.array('reviewed_immutable_walk_stand_subset_v1'))
    bank=dest/'walk_stand.npz';np.savez_compressed(bank,**payload)
    with np.load(bank) as data:
        for k in FIELDS:
            offset=0
            for original in chunks[k]:
                if not np.array_equal(data[k][offset:offset+len(original)],original):raise ValueError('Changed source samples')
                offset+=len(original)
    spec=mujoco.MjSpec.from_file(str(ROBOT_XML));spec.worldbody.add_geom(name='review_floor',type=mujoco.mjtGeom.mjGEOM_PLANE,size=[0,0,.05],rgba=[.5,.53,.55,1])
    model=spec.compile();state=mujoco.MjData(model)
    jnames=payload['joint_names'].tolist(); bids=payload['body_names'].tolist(); root=bids.index('pelvis_link')
    ids=[int(model.joint(n).qposadr[0]) for n in jnames];feet=[model.geom(s+'_foot_collision').id for s in ['left','right']]
    checks=[]
    for index,(start,n) in enumerate(zip(payload['motion_starts'],lengths)):
        minimum=1.; knee=payload['joint_pos'][start:start+n][:,[jnames.index(s+'_knee_joint') for s in ['left','right']]]
        for t in range(start,start+n):
            state.qpos[:3]=payload['body_pos_w'][t,root];state.qpos[3:7]=payload['body_quat_w'][t,root];state.qpos[ids]=payload['joint_pos'][t]
            mujoco.mj_forward(model,state)
            minimum=min(minimum,*[float(state.geom_xpos[g,2]-abs(state.geom_xmat[g].reshape(3,3)[2])@model.geom_size[g]) for g in feet])
        if minimum<-.002:raise ValueError('Source subset penetrates ground')
        checks.append(dict(name=names[index],frames=n,seconds=(n-1)/50,min_sole_height_m=minimum,median_knees_deg=np.rad2deg(np.median(knee,axis=0)).tolist()))
    sheet=Image.new('RGB',(1440,720));draw=ImageDraw.Draw(sheet)
    with mujoco.Renderer(model,width=360,height=320) as renderer:
        for row,(start,n) in enumerate(zip(payload['motion_starts'],lengths)):
            for col,fraction in enumerate([.1,.35,.6,.85]):
                t=int(start+fraction*(n-1));state.qpos[:3]=payload['body_pos_w'][t,root];state.qpos[3:7]=payload['body_quat_w'][t,root];state.qpos[ids]=payload['joint_pos'][t];mujoco.mj_forward(model,state)
                camera=mujoco.MjvCamera();camera.lookat[:]=state.qpos[:3];camera.lookat[2]=.5;camera.distance=1.6;camera.elevation=-15;camera.azimuth=90 if col%2 else 0
                renderer.update_scene(state,camera=camera);sheet.paste(Image.fromarray(renderer.render()),(col*360,row*360+40))
                draw.text((col*360+5,row*360+5),f'{names[row]} / {fraction*(n-1)/50:.2f}s',fill='white')
    sheet.save(dest/'reference-review.png')
    report=dict(asset_revision=ASSET_REVISION,motion_bank_sha256=sha(bank),accepted_for_training=True,visual_review_passed=False,
        split='candidate_training',source_samples_exactly_preserved=True,source_evidence=evidence,checks=checks,
        qualification='Subset of hash-verified numeric/visually reviewed banks; fresh composition visual review pending.',
        excludes=['Neutral walking','Neutral lateral','held-out clips','LAFAN turns','dance'])
    bank.with_suffix('.qa.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(checks,indent=2))

if __name__=='__main__':main()
