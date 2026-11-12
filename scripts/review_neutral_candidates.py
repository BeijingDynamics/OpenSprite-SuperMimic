"""Render and independently audit numeric candidates without granting visual approval."""
import argparse,hashlib,json,os,subprocess,sys
from pathlib import Path
import numpy as np
import imageio.v2 as iio
from PIL import Image

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from retarget.convert_umr import pack
from audit_sole_tracking import geometry,measure
from opensprite_tracker.robot import BODY_JOINT_NAMES


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--batch-dir',type=Path,required=True);args=parser.parse_args()
    base=args.batch_dir;manifest=json.loads((base/'manifest.json').read_text())
    status=json.loads((base/'status.json').read_text())
    env={**os.environ,'MUJOCO_GL':'egl','OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1'}
    for row in status:
        if not row.get('numeric_candidate_passed'):continue
        parent=row['name'];name=row['final_name'];folder=base/parent/'refined'
        if (folder/'review-summary.json').exists():continue
        clip=folder/(name+'.npz');report=json.loads(clip.with_suffix('.json').read_text())
        assert report['numeric_candidate_passed'] and report['source_upper_body_quality']['passed']
        assert report['source_frame_audit']['passed']
        batch_row=next(r for r in manifest['rows'] if r['name']==parent)
        native_manifest=Path(batch_row['native_dir']).parent/'source_manifest.json'
        source=next(dict(s) for s in json.loads(native_manifest.read_text()) if s['name']==parent)
        assert source['crop_sha256']==report['source_sha256']
        source.update(parent_name=parent,name=name,source_category=source['category'],
                      category='stand' if '_ID_' in parent else 'locomotion',require_source_torso=True,
                      split=row['split'],source_speed=row['speed'])
        review_manifest=folder/'review-manifest.json';review_manifest.write_text(json.dumps([source],indent=2)+'\n')
        bank=folder/'review-bank.npz';pack([clip],bank)
        commands=[('validation',[sys.executable,'scripts/validate_reference_bank.py','--motion',str(bank),'--manifest',str(review_manifest)]),
                  ('render',[sys.executable,'scripts/render_reference_views.py','--motion',str(clip),'--output',str(folder/'reference-review.mp4'),'--follow-heading'])]
        results={}
        for label,command in commands:
            with (folder/(label+'.log')).open('w') as log:
                result=subprocess.run(command,cwd=ROOT,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=600)
            results[label]=result.returncode
        with np.load(clip,allow_pickle=False) as z:
            names=z['joint_names'].tolist();pose=np.c_[z['qpos'][:,:7],z['joint_pos'][:,[names.index(n) for n in BODY_JOINT_NAMES]]]
        sole=measure(geometry(pose),50);(folder/'independent-sole-audit.json').write_text(json.dumps(sole,indent=2)+'\n')
        if results['render']==0:
            with iio.get_reader(folder/'reference-review.mp4') as video:
                fps=video.get_meta_data()['fps'];duration=(len(pose)-1)/50
                sheet=Image.new('RGB',(1536,1040),'white')
                for i,t in enumerate(np.linspace(.08,max(.08,duration-.08),12)):
                    sheet.paste(Image.fromarray(video.get_data(round(t*fps))).resize((512,260)),((i%3)*512,(i//3)*260))
                sheet.save(folder/'review-sheet.jpg')
        summary=dict(name=name,split=row['split'],results=results,visual_review_passed=False,
                     candidate_sha256=hashlib.sha256(clip.read_bytes()).hexdigest(),
                     review_bank_sha256=hashlib.sha256(bank.read_bytes()).hexdigest(),
                     sole_height_max_m=sole['sole_height_max_m'],clearance_event_counts=sole['clearance_event_counts'])
        (folder/'review-summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
