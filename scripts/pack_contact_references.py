"""Pack finalized contact references; full banks run independent QA/rendering."""
import argparse,json,sys,subprocess,os
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from retarget.convert_umr import pack

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--partial',action='store_true');parser.add_argument('--folder',type=Path,default=ROOT/'data/motions/contact_v5_v11');args=parser.parse_args()
    folder=args.folder
    names=[row['name'] for row in json.loads((ROOT/'data/sources/lafan1-resolved/selected_segments.json').read_text())]
    paths=[folder/(name+'.npz') for name in names]
    if args.partial:paths=[p for p in paths if p.exists()]
    bank=folder/('partial-review.npz' if args.partial else 'v5_locomotion.npz');pack(paths,bank)
    if not args.partial:subprocess.run([sys.executable,'scripts/validate_reference_bank.py','--motion',str(bank)],cwd=ROOT,check=True)
    subprocess.run([sys.executable,'scripts/render_reference_preview.py','--motion',str(bank)],cwd=ROOT,env={**os.environ,'MUJOCO_GL':'egl'},check=True)
