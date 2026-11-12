"""Convert and contact-refine named source candidates without approving a bank."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import zipfile

ROOT=Path(__file__).resolve().parents[1]


def process(row, args):
    name=row['name']
    if Path(name).name!=name or name in ('.','..'):
        raise ValueError('Expected a plain candidate name')
    native=args.native_dir/name/'native.npz'
    started=time.monotonic()
    # Native retargeting runs separately; never consume a partial ZIP write.
    while True:
        if native.exists():
            import numpy as np
            try:
                with np.load(native,allow_pickle=True) as data:
                    complete=len(data['qpos'])==row['end']-row['begin']
                if complete:break
            except (OSError,ValueError,EOFError,zipfile.BadZipFile):pass
        if time.monotonic()-started>1800:
            return dict(name=name,status='native_timeout')
        time.sleep(2)
    log=args.output_dir/(name+'.log')
    environment={**os.environ,'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','MKL_NUM_THREADS':'1','PYTHONUNBUFFERED':'1'}
    converted=args.output_dir/'converted'/(name+'.npz')
    code='from pathlib import Path; from retarget.convert_umr import convert; import sys; convert(Path(sys.argv[1]),sys.argv[2],Path(sys.argv[3]))'
    commands=[
        [sys.executable,'-c',code,str(native),name,str(converted)],
        [sys.executable,'scripts/refine_reference_contacts.py','--name',name,
         '--input-dir',str(converted.parent),'--native-dir',str(args.native_dir),
         '--output-dir',str(args.output_dir/'refined'),'--stop-when-feasible','--source-torso'],
    ]
    with log.open('w') as stream:
        for index,command in enumerate(commands):
            result=subprocess.run(command,cwd=ROOT,env=environment,stdout=stream,stderr=subprocess.STDOUT)
            if result.returncode:
                return dict(name=name,status='conversion_failed' if index==0 else 'refinement_failed',
                            returncode=result.returncode,log=str(log))
    return dict(name=name,status='refined_candidate',accepted_for_training=False,log=str(log))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--manifest',type=Path,required=True)
    parser.add_argument('--native-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=2)
    args=parser.parse_args()
    if not 1<=args.workers<=4:parser.error('Use one to four isolated clip workers')
    args.output_dir.mkdir(parents=True,exist_ok=False)
    rows=json.loads(args.manifest.read_text())
    results=[]
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        for row in pool.map(lambda item:process(item,args),rows):
            results.append(row)
            (args.output_dir/'status.json').write_text(json.dumps(results,indent=2)+'\n')
            print(json.dumps(row),flush=True)


if __name__=='__main__':main()
