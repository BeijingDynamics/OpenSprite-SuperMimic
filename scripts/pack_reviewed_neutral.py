"""Pack explicitly reviewed Neutral candidates with split and provenance checks."""
import argparse,hashlib,json,shutil,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))


def validate_source_intervals(selected,reserved,split):
    """Prevent overlapping alternatives and leakage across reserved source time."""
    if split not in ('candidate_training','validation'):raise ValueError('Unknown split')
    for i,row in enumerate(selected):
        if row['split']!=split:raise ValueError('Candidate belongs to a different split')
        if not row.get('source_sha256') or not 0<=row['begin']<row['end']:
            raise ValueError('Missing source identity or invalid source interval')
        for other in selected[:i]:
            if row['source_sha256']==other['source_sha256'] and row['begin']<other['end'] and other['begin']<row['end']:
                raise ValueError('Overlapping source alternatives cannot coexist in this bank')
        for other in reserved:
            if other['split']==split or row['source_sha256']!=other['source_sha256']:continue
            gap=round(max(row['fps'],other['fps']))
            if row['begin']<other['end']+gap and other['begin']<row['end']+gap:
                raise ValueError('Candidate overlaps the opposite split or its one-second margin')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--selection',type=Path,required=True)
    parser.add_argument('--reserved-manifest',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--split',choices=['candidate_training','validation'],required=True)
    args=parser.parse_args();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    selection=json.loads(args.selection.read_text());reserved=json.loads(args.reserved_manifest.read_text())
    if not selection:raise ValueError('No selected references')
    rows=[];clips=[];evidence=[]
    for selected in selection:
        folder=Path(selected['directory']);name=selected['name'];clip=folder/(name+'.npz')
        if (folder/'admission-withheld.json').exists():raise ValueError('Candidate admission is explicitly withheld')
        report=json.loads(clip.with_suffix('.json').read_text());review=json.loads((folder/'visual-review.json').read_text())
        summary=json.loads((folder/'review-summary.json').read_text())
        if not report['numeric_candidate_passed'] or not report['source_frame_audit']['passed']:
            raise ValueError('Candidate failed numerical/source-frame QA')
        quality=report['source_upper_body_quality']
        if not quality['passed'] or 'arm_orientation_p95_deg' not in quality or not report.get('elbow_branch_passed'):
            raise ValueError('Candidate lacks complete arm orientation and elbow-branch QA')
        if not review['passed'] or review['candidate_sha256']!=sha(clip):raise ValueError('Missing or stale visual review')
        if review['review_bank_sha256']!=sha(folder/'review-bank.npz'):raise ValueError('Reviewed bank changed')
        if any(value!=0 for value in summary['results'].values()):raise ValueError('Independent candidate validation failed')
        for name_,digest in review['evidence_sha256'].items():
            if sha(folder/name_)!=digest:raise ValueError('Visual evidence changed: '+name_)
        source_rows=json.loads((folder/'review-manifest.json').read_text())
        if len(source_rows)!=1 or source_rows[0]['name']!=name:raise ValueError('Candidate manifest mismatch')
        source=source_rows[0]
        if source['crop_sha256']!=report['source_sha256']:raise ValueError('Source crop identity mismatch')
        rows.append(source);clips.append(clip)
        evidence.append(dict(name=name,candidate_sha256=sha(clip),report_sha256=sha(clip.with_suffix('.json')),
                             visual_review_sha256=sha(folder/'visual-review.json'),refiner_sha256=report['refiner_sha256']))
    validate_source_intervals(rows,reserved,args.split)
    args.output_dir.mkdir(parents=True,exist_ok=False)
    for clip in clips:
        shutil.copy2(clip,args.output_dir/clip.name);shutil.copy2(clip.with_suffix('.json'),args.output_dir/clip.with_suffix('.json').name)
    manifest=args.output_dir/'source-manifest.json';manifest.write_text(json.dumps(rows,indent=2)+'\n')
    from retarget.convert_umr import pack
    from validate_reference_bank import validate
    bank=args.output_dir/'neutral_locomotion.npz';pack(clips,bank);validate(bank,manifest)
    qa_path=bank.with_suffix('.qa.json');qa=json.loads(qa_path.read_text())
    qa.update(visual_review_passed=True,qualification='Reviewed kinematic research library; no physical or hardware qualification',
              split=args.split,reviewed_candidates=evidence,reserved_manifest_sha256=sha(args.reserved_manifest),
              dataset_license='100STYLE motion: CC-BY-4.0; Geno FBX mesh is excluded and retains its separate license')
    qa_path.write_text(json.dumps(qa,indent=2)+'\n')
    print(json.dumps(dict(bank=str(bank),sha256=sha(bank),clips=len(clips),split=args.split)))


if __name__=='__main__':main()
