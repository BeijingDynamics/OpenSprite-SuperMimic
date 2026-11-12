"""Rank nonoverlapping Geno locomotion crops for review, never auto-approve them."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys

import numpy as np
from scipy.signal import find_peaks, savgol_filter

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from retarget.bvh_tools.bvh_adapters import read_transforms


def wrap(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))


def features(path):
    hierarchy,names,parents,offsets,channels,positions,rotations=read_transforms(path)
    if names[0]!='Hips' or 'Spine3' not in names or 'LeftToeBaseEnd' not in names:
        raise ValueError('This screening convention requires OrangeDuck Geno BVH')
    text=path.read_text()
    lines=text.split('MOTION',1)[1].strip().splitlines()
    dt=float(lines[1].split(':',1)[1])
    fps=round(1/dt)
    if abs(fps*dt-1)>1e-4:raise ValueError('Unexpected source timebase')
    p=positions/100.
    root=p[:,0][:,[2,0,1]]
    heading=np.unwrap(np.arctan2(rotations[:,0,0,2],rotations[:,0,2,2]))
    window=min(2*(fps//2)+1,len(root)//2*2-1)
    velocity=np.gradient(savgol_filter(root,window,2,axis=0),dt,axis=0)
    # Separate trajectory turning from normal stride-frequency pelvis yaw.
    turn_window=min(2*fps+1,len(root)//2*2-1)
    heading_smooth=savgol_filter(heading,turn_window,2)
    rate=np.gradient(heading_smooth,dt)
    root_tilt=np.arccos(np.clip(rotations[:,0,1,1],-1,1))
    speed=np.linalg.norm(velocity[:,:2],axis=-1)
    motion_heading=np.arctan2(velocity[:,1],velocity[:,0])
    feet=[];foot_yaw=[];drops=[]
    for side in ('Left','Right'):
        ankle=p[:,names.index(side+'Foot')]
        toe=p[:,names.index(side+'ToeBase')]
        direction=toe-ankle
        foot_yaw.append(wrap(np.arctan2(direction[:,0],direction[:,2])-heading))
        feet.append(np.minimum(ankle[:,1],toe[:,1]))
        drops.append(p[:,names.index(side+'Shoulder'),1]-p[:,names.index(side+'Hand'),1])
    foot_z=np.stack(feet,-1)
    floor=float(np.quantile(foot_z,.03))
    foot_z-=floor
    return dict(dt=dt,fps=fps,root=root,heading=heading_smooth,rate=rate,speed=speed,
                travel_error=wrap(motion_heading-heading),foot_yaw=np.stack(foot_yaw,-1),
                foot_height=foot_z,hand_drop=np.minimum(*drops),root_tilt=root_tilt,
                hierarchy=hierarchy,frames=lines[2:])


def candidates(path,data):
    fps=data['fps'];n=len(data['frames']);rows=[];rejected=Counter()
    performer=re.search(r'subject(\d+)',path.stem)
    if performer is None:raise ValueError('Missing performer identity')
    performer=int(performer.group(1))
    for seconds in (3,6,12,20):
        size=seconds*fps
        for begin in range(0,n-size+1,fps//2):
            end=begin+size;sl=slice(begin,end)
            speed=data['speed'][sl];rate=data['rate'][sl]
            yaw=float(data['heading'][end-1]-data['heading'][begin])
            if np.ptp(data['root'][sl,2])>.18 or speed.max()>2.2:
                rejected['height_or_speed']+=1;continue
            if data['hand_drop'][sl].min()<.12:
                rejected['calibration_or_raised_arms']+=1;continue
            if np.quantile(data['root_tilt'][sl],.9)>np.deg2rad(12) or data['root_tilt'][sl].max()>np.deg2rad(20):
                rejected['non_neutral_root_tilt']+=1;continue
            first=float(speed[:fps].mean());last=float(speed[-fps:].mean())
            active=speed>.2
            travel=float(np.quantile(np.abs(data['travel_error'][sl][active]),.9)) if active.any() else 0.
            foot=float(np.quantile(np.abs(data['foot_yaw'][sl]),.9))
            foot_mean=np.rad2deg(data['foot_yaw'][sl].mean(0)).tolist()
            categories=[]
            if seconds==3 and speed.max()<.10 and np.abs(rate).max()<.12:
                categories.append('stand')
            if seconds>=6:
                if .25<speed.mean()<1.8 and np.mean(active)>.9 and abs(yaw)<.15 and np.abs(rate).mean()<.12 and travel<.25:
                    categories.append('straight')
                if first<.12 and last>.4 and abs(yaw)<.15 and travel<.3:categories.append('start')
                if last<.12 and first>.4 and abs(yaw)<.15 and travel<.3:categories.append('stop')
                if .3<abs(yaw)<1.4 and speed.mean()>.2 and (rate*np.sign(yaw)>-.05).mean()>.9 and travel<.4:
                    categories.append('left' if yaw>0 else 'right')
                # Longer natural paths need not be a single straight/turn class.
                # Preserve their original heading, stops and transitions.
                if seconds>=12 and .2<speed.mean()<1.8 and travel<.4 and np.quantile(abs(rate),.95)<.8:
                    categories.append('locomotion')
            if not categories:rejected['no_clean_task_segment']+=1;continue
            if foot>np.deg2rad(30):rejected['foot_heading_p90_over_30deg']+=1;continue
            peaks=[]
            for side in range(2):
                z=data['foot_height'][sl,side]
                ids,_=find_peaks(z,distance=round(.25*fps),prominence=.015)
                peaks.append(z[ids].tolist())
            if any(c in categories for c in ('straight','locomotion')) and any(len(x)<3 for x in peaks):
                rejected['insufficient_alternating_swing_candidates']+=1;continue
            for category in categories:
                rows.append(dict(source=path.name,performer=performer,
                    split='validation' if performer==5 else 'train',category=category,
                    begin=begin,end=end,fps=1/data['dt'],duration_s=seconds,
                    mean_human_speed_m_s=float(speed.mean()),start_speed_m_s=first,end_speed_m_s=last,
                    net_yaw_rad=yaw,travel_heading_p90_deg=float(np.rad2deg(travel)),
                    foot_heading_p90_deg=float(np.rad2deg(foot)),mean_foot_heading_deg=foot_mean,
                    root_tilt_p90_deg=float(np.rad2deg(np.quantile(data['root_tilt'][sl],.9))),
                    swing_peak_heights_m=peaks,
                    score=float(foot+travel+(.5*abs(yaw) if category in ('straight','start','stop','stand') else 0.))))
    return rows,rejected


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--sources',type=Path,nargs='+',required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--per-category',type=int,default=12)
    parser.add_argument('--per-performer-category',type=int,default=6)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    rows=[];sources=[]
    for path in args.sources:
        data=features(path)
        proposed,rejected=candidates(path,data)
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        for row in proposed:row['source_sha256']=digest
        rows.extend(proposed)
        sources.append(dict(path=str(path),sha256=digest,frames=len(data['frames']),fps=1/data['dt'],
                            candidates=len(proposed),rejections=dict(rejected)))
        print(json.dumps(sources[-1]),flush=True)
    chosen=[];counts=Counter();performer_counts=Counter()
    for row in sorted(rows,key=lambda r:(-r['duration_s'],r['score'])):
        key=(row['split'],row['category']);person=key+(row['performer'],)
        if counts[key]>=args.per_category or performer_counts[person]>=args.per_performer_category:continue
        margin=round(.5*row['fps'])
        if any(other['source']==row['source'] and row['begin']<other['end']+margin
               and other['begin']<row['end']+margin for other in chosen):continue
        chosen.append(row);counts[key]+=1;performer_counts[person]+=1
    # Crops preserve every original BVH sample and are not yet accepted data.
    for path in args.sources:
        selected=[row for row in chosen if row['source']==path.name]
        if not selected:continue
        hierarchy,motion=path.read_text().split('MOTION',1)
        lines=motion.strip().splitlines();dt=float(lines[1].split(':',1)[1]);frames=lines[2:]
        for row in selected:
            name=f"{path.stem}_{row['begin']:06d}_{row['end']:06d}_{row['category']}.bvh"
            target=args.output/name
            target.write_text(hierarchy+f"MOTION\nFrames: {row['end']-row['begin']}\nFrame Time: {dt:.17g}\n"+
                              '\n'.join(frames[row['begin']:row['end']])+'\n')
            row.update(crop=name,crop_sha256=hashlib.sha256(target.read_bytes()).hexdigest())
    report=dict(accepted_for_training=False,visual_review_passed=False,
                scope='Human-source candidate screening only; retargeted robot QA still required',
                validation_split='Subject 5 held out before cropping; no selected overlap within a source',
                sources=sources,selected=chosen,
                proposed_counts=dict(Counter(row['category'] for row in rows)),
                selected_counts={':'.join(key):n for key,n in counts.items()},
                total_selected_seconds=sum(row['duration_s'] for row in chosen))
    (args.output/'candidates.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({key:report[key] for key in ('selected_counts','total_selected_seconds','accepted_for_training')}),flush=True)


if __name__=='__main__':main()
