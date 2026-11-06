"""Select real contiguous human walk/turn/stand segments; never synthesize a root path."""
import json,sys,hashlib
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import savgol_filter
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from retarget.bvh_tools.bvh_adapters import _hierarchy

def load(path):
    hierarchy,motion=path.read_text().split('MOTION',1)
    names,parents,offsets,channels=_hierarchy(hierarchy)
    lines=motion.strip().splitlines();dt=float(lines[1].split(':')[1]);frames=lines[2:]
    # Geno root has translation followed by its declared Euler rotation order.
    all_data=np.loadtxt(frames,ndmin=2)
    data=all_data[:,:len(channels[0])]
    position=np.zeros((len(data),3));angles=[];order=''
    for col,name in enumerate(channels[0]):
        if name.endswith('position'):position[:,'XYZ'.index(name[0])]=data[:,col]/100
        else:angles.append(data[:,col]);order+=name[0]
    rot=Rotation.from_euler(order,np.array(angles).T,degrees=True).as_matrix()
    forward=rot[:,:,2]
    yaw=np.unwrap(np.arctan2(forward[:,0],forward[:,2]))
    # World robot convention is x=human z, y=human x, z=human y.
    pos=position[:,[2,0,1]]
    velocity=np.gradient(savgol_filter(pos,61,2,axis=0),dt,axis=0)
    speed=np.linalg.norm(velocity[:,:2],axis=1)
    yaw=savgol_filter(yaw,121,2);rate=np.gradient(yaw,dt)
    # Exclude calibration T-poses by measuring actual hand height below shoulders.
    wanted=[names.index(side+part) for side in ['Left','Right'] for part in ['Shoulder','Hand']]
    ancestors=set(wanted)
    for j in wanted:
        while parents[j]>=0:j=parents[j];ancestors.add(j)
    world_p={};world_R={};cursor=0
    for j,ch in enumerate(channels):
        if j in ancestors:
            local=np.tile(np.array(offsets[j])/100,(len(data),1));angles=[];order=''
            for col,name in enumerate(ch):
                if name.endswith('position'):local[:,'XYZ'.index(name[0])]=all_data[:,cursor+col]/100
                else:angles.append(all_data[:,cursor+col]);order+=name[0]
            local_R=Rotation.from_euler(order,np.array(angles).T,degrees=True).as_matrix()
            par=parents[j]
            world_p[j]=local if par<0 else world_p[par]+np.einsum('tij,tj->ti',world_R[par],local)
            world_R[j]=local_R if par<0 else world_R[par]@local_R
        cursor+=len(ch)
    drop=np.minimum(world_p[wanted[0]][:,1]-world_p[wanted[1]][:,1],world_p[wanted[2]][:,1]-world_p[wanted[3]][:,1])
    return dict(path=path,hierarchy=hierarchy,frames=frames,dt=dt,pos=pos,speed=speed,yaw=yaw,rate=rate,hand_drop=drop)

def select():
    folder=ROOT/'data/sources/lafan1-resolved'
    sources=[load(folder/name) for name in ['walk1_subject1.bvh','walk2_subject1.bvh'] if (folder/name).exists()]
    if not sources:raise RuntimeError('No genuine source BVH downloaded')
    candidates={k:[] for k in ['straight','left','right','start','stop','stand']}
    for src in sources:
        fps=round(1/src['dt']);length=8*fps
        for begin in range(0,len(src['frames'])-length,fps//2):
            end=begin+length;s=slice(begin,end)
            speed=src['speed'][s];yaw=src['yaw'][end-1]-src['yaw'][begin]
            rate=src['rate'][s];height=src['pos'][s,2]
            if np.ptp(height)>.20 or speed.max()>2.5 or src['hand_drop'][s].min()<.12:continue
            mean=float(speed.mean());first=float(speed[:fps].mean());last=float(speed[-fps:].mean())
            values=dict(begin=begin,end=end,mean_speed=mean,net_yaw=float(yaw),source=src)
            if .45<mean<1.8 and abs(yaw)<.18 and np.mean(abs(rate))<.12:
                candidates['straight'].append((mean,values))
            if .25<mean<1.8 and .55<yaw<2.8:
                candidates['left'].append((yaw,values))
            if .25<mean<1.8 and -2.8<yaw<-.55:
                candidates['right'].append((-yaw,values))
            if first<.12 and last>.4 and abs(yaw)<1.5:
                candidates['start'].append((last-first,values))
            if last<.12 and first>.4 and abs(yaw)<1.5:
                candidates['stop'].append((first-last,values))
        # A genuine quiet 1.5s stand, not the source's calibration T-pose.
        length=round(1.5*fps)
        for begin in range(0,len(src['frames'])-length,fps//2):
            end=begin+length;s=slice(begin,end)
            if src['speed'][s].max()<.18 and src['speed'][s].mean()<.08 and np.abs(src['rate'][s]).mean()<.15 and src['hand_drop'][s].min()>.12:
                values=dict(begin=begin,end=end,mean_speed=float(src['speed'][s].mean()),net_yaw=float(src['yaw'][end-1]-src['yaw'][begin]),source=src)
                candidates['stand'].append((-values['mean_speed'],values))
        # Six-second continuous straight walking, excluding all stationary calibration.
        length=6*fps
        for begin in range(0,len(src['frames'])-length,fps//2):
            end=begin+length;s=slice(begin,end);speed=src['speed'][s];yaw=src['yaw'][end-1]-src['yaw'][begin]
            if .4<speed.mean()<1.5 and (speed>.2).mean()>.9 and abs(yaw)<.2 and np.mean(abs(src['rate'][s]))<.18 and src['hand_drop'][s].min()>.12 and np.ptp(src['pos'][s,2])<.20:
                values=dict(begin=begin,end=end,mean_speed=float(speed.mean()),net_yaw=float(yaw),source=src)
                candidates['straight'].append((values['mean_speed'],values))
    counts={k:len(v) for k,v in candidates.items()};print(json.dumps({'candidates':counts}),flush=True)
    required=('straight','left','right','start','stop')
    missing=[k for k in required if not candidates[k]]
    if missing:raise RuntimeError(f'No real source segment for {missing}; acquire more sources, never synthesize replacements')
    manifest=[]
    for kind,entries in candidates.items():
        if not entries:continue
        # Prefer moderate actual turns, avoiding a sharp 180 degree pivot.
        if kind in ('left','right'):entries.sort(key=lambda e:abs(e[0]-1.0))
        elif kind=='straight':entries.sort(key=lambda e:abs(e[0]-.8))
        else:entries.sort(key=lambda e:-e[0])
        row=entries[0][1];src=row['source'];name='v5_'+kind
        path=folder/(name+'.bvh');frames=src['frames'][row['begin']:row['end']]
        path.write_text(src['hierarchy']+f'MOTION\nFrames: {len(frames)}\nFrame Time: {src["dt"]:.17g}\n'+'\n'.join(frames)+'\n')
        manifest.append(dict(name=name,category=kind,original_source=src['path'].name,
                             original_sha256=hashlib.sha256(src['path'].read_bytes()).hexdigest(),
                             start_frame=row['begin'],end_frame_exclusive=row['end'],fps=1/src['dt'],
                             duration_seconds=len(frames)*src['dt'],mean_human_speed_m_s=row['mean_speed'],
                             human_net_yaw_rad=row['net_yaw'],sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    (folder/'selected_segments.json').write_text(json.dumps(manifest,indent=2));print(json.dumps(manifest,indent=2),flush=True)
if __name__=='__main__':select()
