"""Seeded small root-tilt probes for paired simulation evaluation only."""
import numpy as np

def sample_tilt(count,bound_deg,seed):
    if not np.isfinite(bound_deg) or not 0<=bound_deg<=5:
        raise ValueError('Initial tilt bound must be finite and within 0..5 degrees')
    angles=np.deg2rad(np.random.default_rng(seed).uniform(-bound_deg,bound_deg,(count,2)))
    roll,pitch=angles.T
    cr,sr,cp,sp=np.cos(roll),np.sin(roll),np.cos(pitch),np.sin(pitch)
    matrices=np.zeros((count,3,3))
    matrices[:,0,0]=cp;matrices[:,0,1]=sp*sr;matrices[:,0,2]=sp*cr
    matrices[:,1,1]=cr;matrices[:,1,2]=-sr
    matrices[:,2,0]=-sp;matrices[:,2,1]=cp*sr;matrices[:,2,2]=cp*cr
    return matrices,angles
