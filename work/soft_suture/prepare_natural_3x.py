import json
from pathlib import Path
import numpy as np
from mechanics import SoftSuture,Parameters
ROOT=Path(__file__).resolve().parents[2];source=ROOT/'output/soft_suture_natural';out=ROOT/'output/soft_suture_natural_3x';out.mkdir(exist_ok=True)
m=json.loads((source/'manifest.json').read_text());f=np.fromfile(source/'frames.bin',dtype='<f4').reshape(-1,m['state_width'])
s=SoftSuture(Parameters(**m['parameters']));minimum=1.;eye_error=0.;offset=34+s.n_tissue*3
for row in f:
    pos=row[34:offset].reshape(-1,3);z=pos[s.tets];J=np.einsum('ij,ij->i',z[:,1]-z[:,0],np.cross(z[:,2]-z[:,0],z[:,3]-z[:,0]))/6/s.volume0
    minimum=min(minimum,float(J.min()));assert (J>0).all() and np.isfinite(row).all()
    assert np.max(abs(pos[s.fixed]-s.rest[s.fixed]))<1e-7
    count=int(row[offset]);assert 2<=count<=512
    cable=row[offset+1:offset+1+count*3].reshape(-1,3);T=row[22:34].reshape(3,4)
    eye_error=max(eye_error,float(np.linalg.norm(cable[-1]-(T[:,:3]@np.array([0,.005708948958544,.01124811054846])+T[:,3]))))
assert eye_error<1e-7
assert m['completed_stitches']==5 and len(m['puncture_events'])==5
assert max(m['final_gap_mm'])<1.2 and min(m['final_gap_mm'])>=.299
# Matched indentation/release experiment at two integration timesteps.
recovery=[]
for dt in [.025,.0125]:
    p=Parameters(**{**m['parameters'],'dt':dt,'puncture_force':100.});model=SoftSuture(p)
    for _ in range(round(.5/dt)):model.step()
    baseline=model.x.copy();peak=0.
    for t in np.arange(dt,1+dt/2,dt):
        model.step(tip=[-.022,.345,.592-.004*t],puncture_key=1)
        peak=max(peak,float(np.max(np.linalg.norm(model.x-baseline,axis=1))*1000))
    for _ in range(round(2/dt)):model.step()
    residual=float(np.max(np.linalg.norm(model.x-baseline,axis=1))*1000)
    assert peak>.3 and residual<.15
    recovery.append(dict(dt=dt,peak_mm=peak,residual_mm=residual))
assert abs(recovery[0]['peak_mm']-recovery[1]['peak_mm'])<.15
report=dict(result='PASS',minimum_volume_ratio=minimum,maximum_needle_eye_error_mm=eye_error*1000,final_gap_mm=m['final_gap_mm'],recovery=recovery)
(source/'natural_validation.json').write_text(json.dumps(report,indent=2))
indices=np.unique(np.r_[np.arange(0,len(f),3),len(f)-1]);f[indices].tofile(out/'frames.bin')
m.update(source_frame_count=len(f),frame_count=len(indices),playback_speed=3,source_indices=indices.tolist(),physics_dt=m['parameters']['dt'])
(out/'manifest.json').write_text(json.dumps(m,indent=2));(out/'natural_validation.json').write_text(json.dumps(report,indent=2))
print(report);print('Playback frames',len(indices),flush=True)
