"""Check an unbroken strand, its needle-eye endpoint and feed continuity."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
ROOT=Path(__file__).resolve().parents[2];DATA=ROOT/'output/soft_suture_continuous'
m=json.loads((DATA/'manifest.json').read_text())
f=np.fromfile(DATA/'frames.bin',dtype='<f4').reshape(-1,m['state_width'])
offset=34+m['tissue_nodes']*3
tail=np.array([0,.005708948958544,.01124811054846])
previous=None;maximum=0.;error=0.;minimum=100
for i,row in enumerate(f):
    count=int(row[offset]);minimum=min(minimum,count);assert 2<=count<=22
    line=row[offset+1:offset+1+count*3].reshape(-1,3).astype(float)
    T=row[22:34].reshape(3,4).astype(float);expected=T[:,:3]@tail+T[:,3]
    error=max(error,float(np.linalg.norm(line[-1]-expected)));assert error<1e-7
    assert np.linalg.norm(line[0]-[-.015,.345,.5935])<1e-7
    assert np.isfinite(line).all()
    dense=[]
    for a,b in zip(line[:-1],line[1:]):
        dense.extend(np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.0002))+1)))
    dense=np.array(dense)
    if previous is not None:
        jump=max(cKDTree(previous).query(dense)[0].max(),cKDTree(dense).query(previous)[0].max())
        maximum=max(maximum,float(jump))
        assert jump<.004,('Thread discontinuity',i+1,jump)
    previous=dense
report=dict(result='PASS',frames=len(f),minimum_thread_vertices=minimum,
    maximum_needle_eye_error_mm=error*1000,maximum_thread_step_mm=maximum*1000,
    initial_strand_present=True,all_frames_have_one_connected_strand=True)
metrics=json.loads((DATA/'metrics.json').read_text())
for stitch in range(1,6):
    observed={r['visible_thread_vertices'] for r in metrics if r['stitch']==stitch and r['phase']=='left_extract'}
    base=2+4*(stitch-1)
    assert {base+1,base+2,base+3,base+4}<=observed,('Missing gradual feed stages',stitch,observed)
report['all_five_stitches_feed_progressively']=True
(DATA/'thread_validation.json').write_text(json.dumps(report,indent=2));print(report)
