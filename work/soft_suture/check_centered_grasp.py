"""Read back actual poses and render the corrected bite between both jaws."""
import json,time
from pathlib import Path
import numpy as np
from PIL import Image
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT=Path(__file__).resolve().parents[2];DATA=ROOT/'output/soft_suture_continuous'
m=json.loads((DATA/'manifest.json').read_text());f=np.fromfile(DATA/'frames.bin',dtype='<f4').reshape(-1,m['state_width'])
rows=json.loads((DATA/'metrics.json').read_text());events=[i for i,r in enumerate(rows) if r['phase']=='handoff']
s=RemoteAPIClient(port=23002).require('sim');driver=s.getObject('/SoftSuturePlayback')
controller=s.getObject('/customizableTable/SutureTask/SutureController')
s.callScriptFunction('setLiveMode',driver,True);s.setStepping(True);s.startSimulation()
h=s.createVisionSensor(1+2+4+64+128,[900,900,0,0],[.001,2,np.radians(52),.001,0,0,.15,.17,.19,0,0])
s.setObjectInt32Param(h,s.visionintparam_render_mode,0);s.setObjectInt32Param(h,s.objintparam_visibility_layer,256)
checks=[]
left_clear=next(i for i,r in enumerate(rows) if r['phase']=='left_handoff')
for index,arm,label in [(0,2,'right_initial'),(events[0],1,'left_handoff'),(left_clear,1,'left_extracted'),(events[1],2,'right_regrasp')]:
    s.callScriptFunction('previewFrame',driver,index+1)
    for a in [1,2]:s.callScriptFunction('setSutureArm',controller,a,f[index,(a-1)*11:a*11].astype(float).tolist())
    s.step();state=s.callScriptFunction('getSutureAudit',controller)
    assert np.max(abs(np.array(state['joints']).ravel()-f[index,:22]))<1e-6
    T=np.array(state['tcps'][arm-1]).reshape(3,4)
    eye=T[:,:3]@np.array([.014,.012,.010])+T[:,3]
    target=T[:,:3]@np.array([0,0,-.003])+T[:,3]
    z=target-eye;z/=np.linalg.norm(z);x=np.cross(T[:3,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    s.setObjectMatrix(h,np.column_stack([x,y,z,eye]).ravel().tolist(),-1)
    s.handleVisionSensor(h);raw,res=s.getVisionSensorImg(h)
    Image.frombytes('RGB',tuple(res),bytes(raw)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(DATA/(label+'.png'))
    # The nominal bite now lies 3 mm behind the old TCP; actual section
    # centering is calibrated separately against both jaw/needle meshes.
    key='L' if arm==1 else 'R';N=np.array(state['needle']).reshape(3,4)
    grasp=np.array(m['grasp_correction']['calibration'][key]['needle_grasp_local'])
    local=T[:,:3].T@(N[:,:3]@grasp+N[:,3]-T[:,3])
    assert -.0036<local[2]<-.0024 and abs(local[1])<.0002
    checks.append(dict(frame=index+1,arm=key,needle_grip_in_tcp_mm=(local*1000).tolist()))
s.pauseSimulation();s.setStepping(False)
(DATA/'actual_grasp_validation.json').write_text(json.dumps(dict(result='PASS',checks=checks),indent=2))
print(checks)
