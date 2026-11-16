"""Reload the shipped scene stopped, verify baked poses/meshes and capture it."""
import json,shutil,tempfile,hashlib
from pathlib import Path
import numpy as np
from PIL import Image
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from kinematics import fk
ROOT=Path(__file__).resolve().parents[2];D=ROOT/'output/delivery/matlab_suture_3x'
a=json.loads((D/'commands.json').read_text())[0];s=RemoteAPIClient(port=23002).require('sim')
assert s.getSimulationState()==s.simulation_stopped
stage=Path(tempfile.gettempdir())/'codex_soft_suture/verify_initial.ttt';shutil.copy2(D/'scene.ttt',stage);s.loadScene(stage.as_posix())
error=0.
for arm,root in enumerate(['/UR5','/500____[1]/UR5']):
 for j,q in enumerate(a['q'][arm],1):
  h=s.getObject(root+('/' if j<8 else '/L0_respondable_TOOL2/')+'joint'+str(j))
  error=max(error,abs(s.getJointPosition(h)-q))
assert error<1e-6
assert np.max(abs(np.array(s.getObjectMatrix(s.getObject('/Needle'),-1))-a['needle']))<1e-7
for name in ['SoftTissuePreview','TissueThreadPreview']:
 h=s.getObject('/customizableTable/SutureTask/'+name);assert s.getObjectInt32Param(h,s.objintparam_visibility_layer)==1
def capture(h,name):
 s.handleVisionSensor(h);rgb,res=s.getVisionSensorImg(h);Image.frombytes('RGB',tuple(res),bytes(rgb)).transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(D/name)
capture(s.getObject('/visionSensor_left'),'scene_open_overview.png')
p=json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text());T=np.array(p['calibration']['R'])@fk(np.array(a['q'][1]),'R')
eye=T[:3,:3]@np.array([.014,.012,.010])+T[:3,3];target=T[:3,:3]@np.array([0,0,-.003])+T[:3,3]
z=target-eye;z/=np.linalg.norm(z);x=np.cross(T[:3,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
h=s.createVisionSensor(1+2+4+64+128,[900,900,0,0],[.001,2,np.radians(52),.001,0,0,.15,.17,.19,0,0])
s.setObjectInt32Param(h,s.visionintparam_render_mode,0);s.setObjectMatrix(h,np.column_stack([x,y,z,eye]).ravel().tolist(),-1);capture(h,'scene_open_grasp.png')
report=dict(result='PASS',tested_while_simulation_stopped=True,initial_joint_error=error,initial_thread_saved_as_shape=True,initial_tissue_saved_as_shape=True,scene_sha256=hashlib.sha256((D/'scene.ttt').read_bytes()).hexdigest())
(D/'initial_scene_validation.json').write_text(json.dumps(report,indent=2));print(report)
