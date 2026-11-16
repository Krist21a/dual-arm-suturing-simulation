import json,shutil,tempfile
from pathlib import Path
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT=Path(__file__).resolve().parents[2]
s=RemoteAPIClient(port=23002).require('sim')
assert s.getSimulationState()==s.simulation_stopped
stage=Path(tempfile.gettempdir())/'codex_soft_suture';stage.mkdir(exist_ok=True)
source=stage/'jaw_inspection.ttt';shutil.copy2(ROOT/'output/scenes/dual_DVRK_ARM_soft_suture.ttt',source)
s.loadScene(source.as_posix())
h=s.getObject('/customizableTable/SutureTask/SutureController');s.initScript(h);s.callScriptFunction('initSutureAudit',h)
p=json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text())
result={}
for a,root in enumerate(['/UR5','/500____[1]/UR5'],1):
    q=p['initial']['L' if a==1 else 'R'].copy();q[9:]=[0,0]
    s.callScriptFunction('setSutureArm',h,a,q)
    grasp=s.getObject('/suture_grasp_arm'+str(a));shapes=[]
    for shape in s.getObjectsInTree(s.getObject(root+'/L0_respondable_TOOL2'),s.sceneobject_shape,0):
        name=s.getObjectAlias(shape)
        if 'L3_' not in name:continue
        vertices,triangles,*_=s.getShapeMesh(shape)
        M=np.array(s.getObjectMatrix(shape,grasp)).reshape(3,4)
        vertices=np.array(vertices).reshape(-1,3)@M[:,:3].T+M[:,3]
        shapes.append(dict(name=name,vertices=vertices.tolist(),triangles=triangles))
        print(a,name,'bounds mm',vertices.min(axis=0)*1000,vertices.max(axis=0)*1000,flush=True)
    result[str(a)]=shapes
(ROOT/'work/soft_suture/jaw_geometry.json').write_text(json.dumps(result))
