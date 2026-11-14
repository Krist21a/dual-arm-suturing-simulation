"""Prepare only the disposable, already collision-reviewed scene copy."""
import sys,json,hashlib
from pathlib import Path
ROOT=Path(r'D:\仿真环境')
sys.path[:0]=[str(ROOT/'work/python_deps'),r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from plan_wrist_motion import np,fk
p=json.loads((ROOT/'work/suture_review/wrist_plan.json').read_text())
rows=json.loads((ROOT/'work/suture_review/live_plan_audit.json').read_text())
bad=json.loads((ROOT/'work/suture_review/collisions.json').read_text());assert not bad
assert len(rows)==sum(len(x['q']) for x in p['phases'])
(ROOT/'output/reports/suture_plan_validation.json').write_text(json.dumps({'pass':True,'frames':len(rows),'collision_count':0,'plan_sha256':hashlib.sha256((ROOT/'work/suture_review/wrist_plan.json').read_bytes()).hexdigest()}))
c=RemoteAPIClient(port=23001);c.initialTimeout=120;s=c.require('sim')
assert s.getSimulationState()==s.simulation_stopped
assert 'codex_suture_test' in s.getStringParam(s.stringparam_scene_path)
code=(ROOT/'work/suture_review/audit_runtime.lua').read_text()+'\n'+(ROOT/'work/scene_inspection/record_suture_stereo.lua').read_text()+'\nfunction sysCall_init() end\n'
h=s.getObject('/customizableTable/SutureTask/SutureController',{'noError':True})
if h<0:
    h=s.createScript(s.scripttype_customization,code,0,'lua')
    s.setObjectAlias(h,'SutureController');s.setObjectParent(h,s.getObject('/customizableTable/SutureTask'),True)
else:s.setScriptStringParam(h,s.scriptstringparam_text,code)
s.initScript(h)
def call(name,*args):return s.callScriptFunction(name,h,*args)
call('initSutureAudit')
for i,a in enumerate(['L','R'],1):
    q=p['initial'][a].copy();q[9:]=[.35,.35] if a=='L' else [0,0];call('setSutureArm',i,q)
tn=np.array(p['calibration']['R'])@fk(np.array(p['initial']['R']),'R')@np.array(p['relations']['R'])
call('setSutureNeedle',tn[:3].ravel().tolist(),2)
call('captureAuditImage','C:/Users/Kuo/AppData/Local/Temp/codex_suture_video/audit_initial_corrected')
print('controller',h,'initial collision',call('getSutureAudit')['arm_arm'])
s.saveScene('C:/Users/Kuo/AppData/Local/Temp/codex_suture_test/dual_DVRK_ARM_suture.ttt')
print('Saved reviewed temporary scene; original file unchanged.')
