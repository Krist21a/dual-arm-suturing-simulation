"""Apply each preplanned configuration in the stopped scene and check meshes."""
import sys,json,hashlib
from pathlib import Path
ROOT=Path(r'D:\仿真环境')
sys.path[:0]=[str(ROOT/'work/python_deps'),r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient(port=23001);c.initialTimeout=120;s=c.require('sim')
assert s.getSimulationState()==s.simulation_stopped
assert 'suture_review' in s.getStringParam(s.stringparam_scene_path).replace('\\','/') or 'suture_test' in s.getStringParam(s.stringparam_scene_path)
h=s.getScript(s.scripttype_addonscript,'record_suture_stereo')
s.callScriptFunction('loadSutureAudit',h,(ROOT/'work/suture_review/audit_runtime.lua').read_text())
def call(name,*args):return s.callScriptFunction(name,h,*args)
call('initSutureAudit')
p=json.loads((ROOT/'work/suture_review/wrist_plan.json').read_text())
for key,name in [('entryPoint','EntryPoint'),('exitPoint','ExitPoint')]:
    s.setObjectPosition(s.getObject('/customizableTable/SutureTask/'+name),p[key],-1)
q={a:np.array(p['initial'][a]) for a in ['L','R']}
q['L'][9:]=.35
for a in ['L','R']:call('setSutureArm',1 if a=='L' else 2,q[a].tolist())
from plan_wrist_motion import fk
init=np.array(p['calibration']['R'])@fk(q['R'],'R')@np.array(p['relations']['R'])
call('setSutureNeedle',init[:3].ravel().tolist(),2)
rows=[];bad=[]
grip={'L':.35,'R':0}
for phase in p['phases']:
    a=phase['arm'];idx=1 if a=='L' else 2
    traj=np.array(phase['q']);traj[:,9:]=grip[a]
    batch=[]
    for start in range(0,len(traj),10):batch.extend(call('auditSutureBatch',idx,traj[start:start+10].tolist()))
    for k,st in enumerate(batch):
        st.update(phase=phase['name'],step=k);rows.append(st)
        hits={n:v for n,v in st.items() if isinstance(v,dict) and v.get('collision',0)>0}
        if hits:bad.append({'phase':phase['name'],'step':k,'hits':hits})
    if phase['name']=='left_grasp':
        grip={'L':0,'R':.35};call('handoffSutureNeedle',1)
    if phase['name']=='right_regrasp':
        grip={'L':.35,'R':0};call('handoffSutureNeedle',2)
    for ia,side in enumerate(['L','R'],1):
        qs=batch[-1]['joints'][ia-1];qs[9:]=[grip[side]]*2;call('setSutureArm',ia,qs)
    prefix='C:/Users/Kuo/AppData/Local/Temp/codex_suture_video/audit_'+phase['name']
    call('captureAuditImage',prefix)
    print(phase['name'],'hits',sum(x['phase']==phase['name'] for x in bad),flush=True)
    (ROOT/'work/suture_review/collisions.json').write_text(json.dumps(bad,indent=2))
(ROOT/'work/suture_review/live_plan_audit.json').write_text(json.dumps(rows))
(ROOT/'work/suture_review/collisions.json').write_text(json.dumps(bad,indent=2))
print('collisions',len(bad), 'frames',len(rows))
(ROOT/'output/reports/suture_plan_validation.json').write_text(json.dumps({'pass':not bad,'frames':len(rows),'collision_count':len(bad),'plan_sha256':hashlib.sha256((ROOT/'work/suture_review/wrist_plan.json').read_bytes()).hexdigest()}))
