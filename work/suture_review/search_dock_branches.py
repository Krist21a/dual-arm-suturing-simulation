import json,sys
from pathlib import Path
from plan_wrist_motion import *
sys.path.insert(0,r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient(port=23001);c.initialTimeout=120;s=c.require('sim')
assert s.getSimulationState()==s.simulation_stopped
h=s.getScript(s.scripttype_addonscript,'record_suture_stereo')
def call(name,*args):return s.callScriptFunction(name,h,*args)
call('initSutureAudit')
p=json.loads((ROOT/'work/suture_review/wrist_plan.json').read_text())
ql=np.array(p['initial']['L']);ql[9:]=.35;call('setSutureArm',1,ql.tolist())
rng=np.random.default_rng(18);found=[]
target=needle(0,[0,0,.025])@np.linalg.inv(rel['R'])
for k in range(36):
    q=np.r_[q0['R'][:6],.06,np.pi,np.radians(80),0,0]
    seed=q[:6] if k==0 else q[:6]+rng.uniform(-np.pi,np.pi,6)
    def fun(x):
        qq=q.copy();qq[:6]=x;return residual(qq,'R',target)
    opt=least_squares(fun,seed,max_nfev=180,gtol=1e-11,ftol=1e-11,xtol=1e-11)
    if np.linalg.norm(opt.fun)>1e-6:continue
    q[:6]=q0['R'][:6]+(opt.x-q0['R'][:6]+np.pi)%(2*np.pi)-np.pi
    if any(np.linalg.norm(q-x['q'])<.01 for x in found):continue
    call('setSutureArm',2,q.tolist());st=call('getSutureAudit')
    hit={key:st[key] for key in ['arm_arm','right_pad','right_table'] if st[key]['collision']}
    found.append({'q':q.tolist(),'hits':hit,'cost':float(np.linalg.norm(q[:6]-q0['R'][:6]))})
    print(k,'hits',hit,'q',q.round(5).tolist(),flush=True)
(ROOT/'work/suture_review/dock_candidates.json').write_text(json.dumps(sorted(found,key=lambda x:(bool(x['hits']),x['cost'])),indent=2))
