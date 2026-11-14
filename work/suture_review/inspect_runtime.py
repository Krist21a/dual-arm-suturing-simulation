import sys,json
from pathlib import Path
sys.path[:0]=[r'D:\仿真环境\work\python_deps',r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient(port=23001);c.initialTimeout=120;s=c.require('sim')
h=s.getScript(s.scripttype_addonscript,'record_suture_stereo')
state=s.callScriptFunction('getSutureAudit',h)
for k,v in state.items():
    if isinstance(v,dict):
        print(k,v,[s.getObjectAlias(x,1) for x in v.get('pair',[]) if x>=0])
bad=json.loads(Path('work/suture_review/collisions.json').read_text())
pairs={tuple(v['pair']) for x in bad for v in x['hits'].values()}
for pair in pairs:print(pair,[s.getObjectAlias(x,1) for x in pair])
