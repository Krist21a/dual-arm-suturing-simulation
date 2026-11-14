import sys,json
from pathlib import Path
ROOT=Path(r'D:\仿真环境');sys.path[:0]=[str(ROOT/'work/python_deps'),r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient(port=23001);c.initialTimeout=120;s=c.require('sim')
out=[]
for h in s.getObjectsInTree(s.handle_scene,s.sceneobject_script,0):
    name=s.getObjectAlias(h,1);code=s.getScriptStringParam(h,s.scriptstringparam_text);out.append({'handle':h,'name':name,'code':code})
(ROOT/'work/suture_review/embedded_scripts.json').write_text(json.dumps(out),encoding='utf-8')
for x in out:
    if 'SutureController' not in x['name']:print(x['handle'],x['name'],x['code'])
