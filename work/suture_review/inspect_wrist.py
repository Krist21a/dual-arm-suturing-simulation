import sys,json
from pathlib import Path
sys.path[:0]=[r'D:\仿真环境\work\python_deps',r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient();s=c.require('sim')
assert s.getSimulationState()==s.simulation_stopped
out={}
for name,root in [('L','/UR5'),('R','/500____[1]/UR5')]:
    root+='/L0_respondable_TOOL2';j9=s.getObject(root+'/joint9');out[name]={}
    for suffix in ['joint9','joint10','joint11','L2_visual_TOOL2','L2_respondable_TOOL2','instrument_tcp_arm'+('1' if name=='L' else '2')]:
        h=s.getObject(root+'/'+suffix,{'noError':True})
        if h<0:continue
        row={'handle':h,'parent':s.getObjectAlias(s.getObjectParent(h),1),'relative9':s.getObjectMatrix(h,j9)}
        if 'joint' in suffix:row['q']=s.getJointPosition(h)
        out[name][suffix]=row
Path('work/suture_review/wrist_geometry.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out,indent=2))
