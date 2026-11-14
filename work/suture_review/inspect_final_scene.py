import sys,json,csv
from pathlib import Path
ROOT=Path(r'D:\仿真环境');sys.path[:0]=[str(ROOT/'work/python_deps'),r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient(port=23002);c.initialTimeout=30;s=c.require('sim')
paths=['/customizableTable/SutureTask','/customizableTable/SutureTask/PadBase','/customizableTable/SutureTask/TissueLeft','/customizableTable/SutureTask/TissueRight','/customizableTable/SutureTask/EntryPoint','/customizableTable/SutureTask/ExitPoint','/customizableTable/SutureTask/NeedleArcCenter','/customizableTable/SutureTask/SutureController','/visionSensor_left','/visionSensor_right','/Needle','/Needle/needle_tip','/Needle/needle_hand']
for i,r in enumerate(['/UR5','/500____[1]/UR5'],1):
    paths+=[r+'/L0_respondable_TOOL2/instrument_tcp_arm'+str(i)]
    paths+=[r+('/' if j<8 else '/L0_respondable_TOOL2/')+'joint'+str(j) for j in range(1,12)]
with (ROOT/'output/reports/suture_scene_path_validation.tsv').open('w',encoding='utf-8',newline='')as f:
    w=csv.writer(f,delimiter='\t');w.writerow(['path','result','x','y','z'])
    for p in paths:
        h=s.getObject(p);w.writerow([p,'PASS']+s.getObjectPosition(h,-1))
for p in ['/Needle','/UR5','/500____[1]/UR5']:
    h=s.getScript(s.scripttype_simulation,s.getObject(p))
    if h>=0:print(p,s.getScriptStringParam(h,s.scriptstringparam_text))
print('Scene paths PASS',len(paths))
