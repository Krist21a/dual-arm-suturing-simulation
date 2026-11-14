import sys,json,time
from pathlib import Path
sys.path[:0]=[r'D:\仿真环境\work\python_deps',r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
c=RemoteAPIClient();s=c.require('sim');root=Path(r'D:\仿真环境')
assert s.getStringParam(s.stringparam_scene_name)=='dual_DVRK_ARM_suture.ttt'
action=sys.argv[1]
if action=='reset':
    if s.getSimulationState()!=s.simulation_stopped:
        s.stopSimulation()
        while s.getSimulationState()!=s.simulation_stopped:time.sleep(.1)
    # Stop resets the existing scene; a disk reload additionally restores parents.
    s.loadScene(str(root/'output/scenes/dual_DVRK_ARM_suture.ttt'))
elif action=='reload_recorder':
    h=s.getScript(s.scripttype_addonscript,'record_suture_stereo')
    code=(root/'work/scene_inspection/record_suture_stereo.lua').read_text(encoding='utf-8')
    print(s.executeScriptString(code,h))
elif action=='inspect':
    for path in ['/customizableTable/SutureTask','/customizableTable/SutureTask/PadBase','/customizableTable/SutureTask/TissueLeft','/customizableTable/SutureTask/TissueRight','/visionSensor_left','/visionSensor_right']+['/customizableTable/ring'+str(i) for i in range(6,12)]:
        h=s.getObject(path);print(path,'pose',s.getObjectPose(h,-1),'layer',s.getObjectInt32Param(h,s.objintparam_visibility_layer))
    for path in ['/UR5/L0_respondable_TOOL2/Needle','/UR5/L0_respondable_TOOL2/needle_tip','/UR5/L0_respondable_TOOL2/needle_hand']:
        h=s.getObject(path);print(path,'parent',s.getObjectAlias(s.getObjectParent(h),1),'pose',s.getObjectPose(h,s.getObject('/UR5/L0_respondable_TOOL2/Needle')))
elif action=='save':
    assert s.getSimulationState()==s.simulation_stopped
    s.saveScene(str(root/'output/scenes/dual_DVRK_ARM_suture.ttt'))
