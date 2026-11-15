from common import *
import shutil,time
STAGE.mkdir(parents=True,exist_ok=True)
shutil.copy2(ROOT/'output/scenes/dual_DVRK_ARM_suture.ttt',SCENE)
c,s=connect()
print('Connected',s.getStringParam(s.stringparam_scene_path),s.getSimulationState(),flush=True)
s.stopSimulation()
for _ in range(100):
    if s.getSimulationState()==s.simulation_stopped:break
    time.sleep(.1)
assert s.getSimulationState()==s.simulation_stopped
s.loadScene(str(SCENE).replace('\\','/'))
print('Auto pause:',s.getBoolProperty(s.handle_scene,'pauseSimulationAtTime'),s.getFloatProperty(s.handle_scene,'timeToPause'),flush=True)
s.setBoolProperty(s.handle_scene,'pauseSimulationAtTime',False)
# All motion is prescribed kinematics; mesh collision checks remain active.
# Avoid MuJoCo rebuilding a dynamics world for every moved static link.
s.setBoolParam(s.boolparam_dynamics_handling_enabled,False)
main=s.getScript(s.scripttype_main)
s.setScriptStringParam(main,s.scriptstringparam_text,"""require('defaultMainScript')
-- Trajectory samples are applied explicitly by the reviewed external runner.
-- Do not integrate a second, stale Ruckig profile on top of those samples.
local originalJointCallback=sysCall_joint
function sysCall_joint(inData)
    if inData.mode==sim.jointmode_kinematic then return {immobile=true} end
    return originalJointCallback(inData)
end
""")
for path in ['/UR5','/500____[1]/UR5','/Needle']:
    sh=s.getScript(s.scripttype_simulation,s.getObject(path))
    if sh>=0:
        old=s.getScriptStringParam(sh,s.scriptstringparam_text)
        if 'simRemoteApi.start(19999)' in old or 'Clip' in old:
            (ROOT/'work/continuous_suture'/('legacy_'+str(sh)+'.lua')).write_text(old,encoding='utf-8')
            s.setScriptInt32Param(sh,s.scriptintparam_enabled,0)
            print('Disabled unused legacy script',path,flush=True)
p=json.loads(PLAN.read_text())
h=s.getObject('/customizableTable/SutureTask/SutureController')
code=(ROOT/'work/suture_review/audit_runtime.lua').read_text()+'\n'+(ROOT/'work/continuous_suture/continuous_thread.lua').read_text()+'\nfunction sysCall_init() end\n'
s.setScriptStringParam(h,s.scriptstringparam_text,code);s.initScript(h)
h,call=initialize(s,p)
for root in ['/UR5','/500____[1]/UR5']:
    # This is a prescribed kinematic demonstration. Dynamic link bodies
    # otherwise free-fall behind kinematic joints and detach visual meshes.
    for shape in s.getObjectsInTree(s.getObject(root),s.sceneobject_shape,0):
        s.setObjectInt32Param(shape,s.shapeintparam_static,1)
    for j in range(1,12):
        joint=s.getObject(root+('/' if j<8 else '/L0_respondable_TOOL2/')+'joint'+str(j))
        s.setJointMode(joint,s.jointmode_kinematic,0)
        # In kinematic mode a retained target may still drive an old profile.
        s.setJointTargetPosition(joint,s.getJointPosition(joint))
fixed=call('getSutureAudit')['fixed']
s.saveScene(str(SCENE).replace('\\','/'))
shutil.copy2(SCENE,ROOT/'output/scenes/dual_DVRK_ARM_continuous_suture.ttt')
print('Prepared separate scene',s.getStringParam(s.stringparam_scene_name),'fixed',fixed)
