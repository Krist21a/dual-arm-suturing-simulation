"""Embed a newly computed mechanics run in a separate, standalone .ttt."""
import os,json,time,shutil,tempfile,sys,argparse
from pathlib import Path
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT=Path(__file__).resolve().parents[2]

def lua(v):
    if isinstance(v,dict):return '{'+','.join('['+json.dumps(k)+']='+lua(x) for k,x in v.items())+'}'
    if isinstance(v,list):return '{'+','.join(lua(x) for x in v)+'}'
    if isinstance(v,bool):return 'true' if v else 'false'
    return json.dumps(v,ensure_ascii=False)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--data',type=Path,default=ROOT/'output/soft_suture')
    ap.add_argument('--scene-name',default='dual_DVRK_ARM_soft_suture.ttt');ap.add_argument('--preview-only',action='store_true')
    args=ap.parse_args();data=args.data;manifest=json.loads((data/'manifest.json').read_text())
    c=RemoteAPIClient(port=int(os.environ.get('COPPELIA_ZMQ_PORT','23002')));c.initialTimeout=30
    s=c.require('sim')
    if s.getSimulationState()!=s.simulation_stopped and s.getObject('/SoftSuturePlayback',dict(noError=True))>=0:
        s.stopSimulation()
        for _ in range(100):
            if s.getSimulationState()==s.simulation_stopped:break
            time.sleep(.1)
    assert s.getSimulationState()==s.simulation_stopped,'Stop the current simulation first'
    stage=Path(tempfile.gettempdir())/'codex_soft_suture';stage.mkdir(exist_ok=True)
    source=stage/'source.ttt'
    shutil.copy2(ROOT/'output/scenes/dual_DVRK_ARM_continuous_suture.ttt',source)
    s.loadScene(source.as_posix())
    main_script=s.getScript(s.scripttype_main)
    s.setScriptStringParam(main_script,s.scriptstringparam_text,"""require('defaultMainScript')
local original=sysCall_joint
function sysCall_joint(d)
    if d.mode==sim.jointmode_kinematic then return {pos=d.targetPos or d.pos,vel=0,accel=0,immobile=false}end
    return original(d)
end
""")
    for name in ['/ContinuousPlayback','/ContinuousRecorder','/SoftSuturePlayback']:
        h=s.getObject(name,dict(noError=True))
        if h>=0:s.removeObjects([h])
    task=s.getObject('/customizableTable/SutureTask')
    for marker in s.getObjectsInTree(s.handle_scene,s.sceneobject_dummy,0):
        s.setObjectInt32Param(marker,s.objintparam_visibility_layer,256)
    for marker in s.getObjectsInTree(s.handle_scene,s.sceneobject_shape,0):
        if s.getObjectAlias(marker) in ['needle_tip','needle_hand','sever_tip']:
            s.setObjectInt32Param(marker,s.objintparam_visibility_layer,0)
    for name in ['TissueLeft','TissueRight','WoundVisual']:
        s.setObjectInt32Param(s.getObject('/customizableTable/SutureTask/'+name),s.objintparam_visibility_layer,0)
    raw=(data/'frames.bin').read_bytes()
    initial=np.frombuffer(raw,dtype='<f4',count=manifest['state_width'])
    vertices=initial[34:34+manifest['tissue_nodes']*3].astype(float).tolist()
    mesh=s.createShape(0,0,vertices,np.array(manifest['faces']).ravel().tolist())
    s.setObjectAlias(mesh,'SoftTissuePreview');s.setObjectParent(mesh,task,True)
    s.setShapeColor(mesh,None,s.colorcomponent_ambient_diffuse,[.93,.55,.55])
    s.setObjectInt32Param(mesh,s.shapeintparam_static,1)
    s.setObjectInt32Param(mesh,s.shapeintparam_respondable,0)
    config=dict(width=manifest['state_width'],frames=manifest['frame_count'],
                nodes=manifest['tissue_nodes'],faces=manifest['faces'])
    code='config='+lua(config)+'\n'+(Path(__file__).parent/'playback.lua').read_text()
    driver=s.createScript(s.scripttype_simulation,code,0,'lua');s.setObjectAlias(driver,'SoftSuturePlayback')
    s.writeCustomDataBlock(driver,'soft_suture_frames',raw)
    s.writeCustomDataBlock(driver,'soft_suture_manifest',json.dumps(manifest).encode())
    s.setBoolParam(s.boolparam_dynamics_handling_enabled,False)
    s.setBoolProperty(s.handle_scene,'pauseSimulationAtTime',False)
    scene=ROOT/'output/scenes'/args.scene_name
    staged_scene=stage/scene.name
    s.saveScene(staged_scene.as_posix());shutil.copy2(staged_scene,scene)
    print('Saved',scene,flush=True)
    if not args.preview_only:
        for sensor in s.getObjectsInTree(s.handle_scene,s.sceneobject_visionsensor,0):
            s.setExplicitHandling(sensor,1)
    s.startSimulation()
    if '--preview-only' in sys.argv:
        time.sleep(1);s.pauseSimulation();return
    last=-1
    while True:
        time.sleep(2);r=s.callScriptFunction('getSoftProgress',driver)
        if r['frame']//500!=last:
            print(dict(frame=r['frame'],error=r['error'],collision_count=len(r['collisions'])),flush=True);last=r['frame']//500
        if r['error']:raise RuntimeError(r['error'])
        if r['done']:break
    r['result']='PASS' if not r['collisions'] else 'COLLISIONS'
    (data/'scene_validation.json').write_text(json.dumps(r,indent=2))
    print(dict(result=r['result'],frame=r['frame'],collision_count=len(r['collisions'])),flush=True)

if __name__=='__main__':main()
