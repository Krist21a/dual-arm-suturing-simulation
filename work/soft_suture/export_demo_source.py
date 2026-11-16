"""Export exact embedded demo scripts and check a standalone scene replay."""
import json,shutil,tempfile,time,hashlib
from pathlib import Path
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT=Path(__file__).resolve().parents[2]
out=ROOT/'output/delivery/natural_3x_demo';(out/'code').mkdir(parents=True,exist_ok=True)
scene=ROOT/'output/scenes/dual_DVRK_ARM_soft_suture_natural_3x.ttt'
shutil.copy2(scene,out/'scene.ttt')
stage=Path(tempfile.gettempdir())/'codex_soft_suture/delivery_check.ttt';shutil.copy2(out/'scene.ttt',stage)
s=RemoteAPIClient(port=23002).require('sim');assert s.getSimulationState()==s.simulation_stopped
s.loadScene(stage.as_posix())
scripts={'main.lua':s.getScript(s.scripttype_main),'playback.lua':s.getObject('/SoftSuturePlayback'),'controller.lua':s.getObject('/customizableTable/SutureTask/SutureController')}
for name,h in scripts.items():(out/'code'/name).write_text(s.getScriptStringParam(h,s.scriptstringparam_text),encoding='utf-8')
driver=scripts['playback.lua'];blob=s.readCustomDataBlock(driver,'soft_suture_frames')
assert hashlib.sha256(bytes(blob)).hexdigest()==hashlib.sha256((ROOT/'output/soft_suture_natural_3x/frames.bin').read_bytes()).hexdigest()
for h in s.getObjectsInTree(s.handle_scene,s.sceneobject_visionsensor,0):s.setExplicitHandling(h,1)
s.startSimulation()
while True:
 time.sleep(2);r=s.callScriptFunction('getSoftProgress',driver)
 assert not r['error'],r['error']
 if r['done']:break
assert r['frame']==1170 and not r['collisions']
r.update(result='PASS',scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),embedded_states_match=True)
(out/'standalone_validation.json').write_text(json.dumps(r,indent=2))
print(r,flush=True)
