"""Make a minimal MATLAB/online-physics package; remove embedded replay states."""
import json,shutil,tempfile,time
from pathlib import Path
import numpy as np
from build_scene import lua
from trajectory import build_actions
from bake_initial_state import bake
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT=Path(__file__).resolve().parents[2];out=ROOT/'output/delivery/matlab_suture_3x';out.mkdir(parents=True,exist_ok=True)
for directory in ['physics','matlab','scene_code']:(out/directory).mkdir(exist_ok=True)
for name in ['mechanics.py','natural_strand.py','physics_worker.py']:shutil.copy2(ROOT/'work/soft_suture'/name,out/'physics'/name)
(out/'physics/requirements.txt').write_text('numpy>=1.26\nscipy>=1.12\n')
for p in (ROOT/'work/soft_suture/matlab').iterdir():shutil.copy2(p,out/'matlab'/p.name)
shutil.copy2(ROOT/'work/soft_suture/main_suture.m',out/'main_suture.m')
shutil.copy2(ROOT/'output/videos/soft_suture_natural_3x_clean.mp4',out/'reference_video.mp4')
m=json.loads((ROOT/'output/soft_suture_natural_3x/manifest.json').read_text())
full,_=build_actions(json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text()))
f=np.fromfile(ROOT/'output/soft_suture_continuous/frames.bin',dtype='<f4').reshape(-1,1721)
assert len(full)==len(f)
commands=[]
for i in m['source_indices']:
 a=full[i];assert np.max(abs(np.array(a['needle'])-f[i,22:34]))<1e-6
 commands.append(dict(q=f[i,:22].reshape(2,11).astype(float).tolist(),needle=f[i,22:34].astype(float).tolist(),phase=a['phase'],stitch=a['stitch'],theta=a['theta']))
(out/'commands.json').write_text(json.dumps(commands,separators=(',',':')))
s=RemoteAPIClient(port=23002).require('sim')
if s.getSimulationState()!=s.simulation_stopped:
 s.stopSimulation()
 while s.getSimulationState()!=s.simulation_stopped:time.sleep(.1)
stage=Path(tempfile.gettempdir())/'codex_soft_suture/online_build.ttt';shutil.copy2(ROOT/'output/scenes/dual_DVRK_ARM_soft_suture_natural_3x.ttt',stage);s.loadScene(stage.as_posix())
old=s.getObject('/SoftSuturePlayback');s.removeObjects([old])
code='config='+lua(dict(nodes=m['tissue_nodes'],faces=m['faces'],initial_q=commands[0]['q'],initial_needle=commands[0]['needle']))+'\n'+(ROOT/'work/soft_suture/online_scene.lua').read_text()
h=s.createScript(s.scripttype_simulation,code,0,'lua');s.setObjectAlias(h,'OnlineSoftSuture')
s.setFloatParam(s.floatparam_simulation_time_step,.05)
bake(s,commands[0],m['faces'])
s.saveScene(stage.as_posix());shutil.copy2(stage,out/'scene.ttt')
for name,h in {'online_geometry.lua':h,'controller.lua':s.getObject('/customizableTable/SutureTask/SutureController'),'main.lua':s.getScript(s.scripttype_main)}.items():
 (out/'scene_code'/name).write_text(s.getScriptStringParam(h,s.scriptstringparam_text),encoding='utf-8')
assert s.getObject('/SoftSuturePlayback',dict(noError=True))<0
print('Built online scene with no embedded frame player;',len(commands),'MATLAB commands',flush=True)
