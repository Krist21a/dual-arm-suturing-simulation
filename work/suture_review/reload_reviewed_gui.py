"""Reload the byte-identical ASCII staging copy in the existing review GUI."""
import sys,time,hashlib
from pathlib import Path
ROOT=Path(r'D:\仿真环境')
sys.path[:0]=[str(ROOT/'work/python_deps'),r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src']
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
stage=Path(r'C:\Users\Kuo\AppData\Local\Temp\codex_suture_test\dual_DVRK_ARM_suture.ttt')
assert hashlib.sha256(stage.read_bytes()).digest()==hashlib.sha256((ROOT/'output/scenes/dual_DVRK_ARM_suture.ttt').read_bytes()).digest()
c=RemoteAPIClient(port=23002);c.initialTimeout=120;s=c.require('sim')
assert s.getStringParam(s.stringparam_scene_name)=='dual_DVRK_ARM_suture.ttt'
s.stopSimulation()
for k in range(40):
    if s.getSimulationState()==s.simulation_stopped:break
    time.sleep(.1)
assert s.getSimulationState()==s.simulation_stopped
s.loadScene(str(stage).replace('\\','/'))
print('Reloaded delivered scene (identical SHA256).')
