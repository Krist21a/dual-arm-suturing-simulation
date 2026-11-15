from common import *
import time,shutil
c,s=connect();s.stopSimulation()
for _ in range(100):
    if s.getSimulationState()==s.simulation_stopped:break
    time.sleep(.1)
assert s.getSimulationState()==s.simulation_stopped
STAGE.mkdir(parents=True,exist_ok=True)
shutil.copy2(ROOT/'output/scenes/dual_DVRK_ARM_continuous_suture.ttt',SCENE)
s.loadScene(str(SCENE).replace('\\','/'))
source=(ROOT/'work/continuous_suture/run_native.py').read_text()
source=source.replace('frames=FRAMES','frames=STEREO_FRAMES')
source=source.replace("ROOT/'work/continuous_suture/raw_frames'","ROOT/'work/continuous_suture/stereo_frames'")
source=source.replace('(len(actions)+12)*1920*1440*3','(len(actions)+12)*1080*960*3*2')
source=source.replace('work/continuous_suture/record_overview.lua','work/continuous_suture/record_stereo.lua')
source=source.replace('continuous_suture_final_validation.json','continuous_suture_stereo_capture_validation.json')
source=source.replace('continuous_suture_live_frames.jsonl','continuous_suture_stereo_live_frames.jsonl')
source=source.replace('work/continuous_suture/progress.json','work/continuous_suture/stereo_progress.json')
(ROOT/'work/continuous_suture/run_stereo.py').write_text(source,encoding='utf-8')
print('Stereo capture prepared; original overview and validation preserved.')
