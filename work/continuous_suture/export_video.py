from common import *
import subprocess,hashlib
import imageio_ffmpeg
from PIL import Image
frames=FRAMES;status=(frames/'status.txt').read_text().splitlines()
assert status[0]=='complete';count=int(status[1].split('=')[1])
report=json.loads((ROOT/'output/reports/continuous_suture_final_validation.json').read_text())
assert report['result']=='PASS' and count==report['checked_frames']+12
out=ROOT/'output/videos/continuous_suture_5x3mm_overview.mp4'
ff=imageio_ffmpeg.get_ffmpeg_exe()
for i in range(1,count+1):
    with Image.open(frames/f'front_{i:06d}.bmp') as im:assert im.size==(1920,1440)
subprocess.run([ff,'-y','-v','warning','-framerate','20','-start_number','1','-i',str(frames/'front_%06d.bmp'),'-frames:v',str(count),'-an','-c:v','libx264','-preset','fast','-crf','17','-pix_fmt','yuv420p','-movflags','+faststart',str(out)],check=True)
subprocess.run([ff,'-v','error','-i',str(out),'-f','null','-'],check=True)
(ROOT/'output/previews/continuous_suture_final.bmp').write_bytes((frames/f'front_{count:06d}.bmp').read_bytes())
meta=dict(result='PASS',frames=count,fps=20,duration_seconds=count/20,resolution=[1920,1440],editing='None: fixed single camera, direct frame encoding; no overlays, cropping, compositing or speed change',sha256=hashlib.sha256(out.read_bytes()).hexdigest())
(ROOT/'output/reports/continuous_suture_video_validation.json').write_text(json.dumps(meta,indent=2))
print(out,count,flush=True)
