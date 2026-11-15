"""Add synchronized, uncropped stereo panels in the overview's upper blank area."""
from common import *
import subprocess,hashlib,argparse
import imageio_ffmpeg
from PIL import Image
parser=argparse.ArgumentParser();parser.add_argument('--preview',action='store_true');args=parser.parse_args()
stereo=STEREO_FRAMES;ff=imageio_ffmpeg.get_ffmpeg_exe()
layout='[0:v]format=rgb24[b];[1:v]scale=432:384,format=rgb24[l];[2:v]scale=432:384,format=rgb24[r];[b][l]overlay=520:28:format=rgb[t];[t][r]overlay=968:28:format=rgb[out]'
preview=ROOT/'output/previews/continuous_suture_stereo_inset.png'
if args.preview:
    cmd=[ff,'-y','-v','error','-i',str(FRAMES/'front_000001.bmp'),'-i',str(stereo/'left_000001.bmp'),'-i',str(stereo/'right_000001.bmp'),'-filter_complex',layout,'-map','[out]','-frames:v','1',str(preview)]
    subprocess.run(cmd,check=True)
    original=np.array(Image.open(FRAMES/'front_000001.bmp').convert('RGB'));result=np.array(Image.open(preview).convert('RGB'))
    mask=np.ones(original.shape[:2],bool);mask[28:412,520:952]=False;mask[28:412,968:1400]=False
    assert np.array_equal(original[mask],result[mask]),'Overview pixels outside panels changed'
    print(preview);raise SystemExit
status=(stereo/'status.txt').read_text().splitlines();assert status[0]=='complete';count=int(status[1].split('=')[1])
old=[json.loads(x) for x in (ROOT/'output/reports/continuous_suture_live_frames.jsonl').read_text().splitlines()]
new=[json.loads(x) for x in (ROOT/'output/reports/continuous_suture_stereo_live_frames.jsonl').read_text().splitlines()]
assert len(old)==len(new)==count-12
max_error=0
for a,b in zip(old,new):
    for key in ['frame','stitch','phase','step','thread','owner']:assert a[key]==b[key],(a['frame'],key)
    for key in ['needle','joints','fixed','tcps','simulation_time']:
        error=float(np.max(abs(np.array(a[key])-np.array(b[key]))));max_error=max(max_error,error);assert error<1e-9,(a['frame'],key,error)
for i in range(1,count+1):
    for side in ['left','right']:
        with Image.open(stereo/f'{side}_{i:06d}.bmp') as im:assert im.size==(1080,960)
out=ROOT/'output/videos/continuous_suture_5x3mm_overview_stereo.mp4'
cmd=[ff,'-y','-v','warning','-filter_complex_threads','2']
for path in [FRAMES/'front_%06d.bmp',stereo/'left_%06d.bmp',stereo/'right_%06d.bmp']:
    cmd+=['-framerate','20','-start_number','1','-i',str(path)]
cmd+=['-filter_complex',layout,'-map','[out]','-frames:v',str(count),'-an','-c:v','libx264','-preset','fast','-crf','17','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
subprocess.run(cmd,check=True)
subprocess.run([ff,'-v','error','-i',str(out),'-f','null','-'],check=True)
report=dict(result='PASS',frames=count,fps=20,duration_seconds=count/20,resolution=[1920,1440],synchronized_motion_frames=len(new),max_replay_state_difference=max_error,
    panels={'left':{'x':520,'y':28,'width':432,'height':384},'right':{'x':968,'y':28,'width':432,'height':384}},
    processing='Original full overview with proportional, uncropped stereo insets; no time shift or speed change',sha256=hashlib.sha256(out.read_bytes()).hexdigest())
original_hash=hashlib.sha256((ROOT/'output/videos/continuous_suture_5x3mm_overview.mp4').read_bytes()).hexdigest()
assert original_hash==json.loads((ROOT/'output/reports/continuous_suture_video_validation.json').read_text())['sha256']
report['original_overview_unchanged']=True
(ROOT/'output/reports/continuous_suture_stereo_inset_validation.json').write_text(json.dumps(report,indent=2))
print(out,flush=True)
