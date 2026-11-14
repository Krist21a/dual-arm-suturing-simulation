"""Encode every synchronized sensor frame, with phase labels and keyframes."""
import sys,json,subprocess
from pathlib import Path
ROOT=Path(r'D:\仿真环境');sys.path.insert(0,str(ROOT/'work/python_deps'))
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
frames=Path(r'C:\Users\Kuo\AppData\Local\Temp\codex_suture_video_final')
out=ROOT/'output/videos/single_interrupted_suture_stereo.mp4'
labels={'initial':'初始：双侧末端姿态一致，右臂持针','right_prepare':'右臂：末端姿态准备','right_contact':'右臂：下降至入针点','right_insert':'右臂：绕针圆心穿针 89.38°','right_expose':'右臂：继续沿弧露出可夹持针体','left_prepare':'左臂：末端姿态准备','left_grasp':'左臂：接近已露出的针体','left_close':'左臂：闭合夹爪','right_to_left':'第一次交接：右 → 左','right_open':'右臂：松开夹爪','right_retreat':'右臂：安全撤离','left_extract':'左臂：沿弧拔针，完全出组织后拉线','left_handoff':'左臂：安全区域递针','right_pregrasp':'右臂：接近递针区域','right_regrasp':'右臂：重新夹持针体','right_close':'右臂：闭合夹爪','left_to_right':'第二次交接：左 → 右','left_open':'左臂：松开夹爪','left_retreat':'左臂：安全撤离','complete':'完成：右臂持针，准备下一针'}
rows=[json.loads(x) for x in (ROOT/'output/reports/suture_live_frames.jsonl').read_text().splitlines()]
status=(frames/'status.txt').read_text().splitlines();assert status[0]=='complete'
count=int(status[1].split('=')[1]);images=[frames/f'left_{i:06d}.png' for i in range(1,count+1)]
assert all(x.exists() and (frames/x.name.replace('left_','right_')).exists() for x in images)
font=ImageFont.truetype(r'C:\Windows\Fonts\msyh.ttc',30);small=ImageFont.truetype(r'C:\Windows\Fonts\msyh.ttc',24)
ff=imageio_ffmpeg.get_ffmpeg_exe()
command=[ff,'-y','-v','warning','-f','rawvideo','-vcodec','rawvideo','-pix_fmt','rgb24','-s','2160x1080','-r','20','-i','-','-an','-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(out)]
process=subprocess.Popen(command,stdin=subprocess.PIPE,stderr=subprocess.PIPE)
phaseFrames={};last=None
for i,path in enumerate(images,1):
    state=rows[min(i-1,len(rows)-1)];phase=state['phase'];phaseFrames[phase]=i
    canvas=Image.new('RGB',(2160,1080),(18,24,31));d=ImageDraw.Draw(canvas)
    canvas.paste(Image.open(path).convert('RGB'),(0,64));canvas.paste(Image.open(frames/path.name.replace('left_','right_')).convert('RGB'),(1080,64))
    d.text((24,14),'LEFT | visionSensor_left',font=font,fill='white');d.text((1104,14),'RIGHT | visionSensor_right',font=font,fill='white')
    d.text((24,1032),labels.get(phase,phase),font=small,fill=(215,230,245));d.text((1770,1032),f'{i/20:05.2f}s  |  {i}/{len(images)}',font=small,fill=(180,200,220))
    process.stdin.write(canvas.tobytes())
    if phase!=last:print(i,phase,flush=True);last=phase
process.stdin.close();error=process.stderr.read().decode(errors='replace');status=process.wait();assert status==0,error
check=subprocess.run([ff,'-v','error','-i',str(out),'-f','null','-'],capture_output=True,text=True);assert check.returncode==0,check.stderr
selected=['initial','right_contact','right_insert','left_close','right_retreat','left_extract','right_close','complete']
sheet=Image.new('RGB',(1600,1600),'white');draw=ImageDraw.Draw(sheet)
for k,phase in enumerate(selected):
    i=phaseFrames[phase];pair=Image.new('RGB',(2160,960));pair.paste(Image.open(frames/f'left_{i:06d}.png'),(0,0));pair.paste(Image.open(frames/f'right_{i:06d}.png'),(1080,0))
    x=(k%2)*800;y=(k//2)*400;sheet.paste(pair.resize((800,356)),(x,y+40));draw.text((x+10,y+7),f'{i/20:.2f}s  '+labels[phase],font=ImageFont.truetype(r'C:\Windows\Fonts\msyh.ttc',18),fill='black')
sheet.save(ROOT/'output/previews/suture_keyframes.jpg',quality=94)
(ROOT/'output/reports/suture_video_validation.json').write_text(json.dumps({'file':str(out),'frames_per_eye':len(images),'fps':20,'duration_seconds':len(images)/20,'width':2160,'height':1080,'codec':'H.264','decode_test':'PASS','phase_last_frames':phaseFrames},ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS',out,len(images),'frames',flush=True)
