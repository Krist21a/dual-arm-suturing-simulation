"""Add chapter metadata, correct the footer margin, and verify final delivery."""
import json,hashlib,subprocess
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/soft_suture/video'
video=ROOT/'output/videos/soft_suture_5x3mm_overview_stereo.mp4'
r=json.loads((OUT/'validation.json').read_text(encoding='utf-8'))
assert r['result']=='PASS'
ff=imageio_ffmpeg.get_ffmpeg_exe()
# Keep every caption pixel inside the 1440-pixel canvas.
footer=Image.new('RGB',(1920,23),(20,28,36))
ImageDraw.Draw(footer).text((28,-1),'软组织与缝线张力原型 · 参数未实验标定',
    font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',18),fill=(180,195,207))
footer.save(OUT/'footer.png')
meta=[';FFMETADATA1']
for i,ch in enumerate(r['chapters']):
    end=r['chapters'][i+1]['start_seconds'] if i+1<len(r['chapters']) else r['duration_seconds']
    meta+=['[CHAPTER]','TIMEBASE=1/1000',f'START={round(ch["start_seconds"]*1000)}',
           f'END={round(end*1000)}','title='+ch['name']]
(OUT/'chapters.txt').write_text('\n'.join(meta)+'\n',encoding='utf-8')
temp=video.with_name(video.stem+'.finalizing.mp4')
subprocess.run([ff,'-y','-v','warning','-i',str(video),'-i',str(OUT/'footer.png'),'-i',str(OUT/'chapters.txt'),
    '-filter_complex','[0:v][1:v]overlay=0:1417:eof_action=repeat[out]',
    '-map','[out]','-map_metadata','2','-map_chapters','2','-an','-c:v','libx264',
    '-preset','fast','-crf','16','-pix_fmt','yuv420p','-movflags','+faststart',str(temp)],check=True)
count,duration=imageio_ffmpeg.count_frames_and_secs(str(temp))
assert count==r['encoded_frames'] and abs(duration-r['duration_seconds'])<.1
subprocess.run([ff,'-v','error','-i',str(temp),'-f','null','-'],check=True)
video.replace(OUT/'capture_original.mp4');temp.replace(video)
times=[3.,12.5,90.,175.2,r['chapters'][1]['start_seconds']+3.,r['chapters'][2]['start_seconds']+14.]
board=Image.new('RGB',(1920,1030),(245,247,250));draw=ImageDraw.Draw(board)
font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',22)
for i,t in enumerate(times):
    p=OUT/f'encoded_check_{i}.png'
    subprocess.run([ff,'-y','-v','error','-ss',str(t),'-i',str(video),'-frames:v','1',str(p)],check=True)
    x=(i%3)*640;y=(i//3)*515
    board.paste(Image.open(p).resize((640,480)),(x,y+35))
    draw.text((x+12,y+4),f'{int(t)//60:02d}:{t%60:04.1f}',font=font,fill=(20,35,55))
board.save(OUT/'encoded_contact_sheet.png')
r.update(video_sha256=hashlib.sha256(video.read_bytes()).hexdigest(),bytes=video.stat().st_size,
    decoded_frame_count=count,decoded_duration_seconds=duration,embedded_chapters=True,
    footer_margin_corrected=True,full_decode_check='PASS')
(OUT/'validation.json').write_text(json.dumps(r,indent=2,ensure_ascii=False),encoding='utf-8')
print(json.dumps(r,ensure_ascii=False),flush=True)
