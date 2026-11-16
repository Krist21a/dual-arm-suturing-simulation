"""Record all three synchronized cameras, then append labelled detail replays.

High-quality image buffers go directly to the encoder: no multi-GB BMP dump.
Every camera in a frame is rendered after the same checked mechanics state.
"""
import argparse,hashlib,io,json,os,shutil,subprocess,tempfile,time
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
import imageio_ffmpeg
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from kinematics import fk
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/soft_suture/video'
VIDEO=ROOT/'output/videos/soft_suture_5x3mm_overview_stereo.mp4'

CAPTURE=r'''
sim=require('sim')
function captureMechanicalFrame(index,detail)
    local names={'/SutureOverviewFront','/visionSensor_left','/visionSensor_right','/SoftSutureVideoCloseup'}
    local images={}
    for i,name in ipairs(names)do
      if i<4 or detail then
        local h=sim.getObject(name)
        sim.handleVisionSensor(h)
        local img,res=sim.getVisionSensorImg(h)
        if i==2 or i==3 then img,res=sim.getScaledImage(img,res,{432,384},0)end
        images[i]=sim.saveImage(img,res,0,'.jpg',98)
      end
    end
    return {frame=index,images=images}
end
'''

def camera(s,name,eye,target,res,fov):
    eye=np.array(eye,dtype=float);z=np.array(target)-eye;z/=np.linalg.norm(z)
    x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    h=s.createVisionSensor(1+2+4+64+128,res+[0,0],[.005,10,np.radians(fov),.01,0,0,.16,.19,.22,0,0])
    s.setObjectAlias(h,name);s.setObjectInt32Param(h,s.objintparam_visibility_layer,256)
    s.setObjectMatrix(h,np.column_stack([x,y,z,eye]).ravel().tolist(),-1)
    s.setObjectInt32Param(h,s.visionintparam_render_mode,0);s.setExplicitHandling(h,1)
    return h

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--preview',action='store_true');args=ap.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);VIDEO.parent.mkdir(exist_ok=True)
    assert shutil.disk_usage(OUT).free>2*1024**3,'Need 2 GiB working reserve'
    manifest=json.loads((ROOT/'output/soft_suture/manifest.json').read_text())
    rows=json.loads((ROOT/'output/soft_suture/metrics.json').read_text())
    count=manifest['frame_count'];assert count==len(rows)==3506
    states=np.fromfile(ROOT/'output/soft_suture/frames.bin',dtype='<f4').reshape(count,manifest['state_width'])
    plan=json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text())
    max_joint_error=0.;max_tcp_error=0.
    first=int(manifest['puncture_events'][0]['time']/.05)+1
    puncture=list(range(max(1,first-20),min(count,first+36)))
    tightening=list(range(manifest['before_tightening']['frame']+1,count+1))
    keep=set(puncture+tightening)
    c=RemoteAPIClient(port=int(os.getenv('COPPELIA_ZMQ_PORT','23002')));c.initialTimeout=120;s=c.require('sim')
    if s.getSimulationState()!=s.simulation_stopped:
        assert s.getObject('/SoftSuturePlayback',dict(noError=True))>=0
        s.stopSimulation()
        for _ in range(100):
            if s.getSimulationState()==s.simulation_stopped:break
            time.sleep(.1)
    assert s.getSimulationState()==s.simulation_stopped
    stage=Path(tempfile.gettempdir())/'codex_soft_suture';stage.mkdir(exist_ok=True)
    source=ROOT/'output/scenes/dual_DVRK_ARM_soft_suture.ttt';copy=stage/'record_soft_suture.ttt'
    shutil.copy2(source,copy);s.loadScene(copy.as_posix())
    # In the recording copy, make the kinematic callback explicitly consume
    # each target. An 'immobile' callback can preserve a previous joint state
    # when frames are selected externally between simulation steps.
    main=s.getScript(s.scripttype_main)
    s.setScriptStringParam(main,s.scriptstringparam_text,"""require('defaultMainScript')
local original=sysCall_joint
function sysCall_joint(d)
    if d.mode==sim.jointmode_kinematic then
        return {pos=d.targetPos or d.pos,vel=0,accel=0,immobile=false}
    end
    return original(d)
end
""")
    camera(s,'SutureOverviewFront',[.05,1.80,1.25],[-.01,-.01,.80],[1920,1440],42)
    camera(s,'SoftSutureVideoCloseup',[-.03,.395,.675],[-.03,.351,.589],[960,720],50)
    fixed={name:s.getObjectPose(s.getObject(name),-1) for name in ['/visionSensor_left','/visionSensor_right']}
    for name in fixed:
        h=s.getObject(name);s.setExplicitHandling(h,1)
        s.setObjectInt32Param(h,s.visionintparam_resolution_x,432)
        s.setObjectInt32Param(h,s.visionintparam_resolution_y,384)
    capture=s.createScript(s.scripttype_customization,CAPTURE,0,'lua');s.setObjectAlias(capture,'SoftVideoCapture');s.initScript(capture)
    s.setStepping(True);s.startSimulation();s.step()
    driver=s.getObject('/SoftSuturePlayback')
    s.callScriptFunction('setLiveMode',driver,True)
    controller=s.getObject('/customizableTable/SutureTask/SutureController')
    font=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',26)
    small=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',22)
    big=ImageFont.truetype('C:/Windows/Fonts/msyh.ttc',30)
    phases={'right_prepare':'进针准备','right_contact':'针尖接触','right_insert':'穿刺','right_expose':'露针',
        'left_extract':'抽针','left_handoff':'连续送线','right_regrasp':'右臂接针','tighten':'拉紧缝线','hold_tension':'保持张力'}
    def compose(images,index,detail=None):
        im=Image.open(io.BytesIO(images[0])).convert('RGB')
        for j,x in [(1,520),(2,968)]:im.paste(Image.open(io.BytesIO(images[j])).convert('RGB'),(x,28))
        d=ImageDraw.Draw(im)
        for x,label in [(520,'左目'),(968,'右目')]:
            d.rectangle((x,28,x+70,60),fill=(20,28,36));d.text((x+8,28),label,font=small,fill='white')
        r=rows[index-1]
        phase=phases.get(r['phase'],'双臂连续缝合')
        d.rectangle((0,1374,1920,1440),fill=(20,28,36))
        caption=f'第 {r["stitch"]}/5 针 · {phase}    |    仿真时间 {index*.05:.2f} s'
        if r['phase'] in ['tighten','hold_tension']:
            caption+=f'    |    切口 {min(r["gap_mm"]):.2f}–{max(r["gap_mm"]):.2f} mm'
        d.text((28,1384),caption,font=font,fill='white')
        d.text((28,1412),'软组织与缝线张力原型 · 参数未实验标定',font=small,fill=(180,195,207))
        if detail:
            d.rectangle((472,416,1448,1176),fill=(20,28,36))
            im.paste(Image.open(io.BytesIO(images[3])).convert('RGB'),(480,448))
            d.text((492,416),detail,font=font,fill='white')
        return im
    def capture_frame(i):
        nonlocal max_joint_error,max_tcp_error
        s.callScriptFunction('previewFrame',driver,i)
        q=states[i-1,:22].astype(float).reshape(2,11)
        # Explicit top-level controller updates avoid nested paused-script
        # calls leaving renderer joint transforms from an earlier state.
        for a in [1,2]:s.callScriptFunction('setSutureArm',controller,a,q[a-1].tolist())
        s.step()
        actual=s.callScriptFunction('getSutureAudit',controller)
        joint_error=float(np.max(abs(np.array(actual['joints'])-q)));max_joint_error=max(max_joint_error,joint_error)
        assert joint_error<1e-7,('Recorded joint state mismatch',i,np.array(actual['joints'])-q)
        for a,key in enumerate(['L','R']):
            expected=np.array(plan['calibration'][key])@fk(q[a],key)
            error=float(np.max(abs(np.array(actual['tcps'][a]).reshape(3,4)-expected[:3])))
            max_tcp_error=max(max_tcp_error,error)
            assert error<1e-5,('TCP pose mismatch',i,key,error)
        for key in ['arm_arm','left_pad','right_pad','left_table','right_table','left_self','right_self']:
            assert actual[key]['collision']==0,('Collision during capture',i,key)
        assert np.max(abs(np.array(actual['needle'])-states[i-1,22:34]))<1e-7,'Recorded needle state mismatch'
        r=s.callScriptFunction('captureMechanicalFrame',capture,i,i in keep or args.preview)
        assert r['frame']==i
        return [bytes(v) for v in r['images']]
    if args.preview:
        for i,name in [(first,'puncture'),(count,'closed')]:
            images=capture_frame(i)
            compose(images,i).save(OUT/f'preview_{name}.png')
            compose(images,i,'局部放大 · 同步实景').save(OUT/f'preview_{name}_detail.png')
        s.pauseSimulation();s.setStepping(False)
        print('Preview ready',flush=True);return
    ff=imageio_ffmpeg.get_ffmpeg_exe()
    temporary=VIDEO.with_name(VIDEO.stem+'.partial.mp4')
    log=(OUT/'encoder.log').open('w',encoding='utf-8')
    cmd=[ff,'-y','-v','warning','-f','rawvideo','-pix_fmt','rgb24','-s','1920x1440','-r','20','-i','-',
         '-an','-c:v','libx264','-preset','fast','-crf','17','-pix_fmt','yuv420p','-movflags','+faststart',str(temporary)]
    proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log)
    cached={};written=0;start=time.time();last_notice=0
    try:
        for i in range(1,count+1):
            images=capture_frame(i)
            frame=compose(images,i);proc.stdin.write(frame.tobytes());written+=1
            if i in keep:cached[i]=images
            if i in [first,count]:frame.save(OUT/('recorded_puncture.png' if i==first else 'recorded_closed.png'))
            if time.time()-last_notice>20:
                status=dict(frame=i,total=count,elapsed_seconds=round(time.time()-start))
                print(status,flush=True);(OUT/'progress.json').write_text(json.dumps(status));last_notice=time.time()
        chapters=[dict(name='完整五针缝合与收口',start_seconds=0,frames=count,speed=1)]
        for indices,repeats,label in [(puncture,3,'穿刺变形局部回看 · 1/3 速'),(tightening,2,'拉紧收口局部回看 · 1/2 速')]:
            chapters.append(dict(name=label,start_seconds=written/20,source_first_frame=indices[0],source_last_frame=indices[-1],speed=1/repeats))
            for i in indices:
                raw=compose(cached[i],i,label).tobytes()
                for _ in range(repeats):proc.stdin.write(raw);written+=1
        last=compose(cached[count],count,'拉紧后：五个针位接近闭合').tobytes()
        for _ in range(40):proc.stdin.write(last);written+=1
        proc.stdin.close();assert proc.wait()==0,'Video encoder failed'
    except BaseException:
        if proc.poll() is None:proc.kill();proc.wait()
        raise
    finally:log.close()
    for name,pose in fixed.items():assert np.max(abs(np.array(s.getObjectPose(s.getObject(name),-1))-pose))<1e-10
    s.pauseSimulation();s.setStepping(False)
    subprocess.run([ff,'-v','error','-i',str(temporary),'-f','null','-'],check=True)
    temporary.replace(VIDEO)
    report=dict(result='PASS',motion_frames=count,encoded_frames=written,fps=20,duration_seconds=written/20,
        resolution=[1920,1440],chapters=chapters,stereo_camera_poses_unchanged=True,
        panels=dict(left=[520,28,432,384],right=[968,28,432,384]),
        synchronization='All cameras rendered after one common mechanical state; identical source frame index',
        capture_format='JPEG quality 98 in memory; stereo rendered at its displayed resolution; no camera pose/FOV changes',
        scene_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        physics_frames_sha256=hashlib.sha256((ROOT/'output/soft_suture/frames.bin').read_bytes()).hexdigest(),
        maximum_recorded_joint_error=max_joint_error,maximum_tcp_matrix_error=max_tcp_error,reference_collision_events=0,
        video_sha256=hashlib.sha256(VIDEO.read_bytes()).hexdigest(),bytes=VIDEO.stat().st_size,
        elapsed_seconds=time.time()-start)
    (OUT/'validation.json').write_text(json.dumps(report,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False),flush=True)

if __name__=='__main__':main()
