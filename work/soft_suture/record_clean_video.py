"""Native synchronized capture: panorama + stereo only, no text or replay."""
import os,json,time,tempfile,shutil,subprocess,hashlib,argparse
from pathlib import Path
import numpy as np
from PIL import Image
import imageio_ffmpeg
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from record_video import camera
ROOT=Path(__file__).resolve().parents[2]
DATA=Path(os.getenv('SOFT_DATA_DIR',str(ROOT/'output/soft_suture_continuous')))
OUT=DATA/'video';OUT.mkdir(parents=True,exist_ok=True)
VIDEO=Path(os.getenv('SOFT_VIDEO_PATH',str(ROOT/'output/videos/soft_suture_continuous_thread_clean.mp4')))
HOOK=r'''
local cleanCaptured=0
local cleanJointError=0
local originalSensing=sysCall_sensing
function sysCall_sensing()
    originalSensing()
    if failure~='' or frame<=cleanCaptured then return end
    local ok,err=xpcall(function()
        local off=(frame-1)*config.width*4
        local v=sim.unpackFloatTable(buffer:sub(off+1,off+config.width*4))
        local st=call('getSutureAudit')
        for a=1,2 do for j=1,11 do
            local error=math.abs(st.joints[a][j]-v[(a-1)*11+j])
            cleanJointError=math.max(cleanJointError,error)
            assert(error<1e-6,'joint mismatch at frame '..frame)
        end end
        assert(v[35+config.nodes*3]>=2,'missing continuous thread')
        assert(#collisions==0,'arm/environment collision at frame '..frame)
        for i,name in ipairs({'/SutureOverviewFront','/visionSensor_left','/visionSensor_right'})do
            local sensor=sim.getObject(name)
            sim.handleVisionSensor(sensor)
            local rgb,res=sim.getVisionSensorImg(sensor)
            sim.saveImage(rgb,res,0,cleanDirectory..string.format('/%s_%06d.jpg',({'front','left','right'})[i],frame),98)
        end
        cleanCaptured=frame
    end,debug.traceback)
    if not ok then failure=err;sim.pauseSimulation()end
end
function getCleanProgress()
    return {frame=cleanCaptured,total=config.frames,done=cleanCaptured==config.frames,
        error=failure,max_joint_error=cleanJointError,collisions=collisions}
end
'''

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--encode-only',action='store_true');args=parser.parse_args()
    manifest=json.loads((DATA/'manifest.json').read_text());count=manifest['frame_count']
    frames=Path(os.getenv('SOFT_CLEAN_FRAME_DIR',str(OUT/'frames')));frames.mkdir(parents=True,exist_ok=True)
    assert shutil.disk_usage(frames.resolve()).free>4*1024**3
    if not args.encode_only:
        client=RemoteAPIClient(port=int(os.getenv('COPPELIA_ZMQ_PORT','23002')));s=client.require('sim')
        if s.getSimulationState()!=s.simulation_stopped:
            assert s.getObject('/SoftSuturePlayback',dict(noError=True))>=0
            s.stopSimulation()
            for _ in range(100):
                if s.getSimulationState()==s.simulation_stopped:break
                time.sleep(.1)
        assert s.getSimulationState()==s.simulation_stopped
        source=Path(os.getenv('SOFT_SCENE_PATH',str(ROOT/'output/scenes/dual_DVRK_ARM_soft_suture_continuous_thread.ttt')))
        stage=Path(tempfile.gettempdir())/'codex_soft_suture';stage.mkdir(exist_ok=True)
        copy=stage/'clean_recording.ttt';shutil.copy2(source,copy);s.loadScene(copy.as_posix())
        camera(s,'SutureOverviewFront',[.05,1.80,1.25],[-.01,-.01,.80],[1920,1440],42)
        fixed={}
        for name in ['/visionSensor_left','/visionSensor_right']:
            h=s.getObject(name);fixed[name]=s.getObjectPose(h,-1);s.setExplicitHandling(h,1)
            s.setObjectInt32Param(h,s.visionintparam_resolution_x,432)
            s.setObjectInt32Param(h,s.visionintparam_resolution_y,384)
        driver=s.getObject('/SoftSuturePlayback');code=s.getScriptStringParam(driver,s.scriptstringparam_text)
        code+='\ncleanDirectory='+json.dumps(frames.as_posix(),ensure_ascii=False)+'\n'+HOOK
        s.setScriptStringParam(driver,s.scriptstringparam_text,code)
        s.setStepping(False);s.startSimulation();notice=time.time()-30;started=time.time()
        while True:
            time.sleep(2);progress=s.callScriptFunction('getCleanProgress',driver)
            if time.time()-notice>20:
                print(progress['frame'],count,'elapsed',round(time.time()-started),flush=True);notice=time.time()
                (OUT/'progress.json').write_text(json.dumps(progress))
            if progress['error']:raise RuntimeError(progress['error'])
            if progress['done']:break
        for name,pose in fixed.items():assert np.max(abs(np.array(s.getObjectPose(s.getObject(name),-1))-pose))<1e-10
        progress['camera_poses_unchanged']=True
        (OUT/'capture_validation.json').write_text(json.dumps(progress,indent=2))
    check=json.loads((OUT/'capture_validation.json').read_text());assert check['done'] and check['frame']==count
    for name in ['front','left','right']:
        assert len(list(frames.glob(name+'_*.jpg')))==count
    ff=imageio_ffmpeg.get_ffmpeg_exe()
    temporary=VIDEO.with_name(VIDEO.stem+'.partial.mp4')
    cmd=[ff,'-y','-v','warning','-filter_complex_threads','2']
    for name in ['front','left','right']:cmd+=['-framerate','20','-start_number','1','-i',str(frames/(name+'_%06d.jpg'))]
    hold=13 if manifest.get('playback_speed')==3 else 40
    layout=f'[0:v][1:v]overlay=520:28[a];[a][2:v]overlay=968:28,tpad=stop_mode=clone:stop_duration={hold/20}[out]'
    cmd+=['-filter_complex',layout,'-map','[out]','-an','-c:v','libx264','-preset','fast','-crf','17',
          '-pix_fmt','yuv420p','-movflags','+faststart',str(temporary)]
    subprocess.run(cmd,check=True)
    actual,duration=imageio_ffmpeg.count_frames_and_secs(str(temporary))
    assert actual==count+hold and abs(duration-(count+hold)/20)<.05
    subprocess.run([ff,'-v','error','-i',str(temporary),'-f','null','-'],check=True)
    temporary.replace(VIDEO)
    for i,name in [(1,'initial'),(430,'thread_feeding'),(count,'closed')]:
        im=Image.open(frames/f'front_{i:06d}.jpg').convert('RGB')
        im.paste(Image.open(frames/f'left_{i:06d}.jpg'),(520,28))
        im.paste(Image.open(frames/f'right_{i:06d}.jpg'),(968,28))
        im.save(OUT/(name+'.png'))
    report=dict(result='PASS',motion_frames=count,frames=actual,fps=20,duration_seconds=duration,
        playback_speed=manifest.get('playback_speed',1),
        resolution=[1920,1440],text_overlays=False,slow_motion=False,
        continuous_thread_from_frame_one=True,matched_jaw_grasp=True,
        camera_poses_unchanged=True,maximum_joint_error=check['max_joint_error'],
        sha256=hashlib.sha256(VIDEO.read_bytes()).hexdigest(),bytes=VIDEO.stat().st_size)
    (OUT/'validation.json').write_text(json.dumps(report,indent=2));print(report,flush=True)

if __name__=='__main__':main()
