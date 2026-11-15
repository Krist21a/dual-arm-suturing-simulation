sim=require('sim')
local directory,frame,active=nil,0,false
local handles={}
local function status(value)
    local f=assert(io.open(directory..'/status.txt','w'));f:write(value..'\nframes='..frame..'\n');f:close()
end
function sysCall_init()end
function startSutureRecording(path)
    directory=path;frame=0;active=true
    handles={sim.getObject('/SutureOverviewFront')}
    for _,h in ipairs(handles)do sim.setExplicitHandling(h,1)end
    status('recording');return 0
end
function captureSutureFrame()
    assert(active);frame=frame+1
    for i,name in ipairs({'front'})do
        sim.handleVisionSensor(handles[i]);local img,res=sim.getVisionSensorImg(handles[i])
        sim.saveImage(img,res,0,directory..string.format('/%s_%06d.bmp',name,frame),-1)
    end
    return frame
end
function finishSutureRecording()
    for k=1,12 do captureSutureFrame()end
    status('complete');active=false;return frame
end
function abortSutureRecording()if active then status('aborted')end;active=false;return frame end
