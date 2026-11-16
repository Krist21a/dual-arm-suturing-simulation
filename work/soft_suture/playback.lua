-- Replay states computed by the coupled implicit mechanical solver.
-- This scene is a reproducible result viewer, not an in-scene live solver.
sim=require('sim')
local controller,buffer,tissue,line,frame,done,failure,collisions
local function call(name,...)return sim.callScriptFunction(name,controller,...)end
local function render(v)
    local q1,q2={},{}
    for i=1,11 do q1[i]=v[i];q2[i]=v[i+11]end
    call('setSutureArm',1,q1);call('setSutureArm',2,q2)
    local matrix={};for i=1,12 do matrix[i]=v[22+i]end
    call('setSutureNeedle',matrix,0)
    local triangles={}
    for _,face in ipairs(config.faces)do
        for _,vertex in ipairs(face)do
            for j=1,3 do triangles[#triangles+1]=v[34+vertex*3+j]end
        end
    end
    sim.addDrawingObjectItem(tissue,nil)
    sim.addDrawingObjectItem(tissue+sim.handleflag_addmultiple,triangles)
    local count=v[35+config.nodes*3];local base=35+config.nodes*3
    local segments={}
    for i=1,count-1 do
        for j=1,3 do segments[#segments+1]=v[base+(i-1)*3+j]end
        for j=1,3 do segments[#segments+1]=v[base+i*3+j]end
    end
    sim.addDrawingObjectItem(line,nil)
    if #segments>0 then sim.addDrawingObjectItem(line+sim.handleflag_addmultiple,segments)end
end
local function draw(index)
    local off=(index-1)*config.width*4
    render(sim.unpackFloatTable(buffer:sub(off+1,off+config.width*4)))
end
function sysCall_init()
    controller=sim.getObject('/customizableTable/SutureTask/SutureController')
    call('initSutureAudit')
    buffer=sim.readCustomDataBlock(sim.getObject('/SoftSuturePlayback'),'soft_suture_frames')
    assert(#buffer==config.width*4*config.frames,'mechanical result buffer size mismatch')
    for _,name in ipairs({'TissueLeft','TissueRight','WoundVisual','SoftTissuePreview'})do
        local h=sim.getObject('/customizableTable/SutureTask/'..name,{noError=true})
        if h>=0 then sim.setObjectInt32Param(h,sim.objintparam_visibility_layer,0)end
    end
    tissue=sim.addDrawingObject(sim.drawing_triangles+sim.drawing_painttag,1,0,-1,#config.faces,{.93,.55,.55})
    line=sim.addDrawingObject(sim.drawing_lines+sim.drawing_painttag,2,0,-1,2048,{.08,.25,.65})
    frame=0;done=false;failure='';collisions={};draw(1)
end
function sysCall_actuation()
    if config.live then return end
    if done or failure~='' then return end
    local ok,err=xpcall(function()frame=frame+1;draw(frame)end,debug.traceback)
    if not ok then failure=err;sim.pauseSimulation()end
end
function sysCall_sensing()
    if config.live then return end
    if done or failure~='' then return end
    local ok,err=xpcall(function()
        -- The original rigid envelope is a conservative reference only;
        -- deformable contact is handled by the external mechanical solver.
        local st=call('getSutureAudit')
        for _,key in ipairs({'arm_arm','left_pad','right_pad','left_table','right_table','left_self','right_self'})do
            if st[key].collision~=0 then collisions[#collisions+1]={frame=frame,pair=key}end
        end
        if frame==config.frames then done=true;sim.pauseSimulation()end
    end,debug.traceback)
    if not ok then failure=err;sim.pauseSimulation()end
end
function getSoftProgress()return {frame=frame,total=config.frames,done=done,error=failure,collisions=collisions}end
function previewFrame(index)draw(index);return true end
function setLiveMode(enabled)config.live=enabled;return true end
function applyLiveState(values)render(values);return true end
