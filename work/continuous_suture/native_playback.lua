sim=require('sim')
local controller,recorder,frame,warmup,previous,done,failure
local rows,transfers={},{}
local function call(name,...)return sim.callScriptFunction(name,controller,...)end
local function near(a,b,tol)
    for i=1,#a do assert(math.abs(a[i]-b[i])<tol,'value mismatch '..i)end
end
local function apply(action)
    if action.arm>0 then call('setSutureArm',action.arm,action.q[action.arm])end
    if action.receiver>0 then
        local pair=call('handoffSutureNeedle',action.receiver);near(pair[1],pair[2],1e-9)
        local delta=0;for j=1,12 do delta=math.max(delta,math.abs(pair[1][j]-pair[2][j]))end
        transfers[#transfers+1]={stitch=action.stitch,receiver=action.receiver==1 and 'L' or 'R',max_matrix_change=delta}
    end
    return call('updateContinuousThread',action.stitch,action.theta,action.extracted)
end
function sysCall_init()
    controller=sim.getObject('/customizableTable/SutureTask/SutureController')
    recorder=sim.getObject('/ContinuousRecorder',{noError=true})
    call('initSutureAudit')
    for a=1,2 do call('setSutureArm',a,config.initial[a])end
    call('setSutureNeedle',config.needle,2)
    call('initContinuousThread',config.stitches)
    frame=0;warmup=0;done=false;failure=nil
    if recorder>=0 then sim.callScriptFunction('startSutureRecording',recorder,config.frames)end
end
local function protect(fn)
    if failure or done then return end
    local ok,err=xpcall(fn,debug.traceback)
    if not ok then failure=err;sim.pauseSimulation();if recorder>=0 then sim.callScriptFunction('abortSutureRecording',recorder)end end
end
function sysCall_actuation()
    protect(function()
        if warmup<6 then
            for a=1,2 do call('setSutureArm',a,config.initial[a])end
        else apply(config.actions[frame+1])end
    end)
end
function sysCall_sensing()
    protect(function()
        if warmup<6 then warmup=warmup+1;return end
        frame=frame+1;local action=config.actions[frame];local st=call('getSutureAudit')
        local checks={'arm_arm','left_pad','right_pad','left_table','right_table','left_self','right_self'}
        for _,key in ipairs(checks)do assert(st[key].collision==0,action.name..' collision '..key)end
        for a=1,2 do
            near(st.joints[a],action.q[a],1e-5)
            assert(st.joints[a][7]>=-1e-9 and st.joints[a][7]<=.16000001,'joint7')
            assert(math.abs(st.joints[a][8])<=math.pi+1e-8,'joint8')
            assert(math.abs(st.joints[a][9])<=math.rad(85)+1e-8,'joint9')
            for j=1,6 do assert(math.abs(st.joints[a][j]-config.initial[a][j])<=math.rad(5)+1e-8,'proximal')end
        end
        for i=1,3 do near(st.fixed[i],config.fixed[i],1e-9)end
        local n=st.needle;assert(math.abs(n[9])<math.sin(math.rad(.5)),'needle tilt')
        local dp,da=0,0
        if previous then
            dp=math.sqrt((n[4]-previous[4])^2+(n[8]-previous[8])^2+(n[12]-previous[12])^2)
            local tr=0;for _,j in ipairs({1,2,3,5,6,7,9,10,11})do tr=tr+n[j]*previous[j]end
            da=math.acos(math.max(-1,math.min(1,(tr-1)/2)))
            assert(dp<.004 and da<math.rad(6),'needle discontinuity')
            assert(math.abs(sim.getSimulationTime()-rows[#rows].simulation_time-.05)<1e-8,'time did not advance')
        end
        if #action.target>0 then
            local t=st.tcps[action.arm];local v=action.target
            assert(math.sqrt((t[4]-v[4])^2+(t[8]-v[8])^2+(t[12]-v[12])^2)<.0001,'TCP tracking')
        end
        previous=n;st.frame=frame;st.phase=action.name;st.step=action.step;st.stitch=action.stitch;st.owner=action.owner
        st.thread=call('updateContinuousThread',action.stitch,action.theta,action.extracted)
        st.needle_step_m=dp;st.needle_step_rad=da;st.simulation_time=sim.getSimulationTime()
        rows[#rows+1]=st
        if recorder>=0 then sim.callScriptFunction('captureSutureFrame',recorder)end
        if frame==#config.actions then
            assert(st.thread.completed==#config.stitches,'missing thread arcs')
            if recorder>=0 then sim.callScriptFunction('finishSutureRecording',recorder)end
            done=true;sim.pauseSimulation()
        end
    end)
end
function getContinuousProgress()
    return {frame=frame,total=#config.actions,done=done,error=failure or '',stitch=rows[#rows] and rows[#rows].stitch or 0,phase=rows[#rows] and rows[#rows].phase or 'warmup'}
end
function getContinuousResults()return {rows=rows,transfers=transfers}end
