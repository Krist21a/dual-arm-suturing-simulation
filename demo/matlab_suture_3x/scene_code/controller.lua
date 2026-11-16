sim=require('sim')
audit={}
local function H(path)return sim.getObject(path)end
local function values(m)local a={} for i=1,#m do a[i]=string.format('%.12g',m[i])end return table.concat(a,',')end
local function collection(shapes)
    local c=sim.createCollection(1) -- override collidable/measurable properties
    for _,h in ipairs(shapes)do sim.addItemToCollection(c,sim.handle_single,h,0)end
    return c
end
function initSutureAudit()
    audit={}
    -- Unique aliases survive the two intentional parent changes and reload.
    audit.needle=H('/Needle')
    audit.tip=H('/Needle/needle_tip')
    audit.task=H('/customizableTable/SutureTask')
    audit.roots={H('/UR5'),H('/500____[1]/UR5')}
    audit.joints={};audit.tcps={};audit.wrists={};audit.collections={};audit.tools={};audit.baseTcpMatrices={};audit.yawPivots={}
    sim.setObjectParent(audit.needle,-1,true)
    for a,root in ipairs({'/UR5','/500____[1]/UR5'})do
        local joints={}
        for j=1,11 do joints[j]=H(root..(j<8 and '/' or '/L0_respondable_TOOL2/')..'joint'..j)end
        audit.joints[a]=joints
        for j=10,11 do sim.setJointPosition(joints[j],0)end
        -- Keep the original jaw axes, signs, opening logic and fixed TCP.
        sim.setJointInterval(joints[7],false,{0,.16})
        sim.setJointInterval(joints[9],false,{-math.rad(85),math.rad(170)})
        local tcp=H(root..'/L0_respondable_TOOL2/instrument_tcp_arm'..a)
        local wrist=sim.getObjectParent(tcp);audit.wrists[a]=wrist
        audit.baseTcpMatrices[a]=sim.getObjectMatrix(tcp,wrist)
        audit.yawPivots[a]=sim.getObjectPosition(joints[10],wrist)
        local grasp=sim.getObject('/suture_grasp_arm'..a,{noError=true})
        if grasp<0 then grasp=sim.createDummy(.001);sim.setObjectAlias(grasp,'suture_grasp_arm'..a);sim.setObjectParent(grasp,wrist,false);sim.setObjectInt32Param(grasp,sim.objintparam_visibility_layer,256)end
        audit.tcps[a]=grasp
        sim.setObjectMatrix(grasp,audit.baseTcpMatrices[a],wrist)
        local shapes=sim.getObjectsInTree(audit.roots[a],sim.sceneobject_shape,0)
        audit.collections[a]=collection(shapes)
        local toolShapes=sim.getObjectsInTree(H(root..'/L0_respondable_TOOL2'),sim.sceneobject_shape,0)
        audit.tools[a]=collection(toolShapes)
    end
    local env={H('/customizableTable/SutureTask/PadBase'),H('/customizableTable/SutureTask/TissueLeft'),H('/customizableTable/SutureTask/TissueRight')}
    audit.pad=collection(env)
    local tableShapes=sim.getObjectsInTree(H('/customizableTable'),sim.sceneobject_shape,0)
    local furniture={}
    for _,h in ipairs(tableShapes)do
        local alias=sim.getObjectAlias(h)
        if not alias:match('ring') and not alias:match('Tissue') and not alias:match('PadBase') and not alias:match('Wound') and sim.getObjectInt32Param(h,sim.objintparam_visibility_layer)~=256 then furniture[#furniture+1]=h end
    end
    audit.table=collection(furniture)
    audit.fixed={audit.task,H('/visionSensor_left'),H('/visionSensor_right')};audit.fixedPoses={}
    for i,h in ipairs(audit.fixed)do audit.fixedPoses[i]=sim.getObjectPose(h,-1)end
    audit.selfPairs={};audit.rails={}
    for a,root in ipairs({'/UR5','/500____[1]/UR5'})do
        local links={};local pairs={}
        for j=1,7 do links[j]=collection({H(root..'/link'..j..'_visible')})end
        for j=1,7 do for k=j+3,7 do pairs[#pairs+1]={links[j],links[k]}end end
        local module=collection(sim.getObjectsInTree(H(root..'/arm_ref'),sim.sceneobject_shape,0))
        for j=1,3 do pairs[#pairs+1]={links[j],module}end
        audit.selfPairs[a]=pairs;audit.rails[a]={}
        for _,name in ipairs({'left','right','front','rear'})do audit.rails[a][name]=H('/collision_a'..a..'_fixed_rail_'..name)end
    end
    audit.frame=0
    return true
end
function setSutureArm(a,q)
    for i=1,11 do sim.setJointPosition(audit.joints[a][i],q[i])end
    -- Original opposing jaw axes are unchanged. Equal raw commands keep
    -- the original opening; their differential adds common physical yaw.
    local yaw=(q[10]-q[11])/2;local c,s=math.cos(yaw),math.sin(yaw)
    local p=audit.yawPivots[a]
    local m={1,0,0,0,0,c,-s,p[2]-c*p[2]+s*p[3],0,s,c,p[3]-s*p[2]-c*p[3]}
    sim.setObjectMatrix(audit.tcps[a],sim.multiplyMatrices(m,audit.baseTcpMatrices[a]),audit.wrists[a])
    return sim.getObjectMatrix(audit.tcps[a],-1)
end
function setSutureNeedle(matrix,parent)
    sim.setObjectParent(audit.needle,-1,true)
    sim.setObjectMatrix(audit.needle,matrix,-1)
    if parent>0 then sim.setObjectParent(audit.needle,audit.tcps[parent],true)end
end
function handoffSutureNeedle(receiver)
    local before=sim.getObjectMatrix(audit.needle,-1)
    sim.setObjectParent(audit.needle,audit.tcps[receiver],true)
    local after=sim.getObjectMatrix(audit.needle,-1)
    return {before,after}
end
local function distance(a,b)
    local hit,pair=sim.checkCollision(a,b)
    return {distance=hit>0 and 0 or -1,collision=hit,pair=pair}
end
function auditSutureBatch(a,trajectory)
    local rows={}
    for _,q in ipairs(trajectory)do setSutureArm(a,q);rows[#rows+1]=getSutureAudit()end
    return rows
end
function getSutureAudit()
    local state={needle=sim.getObjectMatrix(audit.needle,-1),tip=sim.getObjectPosition(audit.tip,-1),joints={},tcps={},fixed={}}
    for a=1,2 do
        state.joints[a]={}
        for i,h in ipairs(audit.joints[a])do state.joints[a][i]=sim.getJointPosition(h)end
        state.tcps[a]=sim.getObjectMatrix(audit.tcps[a],-1)
    end
    state.posture={}
    for a,root in ipairs({'/UR5','/500____[1]/UR5'})do
        local result={collision=0,pair={-1,-1},distance=-1}
        for _,pair in ipairs(audit.selfPairs[a])do local hit=distance(pair[1],pair[2]);if hit.collision>0 then result=hit;break end end
        state[a==1 and 'left_self' or 'right_self']=result
        local p={};for name,h in pairs(audit.rails[a])do p[name]=sim.getObjectPosition(h,-1)end
        local width,axis={},{};local wn,an=0,0
        for i=1,3 do width[i]=p.right[i]-p.left[i];axis[i]=p.front[i]-p.rear[i];wn=wn+width[i]^2;an=an+axis[i]^2 end
        for i=1,3 do width[i]=width[i]/math.sqrt(wn);axis[i]=axis[i]/math.sqrt(an)end
        local front={width[2]*axis[3]-width[3]*axis[2],width[3]*axis[1]-width[1]*axis[3],width[1]*axis[2]-width[2]*axis[1]}
        state.posture[a]={elbow=sim.getObjectPosition(audit.joints[a][3],-1),shaft=axis,module_front=front}
    end
    state.arm_arm=distance(audit.collections[1],audit.collections[2])
    state.left_pad=distance(audit.collections[1],audit.pad);state.right_pad=distance(audit.collections[2],audit.pad)
    state.left_table=distance(audit.collections[1],audit.table);state.right_table=distance(audit.collections[2],audit.table)
    for _,h in ipairs(audit.fixed)do state.fixed[#state.fixed+1]=sim.getObjectPose(h,-1)end
    return state
end
function captureAuditImage(prefix)
    for _,side in ipairs({'left','right'})do
        local sensor=H('/visionSensor_'..side)
        sim.setExplicitHandling(sensor,1);sim.handleVisionSensor(sensor)
        local img,res=sim.getVisionSensorImg(sensor)
        sim.saveImage(img,res,0,prefix..'_'..side..'.png',-1)
    end
end

function updateSutureThread(theta,extracted)
    if not audit.thread then audit.thread=sim.addDrawingObject(sim.drawing_lines+sim.drawing_painttag,2,0,-1,160,{.12,.4,.8})end
    sim.addDrawingObjectItem(audit.thread,nil)
    local cl={0,.005854474479272,-.000125944725770}
    local r=.011375;local a=math.atan2(-.0115-cl[3],.006-cl[2])-math.pi
    local tail=sim.multiplyVector(sim.getObjectMatrix(audit.needle,-1),{0,cl[2]+r*math.cos(a),cl[3]+r*math.sin(a)})
    local entry=sim.getObjectPosition(H('/customizableTable/SutureTask/EntryPoint'),-1)
    local exit=sim.getObjectPosition(H('/customizableTable/SutureTask/ExitPoint'),-1)
    local points={{entry[1]+.026,entry[2]+.018,entry[3]+.0003},{entry[1]+.018,entry[2]+.024,entry[3]+.0003}}
    if theta<math.pi and not extracted then
        -- A slack trailing strand outside the tissue, following the needle tail.
        points[#points+1]={tail[1]+.015,tail[2]+.012,math.max(entry[3],tail[3]-.008)}
    else
        points[#points+1]=entry
        local sweep=2*math.asin(.008/r)
        local passed=extracted and sweep or math.min(sweep,theta-math.pi)
        local center=sim.getObjectPosition(H('/customizableTable/SutureTask/NeedleArcCenter'),-1)
        local u,v={},{};for i=1,3 do u[i]=entry[i]-center[i];v[i]=exit[i]-center[i]end
        local n={u[2]*v[3]-u[3]*v[2],u[3]*v[1]-u[1]*v[3],u[1]*v[2]-u[2]*v[1]}
        local len=math.sqrt(n[1]^2+n[2]^2+n[3]^2);for i=1,3 do n[i]=n[i]/len end
        local cross={n[2]*u[3]-n[3]*u[2],n[3]*u[1]-n[1]*u[3],n[1]*u[2]-n[2]*u[1]}
        for k=1,24 do
            local angle=k/24*passed;local point={}
            for i=1,3 do point[i]=center[i]+math.cos(angle)*u[i]+math.sin(angle)*cross[i]end
            points[#points+1]=point
        end
        if passed>=sweep then points[#points+1]={(exit[1]+tail[1])/2,exit[2],math.max(entry[3]+.001,(exit[3]+tail[3])/2-.004)}end
    end
    points[#points+1]=tail
    for k=2,#points do local a,b=points[k-1],points[k];sim.addDrawingObjectItem(audit.thread,{a[1],a[2],a[3],b[1],b[2],b[3]})end
    return true
end

-- One connected kinematic strand. Completed tissue arcs persist across stitches.
local acceptedSetSutureArm=setSutureArm
function setSutureArm(a,q)
    -- The accepted mapper is unchanged. Keep dormant kinematic targets from
    -- pulling the proximal joints back toward their old positions on a step.
    local result=acceptedSetSutureArm(a,q)
    for i=1,11 do sim.setJointTargetPosition(audit.joints[a][i],q[i])end
    return result
end
function initContinuousThread(stitches)
    audit.stitches=stitches;audit.completed=0
    audit.thread=sim.addDrawingObject(sim.drawing_lines+sim.drawing_painttag,2,0,-1,2048,{.12,.4,.8})
end
function updateContinuousThread(index,theta,extracted)
    local stitches=audit.stitches
    if extracted then audit.completed=math.max(audit.completed,index) end
    local entry=stitches[1].entryPoint
    local points={{entry[1]+.026,entry[2]+.018,entry[3]+.0003},{entry[1]+.018,entry[2]+.024,entry[3]+.0003}}
    local function append(p)points[#points+1]=p end
    local function arc(st,angle)
        local e,c=st.entryPoint,st.arcCenter;append(e)
        local ux,uz=e[1]-c[1],e[3]-c[3]
        for k=1,24 do
            local t=angle*k/24
            append({c[1]+math.cos(t)*ux+math.sin(t)*uz,c[2],c[3]-math.sin(t)*ux+math.cos(t)*uz})
        end
    end
    local sweep=2*math.asin(.008/.011375)
    for j=1,audit.completed do
        if j>1 then
            local a,b=stitches[j-1].exitPoint,stitches[j].entryPoint
            append({a[1],a[2],a[3]+.0004});append({b[1],b[2],b[3]+.0004})
        end
        arc(stitches[j],sweep)
    end
    if index>audit.completed and theta>math.pi then
        local st=stitches[index]
        if index>1 then append({st.entryPoint[1],st.entryPoint[2],st.entryPoint[3]+.0004})end
        arc(st,math.min(sweep,theta-math.pi))
    end
    local cl={0,.005854474479272,-.000125944725770}
    local r=.011375;local a=math.atan2(-.0115-cl[3],.006-cl[2])-math.pi
    local tail=sim.multiplyVector(sim.getObjectMatrix(audit.needle,-1),{0,cl[2]+r*math.cos(a),cl[3]+r*math.sin(a)})
    local last=points[#points]
    append({(last[1]+tail[1])/2,(last[2]+tail[2])/2,math.max(last[3]+.001,tail[3]-.008)})
    append(tail)
    sim.addDrawingObjectItem(audit.thread,nil)
    for k=2,#points do
        local a,b=points[k-1],points[k]
        sim.addDrawingObjectItem(audit.thread,{a[1],a[2],a[3],b[1],b[2],b[3]})
    end
    return {completed=audit.completed,vertices=#points}
end

function sysCall_init() end
