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
    audit.joints={};audit.tcps={};audit.wrists={};audit.collections={};audit.tools={}
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
        audit.tcps[a]=tcp;audit.wrists[a]=sim.getObjectParent(tcp)
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
    audit.frame=0
    return true
end
function setSutureArm(a,q)
    for i=1,11 do sim.setJointPosition(audit.joints[a][i],q[i])end
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
        local cz=entry[3]+math.sqrt(r*r-.008*.008)
        for k=1,24 do
            local sign=entry[1]>exit[1] and 1 or -1
            local angle=sign*k/24*passed;local x,z=entry[1]+.03,entry[3]-cz
            points[#points+1]={-.03+math.cos(angle)*x+math.sin(angle)*z,entry[2],cz-math.sin(angle)*x+math.cos(angle)*z}
        end
        if passed>=sweep then points[#points+1]={(exit[1]+tail[1])/2,exit[2],math.max(entry[3]+.001,(exit[3]+tail[3])/2-.004)}end
    end
    points[#points+1]=tail
    for k=2,#points do local a,b=points[k-1],points[k];sim.addDrawingObjectItem(audit.thread,{a[1],a[2],a[3],b[1],b[2],b[3]})end
    return true
end
