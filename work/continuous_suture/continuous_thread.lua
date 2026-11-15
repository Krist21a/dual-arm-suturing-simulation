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
