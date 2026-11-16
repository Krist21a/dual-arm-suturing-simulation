-- Geometry is supplied by the online solver under MATLAB control.
-- No trajectory, precomputed deformations or prerecorded frames are loaded.
sim=require('sim')
local tissue,line,controller
function sysCall_init()
 controller=sim.getObject('/customizableTable/SutureTask/SutureController')
 sim.callScriptFunction('initSutureAudit',controller)
 for _,name in ipairs({'TissueLeft','TissueRight','WoundVisual','SoftTissuePreview'})do
  local h=sim.getObject('/customizableTable/SutureTask/'..name,{noError=true})
  if h>=0 then sim.setObjectInt32Param(h,sim.objintparam_visibility_layer,0)end
 end
 tissue=sim.addDrawingObject(sim.drawing_triangles+sim.drawing_painttag,1,0,-1,#config.faces,{.93,.55,.55})
 line=sim.addDrawingObject(sim.drawing_lines+sim.drawing_painttag,2,0,-1,2048,{.08,.25,.65})
end
function applySoftGeometry(v)
 local triangles={}
 for _,face in ipairs(config.faces)do for _,vertex in ipairs(face)do
  for j=1,3 do triangles[#triangles+1]=v[vertex*3+j]end
 end end
 sim.addDrawingObjectItem(tissue,nil);sim.addDrawingObjectItem(tissue+sim.handleflag_addmultiple,triangles)
 local base=config.nodes*3+1;local count=v[base];local segments={}
 assert(count>=2 and count<=512,'Invalid continuous strand')
 for i=1,count-1 do
  for j=1,3 do segments[#segments+1]=v[base+(i-1)*3+j]end
  for j=1,3 do segments[#segments+1]=v[base+i*3+j]end
 end
 sim.addDrawingObjectItem(line,nil);sim.addDrawingObjectItem(line+sim.handleflag_addmultiple,segments)
 return count
end
function sysCall_actuation()end
