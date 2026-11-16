"""Persist correct initial joints, needle and visible tissue/thread meshes."""
import numpy as np
from physics_worker import Solver

def bake(s,first,faces):
 for arm,root in enumerate(['/UR5','/500____[1]/UR5']):
  for j,q in enumerate(first['q'][arm],1):
   path=root+('/' if j<8 else '/L0_respondable_TOOL2/')+'joint'+str(j)
   h=s.getObject(path);s.setJointPosition(h,q);s.setJointTargetPosition(h,q)
 needle=s.getObject('/Needle');s.setObjectParent(needle,-1,True);s.setObjectMatrix(needle,first['needle'],-1)
 geometry=np.array(Solver().step(first)['geometry']);nodes=540
 tissue=geometry[:nodes*3];count=int(geometry[nodes*3]);line=geometry[nodes*3+1:].reshape(count,3)
 vertices=[];triangles=[];radius=.00018;sides=8
 for i,p in enumerate(line):
  tangent=line[min(i+1,count-1)]-line[max(0,i-1)];tangent/=np.linalg.norm(tangent)
  normal=np.cross(tangent,[1.,0.,0.])
  if np.linalg.norm(normal)<.1:normal=np.cross(tangent,[0.,1.,0.])
  normal/=np.linalg.norm(normal);other=np.cross(tangent,normal)
  for k in range(sides):vertices.append(p+radius*(np.cos(2*np.pi*k/sides)*normal+np.sin(2*np.pi*k/sides)*other))
  if i:
   for k in range(sides):
    a=(i-1)*sides+k;b=(i-1)*sides+(k+1)%sides;c=i*sides+k;d=i*sides+(k+1)%sides
    triangles.extend([[a,b,c],[b,d,c]])
 task=s.getObject('/customizableTable/SutureTask')
 for name,v,f,color in [('SoftTissuePreview',tissue,faces,[.93,.55,.55]),('TissueThreadPreview',np.array(vertices).ravel(),triangles,[.08,.25,.65])]:
  old=s.getObject('/customizableTable/SutureTask/'+name,dict(noError=True))
  if old>=0:s.removeObjects([old])
  h=s.createShape(0,.35,np.asarray(v,dtype=float).ravel().tolist(),np.asarray(f,dtype=int).ravel().tolist())
  s.setObjectAlias(h,name);s.setObjectParent(h,task,True);s.setShapeColor(h,None,s.colorcomponent_ambient_diffuse,color)
  s.setObjectInt32Param(h,s.shapeintparam_static,1);s.setObjectInt32Param(h,s.shapeintparam_respondable,0);s.setObjectInt32Param(h,s.objintparam_visibility_layer,1)
 return count
