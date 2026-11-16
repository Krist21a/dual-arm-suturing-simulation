"""Quasi-static gravity sag of slack spans, with a supporting pad surface.

The tension solver remains a global massless cable. These sampled centerlines
resolve its otherwise undetermined slack shape; they are not a dynamic rod.
"""
import numpy as np
from scipy.interpolate import LinearNDInterpolator

def strand(model,points):
    lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
    slack=max(0.,model.cable_rest-model.cable_length())
    surfaces=[]
    for side in range(2):
        top=model.x[model.ids[side,:,:,-1].ravel()]
        surfaces.append(LinearNDInterpolator(top[:,:2],top[:,2]+.00018,fill_value=np.nan))
    output=[points[0]]
    for i,(a,b,L) in enumerate(zip(points[:-1],points[1:],lengths)):
        if L<1e-7:continue
        t=np.linspace(0,1,13);base=a[None,:]*(1-t[:,None])+b[None,:]*t[:,None]
        target=L+slack*L/max(lengths.sum(),1e-9)
        tangent=(b-a)/L;down=np.array([0.,0.,-1.]);direction=down-tangent*np.dot(down,tangent)
        if np.linalg.norm(direction)<.1:direction=np.cross(tangent,[1.,0.,0.])
        direction/=np.linalg.norm(direction)
        def curve(h):
            x=base+np.sin(np.pi*t)[:,None]*h*direction
            for surface in surfaces:
                z=surface(x[:,0],x[:,1]);valid=np.isfinite(z)
                # Interior samples contact the pad; guide/needle endpoints stay exact.
                valid[[0,-1]]=False;x[valid,2]=np.maximum(x[valid,2],z[valid])
            return x
        low=0.;high=min(.02,max(.001,np.sqrt(max(0.,target*target-L*L))))
        for _ in range(18):
            mid=(low+high)/2;x=curve(mid)
            if np.linalg.norm(np.diff(x,axis=0),axis=1).sum()<target:low=mid
            else:high=mid
        output.extend(curve((low+high)/2)[1:])
    return np.array(output)
