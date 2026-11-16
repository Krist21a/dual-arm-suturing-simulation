"""Local online mechanics service. Receives commands, never reads result frames."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import argparse,json,socket,traceback
import numpy as np
from mechanics import SoftSuture,Parameters
from natural_strand import strand

class Solver:
 def __init__(self,stiffness=35.):
  self.model=SoftSuture(Parameters(dt=1/120,spring_stiffness=stiffness,strain_stiffening=4.,edge_viscosity=.15,distributed_contact=True))
  self.previous_tip=None;self.previous_tail=None;self.deployed=set();self.frames=0
 def step(self,a):
  m=self.model;T=np.array(a['needle']).reshape(3,4);tip=T[:,:3]@np.array([0,.006,-.0115])+T[:,3];tail=T[:,:3]@np.array([0,.005708948958544,.01124811054846])+T[:,3]
  if not m.route:m.start_strand(tail)
  if a['phase']=='left_handoff' and a['stitch'] not in self.deployed:
   assert a['stitch'] in m.punctured,'Needle has not punctured; refusing to install thread guide.'
   m.deploy(a['stitch'],tail);self.deployed.add(a['stitch'])
  for k in range(1,7):
   f=k/6;t=tip if self.previous_tip is None else self.previous_tip*(1-f)+tip*f;e=tail if self.previous_tail is None else self.previous_tail*(1-f)+tail*f
   metrics=m.step(tip=t,puncture_key=a['stitch'],cable_end=e,feed=a['phase'] not in ['tighten','hold_tension'])
  self.previous_tip=tip;self.previous_tail=tail;self.frames+=1
  line=strand(m,m.display_cable(a['stitch'],a['theta'],a['phase'],len(self.deployed)))
  z=m.x[m.tets];J=np.einsum('ij,ij->i',z[:,1]-z[:,0],np.cross(z[:,2]-z[:,0],z[:,3]-z[:,0]))/6/m.volume0
  assert np.isfinite(m.x).all() and (J>0).all()
  return dict(ok=True,geometry=np.r_[m.x[:m.n_tissue].ravel(),len(line),line.ravel()].tolist(),metrics=metrics,minimum_volume_ratio=float(J.min()),completed_stitches=len(self.deployed),frame=self.frames)

def main():
 p=argparse.ArgumentParser();p.add_argument('--port',type=int,required=True);p.add_argument('--stiffness',type=float,default=35.);args=p.parse_args();solver=Solver(args.stiffness)
 with socket.socket() as server:
  server.bind(('127.0.0.1',args.port));server.listen(1);server.settimeout(120)
  conn,_=server.accept()
  with conn,conn.makefile('rwb') as stream:
   while True:
    raw=stream.readline()
    if not raw:break
    try:
     request=json.loads(raw)
     if request.get('stop'):break
     result=solver.step(request)
    except Exception as exc:result=dict(ok=False,error=str(exc),traceback=traceback.format_exc())
    stream.write((json.dumps(result,allow_nan=False)+'\n').encode());stream.flush()
if __name__=='__main__':main()
