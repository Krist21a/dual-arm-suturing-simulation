import os,json,argparse
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from kinematics import fk
from retarget_grasp import calibration
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
ROOT=Path(__file__).resolve().parents[2];D=ROOT/'output/soft_suture_continuous'
ap=argparse.ArgumentParser();ap.add_argument('--arm',default='R');ap.add_argument('--axis',default='X');ap.add_argument('--flip',action='store_true');ap.add_argument('--grip-search',action='store_true');ap.add_argument('--yaw-search',action='store_true');ap.add_argument('--pitch-search',action='store_true');args=ap.parse_args();arm=args.arm;offset=0 if arm=='L' else 11
p=json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text())
m=json.loads((D/'manifest.json').read_text());frames=np.fromfile(D/'frames.bin',dtype='<f4').reshape(-1,m['state_width'])
s=RemoteAPIClient(port=23002).require('sim');driver=s.getObject('/SoftSuturePlayback');controller=s.getObject('/customizableTable/SutureTask/SutureController')
s.callScriptFunction('setLiveMode',driver,True);s.setStepping(True);s.startSimulation()
base=np.array(p['initial'][arm]);cal=np.array(p['calibration'][arm])
span=25 if args.yaw_search or args.pitch_search else 5
lo=np.r_[base[:6]-np.radians(span),0,-np.pi,-np.radians(85),-np.radians(85)]
hi=np.r_[base[:6]+np.radians(span),.16,np.pi,np.radians(85),np.radians(85)]
results=[]
for degrees in ([-75,-60,-45,-30,0,30] if args.pitch_search else [-60,-30,0,30,60] if args.yaw_search else [8,10,12] if args.grip_search else [0,-15,15,-30,30,-45,45]):
 a=np.radians(degrees);R=np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
 if args.axis=='Y':R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]])
 if args.flip:R=np.diag([-1,-1,1])@R
 g=np.array([0,0,-.003]);S=np.eye(4);S[:3,:3]=R;S[:3,3]=g-R@g
 if args.grip_search:
  candidate=calibration(p,degrees)['L'];S=np.linalg.inv(np.array(m['grasp_correction']['calibration']['L']['transform']))@np.array(candidate['transform'])
 if args.yaw_search or args.pitch_search:S=np.eye(4)
 hits=[];errors=[]
 for i in [180,220,252,270,280,300,320,350,425,527,600,920,960,990]:
  row=frames[i].copy();q=row[offset:offset+11].astype(float);T=cal@fk(q,arm)@S;seed=np.r_[q[:9],(q[9]-q[10])/2]
  def residual(v):
   W=cal@fk(np.r_[v[:9],v[9],-v[9]],arm)
   extra=[.2*(v[9]-np.radians(degrees))] if args.yaw_search else []
   if args.pitch_search:extra=[.2*(v[8]-np.radians(degrees))]
   return np.r_[100*(W[:3,3]-T[:3,3]),(W[:3,:3]-T[:3,:3]).ravel(),.00001*(v-seed),extra]
  fit=least_squares(residual,np.clip(seed,lo+1e-9,hi-1e-9),bounds=(lo,hi),max_nfev=180,ftol=1e-10,gtol=1e-10,xtol=1e-10);v=fit.x
  error=np.max(abs(residual(v)[:12]));errors.append(float(error))
  grip=(q[9]+q[10])/2
  if args.grip_search:
   old=m['grasp_correction']['calibration']['L']['closed_grip_rad'];opening=np.clip((grip-old)/(.35-old),0,1);grip=candidate['closed_grip_rad']*(1-opening)+.35*opening
  row[offset:offset+11]=np.r_[v[:9],v[9]+grip,-v[9]+grip]
  s.callScriptFunction('applyLiveState',driver,row.astype(float).tolist());s.step();state=s.callScriptFunction('getSutureAudit',controller)
  assert np.max(abs(np.array(state['joints']).ravel()-row[:22]))<1e-6,'Stale joint positions'
  prefix='left' if arm=='L' else 'right'
  for key in ['arm_arm',prefix+'_pad',prefix+'_table',prefix+'_self']:
   if state[key]['collision']:hits.append([i+1,key,state[key]['pair']])
 result=dict(tilt=degrees,hits=hits,max_error=max(errors));results.append(result);print(result,flush=True)
s.pauseSimulation();s.setStepping(False)
(D/('tilt_search_'+arm+'.json')).write_text(json.dumps(results,indent=2))
