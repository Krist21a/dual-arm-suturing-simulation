"""Tilt the right holder around its needle bite to clear the pad."""
import os,json,time,argparse
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from kinematics import fk
ROOT=Path(__file__).resolve().parents[2];D=ROOT/'output/soft_suture_continuous'
ap=argparse.ArgumentParser();ap.add_argument('--left',action='store_true');args=ap.parse_args();arm='L' if args.left else 'R';offset=0 if args.left else 11;report_key='left_holder_clearance' if args.left else 'holder_clearance'
p=json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text())
m=json.loads((D/'manifest.json').read_text());assert 'grasp_correction' in m and report_key not in m
frames=np.fromfile(D/'frames.bin',dtype='<f4').reshape(-1,m['state_width']).copy();original=frames.copy()
base=np.array(p['initial'][arm]);cal=np.array(p['calibration'][arm])
lo=np.r_[base[:6]-np.radians(5),0,-np.pi,-np.radians(85),-np.radians(85)]
hi=np.r_[base[:6]+np.radians(5),.16,np.pi,np.radians(85),np.radians(85)]
degrees=15 if args.left else 30;a=np.radians(degrees)
R=np.array([[np.cos(a),0,np.sin(a)],[0,1,0],[-np.sin(a),0,np.cos(a)]]) if args.left else np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
g=np.array([0,0,-.003]);S=np.eye(4);S[:3,:3]=R;S[:3,3]=g-R@g
previous=None;previous_key=None;cache={};maximum=0.;maxstep=0.;peak=None;started=time.time()
for i,row in enumerate(frames):
 q=original[i,offset:offset+11].astype(float);target=cal@fk(q,arm)@S
 seed=np.r_[q[:9],(q[9]-q[10])/2] if previous is None else previous
 def residual(v):
  T=cal@fk(np.r_[v[:9],v[9],-v[9]],arm)
  return np.r_[100*(T[:3,3]-target[:3,3]),(T[:3,:3]-target[:3,:3]).ravel(),(.003 if args.left else .00001)*(v-seed)]
 key=tuple(np.round(np.r_[q[:9],(q[9]-q[10])/2],6))
 if args.left and key==previous_key:v=previous.copy()
 elif not args.left and key in cache:v=cache[key].copy()
 else:
  fit=least_squares(residual,np.clip(seed,lo+1e-9,hi-1e-9),bounds=(lo,hi),max_nfev=180,ftol=1e-11,gtol=1e-11,xtol=1e-11);v=fit.x;cache[key]=v.copy()
 error=float(np.max(abs(residual(v)[:12])));assert error<.0003,(i,error);maximum=max(maximum,error)
 if previous is not None:
  step=float(np.max(abs(v[np.r_[0:6,7:10]]-previous[np.r_[0:6,7:10]])))
  if step>maxstep:maxstep=step;peak=i+1
 previous=v;previous_key=key;grip=(q[9]+q[10])/2;row[offset:offset+11]=np.r_[v[:9],v[9]+grip,-v[9]+grip]
 if i%400==0:print('clearance',i+1,len(frames),'elapsed',round(time.time()-started),flush=True)
frames.tofile(D/('clearance_candidate_'+arm+'.bin'))
assert maxstep<np.radians(20),(np.degrees(maxstep),peak)
other=11-offset
assert np.array_equal(frames[:,22:],original[:,22:]) and np.array_equal(frames[:,other:other+11],original[:,other:other+11])
original.tofile(D/('frames_before_clearance_'+arm+'.bin'));frames.tofile(D/'frames.bin')
c=m['grasp_correction']['calibration'][arm];c['transform']=(np.array(c['transform'])@S).tolist();c['bite_tilt_deg']=degrees
report=dict(result='PASS',arm=arm,tilt_deg=degrees,maximum_weighted_pose_error=maximum,maximum_joint_step_deg=float(np.degrees(maxstep)),needle_and_tissue_unchanged=True)
m[report_key]=report;(D/'manifest.json').write_text(json.dumps(m,indent=2));(D/('clearance_validation_'+arm+'.json')).write_text(json.dumps(report,indent=2));print(report,flush=True)
if not args.left:(D/'clearance_validation.json').write_text(json.dumps(report,indent=2))
(ROOT/'work/soft_suture/grasp_calibration.json').write_text(json.dumps(m['grasp_correction']['calibration'],indent=2))
