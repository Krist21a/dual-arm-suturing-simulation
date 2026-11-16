"""Place the needle body between the jaw faces, 3 mm behind their tips.

Needle/tissue trajectories stay fixed. Re-solve both arms for a transverse
needle bite and a mesh-derived jaw gap, including the handover trajectories.
"""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import json,itertools,time,hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
from kinematics import fk
ROOT=Path(__file__).resolve().parents[2]

def calibration(plan,left_grip_degrees=15,bite_depth=.003):
    scene=json.loads((ROOT/'work/suture_review/scene_live.json').read_text())
    needle=next(o for o in scene if o['alias'].endswith('/Needle'))
    mesh=np.array(needle['mesh'][0]).reshape(-1,3);triangles=np.array(needle['mesh'][1]).reshape(-1,3)
    center=np.array([0,.005854474479272,-.000125944725770]);tip=np.array([0,.006,-.0115])
    result={}
    for key in ['L','R']:
        relation=np.array(plan['relations'][key]);g=np.linalg.inv(relation)[:3,3]
        if key=='L':
            angle=np.arctan2(*(tip-center)[[2,1]])-np.radians(left_grip_degrees)
            g=center+.011375*np.array([0,np.cos(angle),np.sin(angle)])
        tangent=relation[:3,:3]@np.cross([1,0,0],g-center)
        angle=(np.arctan2(tangent[1],tangent[0])+np.pi/2)%np.pi-np.pi/2
        c,s=np.cos(angle),np.sin(angle);R=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        grasp=relation[:3,:3]@g+relation[:3,3]
        translation=grasp+np.array([0,0,bite_depth])
        V=(mesh@relation[:3,:3].T+relation[:3,3]-translation)@R
        section=[]
        for tri in V[triangles]:
            for a,b in [(tri[0],tri[1]),(tri[1],tri[2]),(tri[2],tri[0])]:
                if a[0]*b[0]<0:
                    x=a+(b-a)*(-a[0])/(b[0]-a[0])
                    if abs(x[2]+bite_depth)<.002 and abs(x[1])<.002:section.append(x)
        section=np.array(section);lo=section.min(axis=0);hi=section.max(axis=0)
        center_offset=(lo+hi)/2;center_offset[0]=0;center_offset[2]+=bite_depth
        translation+=R@center_offset
        width=hi[1]-lo[1]
        # Closed mesh inner faces at depth 3 mm: +/-0.244 mm; pivot is
        # 9.385 mm behind TCP. Positive common grip opens both jaw halves.
        closed=(width-.000488)/(2*.006385)
        S=np.eye(4);S[:3,:3]=R;S[:3,3]=translation
        result[key]=dict(transform=S.tolist(),needle_grasp_local=g.tolist(),
            closed_grip_rad=float(closed),body_width_mm=float(width*1000),
            tool_roll_change_deg=float(np.degrees(angle)),bite_depth_mm=bite_depth*1000)
    return result

def retarget(data):
    p=json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text())
    config=calibration(p);manifest=json.loads((data/'manifest.json').read_text())
    if 'grasp_correction' in manifest:
        raise RuntimeError('This dataset already has centered grasps. Regenerate it with simulate.py before retargeting again.')
    frames=np.fromfile(data/'frames.bin',dtype='<f4').reshape(-1,manifest['state_width']).copy()
    original=frames.copy();previous={};maxpos=0.;maxrot=0.;maxstep=0.;step_peak=None;cache={}
    start=time.time()
    for i,row in enumerate(frames):
        for a,key in enumerate(['L','R']):
            q=original[i,a*11:a*11+11].astype(float);base=np.array(p['initial'][key])
            oldpose=np.array(p['calibration'][key])@fk(q,key)
            target=oldpose@np.array(config[key]['transform'])
            lo=np.r_[base[:6]-np.radians(5),0,-np.pi,-np.radians(85),-np.radians(85)]
            hi=np.r_[base[:6]+np.radians(5),.16,np.pi,np.radians(85),np.radians(85)]
            seed=previous.get(key,np.r_[q[:9],(q[9]-q[10])/2])
            seed=np.clip(seed,lo+1e-9,hi-1e-9)
            cal=np.array(p['calibration'][key])
            def residual(v):
                qq=np.r_[v[:9],v[9],-v[9]];T=cal@fk(qq,key)
                return np.r_[100*(T[:3,3]-target[:3,3]),(T[:3,:3]-target[:3,:3]).ravel(),.00001*(v-seed)]
            cache_key=(key,tuple(np.round(np.r_[q[:9],(q[9]-q[10])/2],6)))
            if cache_key in cache:v=cache[cache_key].copy()
            else:
                fit=least_squares(residual,seed,bounds=(lo,hi),max_nfev=160,
                    ftol=1e-11,xtol=1e-11,gtol=1e-11)
                v=fit.x;cache[cache_key]=v.copy()
            T=cal@fk(np.r_[v[:9],v[9],-v[9]],key)
            pe=np.linalg.norm(T[:3,3]-target[:3,3]);re=np.max(abs(T[:3,:3]-target[:3,:3]))
            if pe>.00003 or re>.0003:
                raise RuntimeError(f'Grasp IK failed frame={i+1} arm={key} position={pe} rotation={re}')
            if key in previous:
                step=float(np.max(abs(v[np.r_[0:6,7:10]]-previous[key][np.r_[0:6,7:10]])))
                if step>maxstep:maxstep=step;step_peak=dict(frame=i+1,arm=key)
            previous[key]=v
            opening=float(np.clip((q[9]+q[10])/2/.35,0,1))
            grip=config[key]['closed_grip_rad']*(1-opening)+.35*opening
            row[a*11:a*11+11]=np.r_[v[:9],v[9]+grip,-v[9]+grip]
            maxpos=max(maxpos,float(pe));maxrot=max(maxrot,float(re))
        if i%200==0:print('grasp',i+1,len(frames),'elapsed',round(time.time()-start),flush=True)
    assert np.array_equal(frames[:,22:],original[:,22:]),'Needle or tissue was moved by grasp correction'
    frames.tofile(data/'grasp_candidate.bin')
    # This checks branch flips, not needle angular speed. Wrist coordinates
    # can move faster than the needle near a pitch singularity.
    assert maxstep<np.radians(20),('Joint branch discontinuity',maxstep,step_peak)
    original.tofile(data/'frames_before_grasp.bin');frames.tofile(data/'frames.bin')
    report=dict(result='PASS',frames=len(frames),calibration=config,
        maximum_tcp_position_error_mm=maxpos*1000,maximum_tcp_rotation_matrix_error=maxrot,
        maximum_joint_step_deg=float(np.degrees(maxstep)),maximum_joint_step_location=step_peak,
        needle_and_tissue_unchanged=True)
    (data/'grasp_validation.json').write_text(json.dumps(report,indent=2))
    (ROOT/'work/soft_suture/grasp_calibration.json').write_text(json.dumps(config,indent=2))
    manifest['grasp_correction']=report
    (data/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(report),flush=True)

if __name__=='__main__':retarget(ROOT/'output/soft_suture_continuous')
