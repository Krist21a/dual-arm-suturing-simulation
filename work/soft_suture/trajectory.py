"""Reuse the accepted dual-arm plan and add a separately checked pull motion."""
import sys,json
from pathlib import Path
import numpy as np
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parents[2]

from kinematics import fk

def build_actions(p):
    q=[np.array(p['initial'][a]) for a in ['L','R']]
    grip=[.35,0.];owner=1;stitch=1;theta=0.;actions=[]
    cal=[np.array(p['calibration'][a]) for a in ['L','R']]
    relation=np.array(p['relations']['R'])
    def append(name,arm=0,receiver=0):
        nonlocal owner,relation
        tcp=[cal[i]@fk(q[i],['L','R'][i]) for i in range(2)]
        needle=tcp[owner]@relation
        if receiver:
            owner=receiver-1;relation=np.linalg.inv(tcp[owner])@needle
        actual=[(qq+np.r_[np.zeros(9),[grip[i]]*2]).tolist() for i,qq in enumerate(q)]
        actions.append(dict(phase=name,stitch=stitch,theta=theta,q=actual,
                            needle=needle[:3].ravel().tolist(),owner=owner+1))
    def jaws(a,g,name):
        for val in np.linspace(grip[a],g,12):grip[a]=float(val);append(name,a+1)
    append('initial')
    for ph in p['phases']:
        stitch=ph['stitch'];a=0 if ph['arm']=='L' else 1
        for k,qq in enumerate(ph['q']):
            q[a]=np.array(qq)
            if ph['needleAngles']:theta=ph['needleAngles'][k]
            append(ph['name'],a+1)
        if ph['name']=='left_grasp':
            jaws(0,0,'left_close');append('handoff',receiver=1);jaws(1,.35,'right_open')
        elif ph['name']=='right_regrasp':
            jaws(1,0,'right_close');append('handoff',receiver=2);jaws(0,.35,'left_open')
    # Keep orientation; pull away from the tissue using the actual right arm.
    start=cal[1]@fk(q[1],'R');base=np.array(p['initial']['R'])
    lower=base[:9]-np.r_[np.full(6,np.radians(5)),.3,7,4]
    upper=base[:9]+np.r_[np.full(6,np.radians(5)),.3,7,4]
    lower[6:]=[0,-np.pi,-np.radians(85)];upper[6:]=[.16,np.pi,np.radians(85)]
    initial=q[1].copy();maxerr=0.
    # Follow the reverse insertion axis first; use redundant proximal joints
    # within the existing +/-5 degree limit if a pure prismatic pull is short.
    direction=np.array([0.,0.,1.])
    for k in range(1,121):
        target=start.copy();target[:3,3]+=.100*(k/120)*direction
        previous=q[1].copy()
        def residual(v):
            qq=np.r_[v,initial[9:]];T=cal[1]@fk(qq,'R')
            return np.r_[(T[:3,3]-target[:3,3])*100,
                         (T[:3,:3]-target[:3,:3]).ravel(),(v-previous[:9])*.0001]
        opt=least_squares(residual,previous[:9],bounds=(lower,upper),
                          max_nfev=180,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        q[1]=np.r_[opt.x,initial[9:]]
        T=cal[1]@fk(q[1],'R');err=np.linalg.norm(T[:3,3]-target[:3,3])
        if err>.0001:raise RuntimeError(f'Pull IK target unreachable: {k}, {err} m')
        maxerr=max(maxerr,err);append('tighten',2)
    for _ in range(60):append('hold_tension')
    return actions,dict(pull_distance_mm=100.,pull_tcp_max_error_mm=maxerr*1000)

if __name__=='__main__':
    p=json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text())
    actions,report=build_actions(p)
    (Path(__file__).parent/'actions.json').write_text(json.dumps(actions))
    print(len(actions),report)
