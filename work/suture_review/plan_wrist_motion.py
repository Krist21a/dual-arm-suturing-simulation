"""Constrained task planner; retains the existing FK and jaw-only q10/q11."""
import sys,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'work/python_deps'),str(ROOT/'work/scene_inspection')]
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from validate_suture_trajectory import fk as fk_legacy,pose4,ry,interpolate
data=json.loads((ROOT/'work/suture_review/scene_live.json').read_text())
def obj(alias):return next(o for o in data if o['alias']==alias)
roots={'L':'/UR5','R':'/500____[1]/UR5'}
q0={a:np.array([obj(roots[a]+('/' if j<8 else '/L0_respondable_TOOL2/')+'joint'+str(j))['q'] for j in range(1,12)]) for a in roots}
cal={a:pose4(*next(o for o in data if o['alias'].endswith('/instrument_tcp_arm'+str(i)))['pose'])@np.linalg.inv(fk_legacy(q0[a],a)) for i,a in enumerate(['L','R'],1)}
def fk(q,arm):
    return fk_legacy(q,arm)
cl=np.array([0,.005854474479272,-.000125944725770]);tip=np.array([0,.006,-.0115]);rad=tip-cl;r=np.linalg.norm(rad)
center=np.array([-.03,.345,.5905+np.sqrt(r*r-.008**2)])
entryPoint=np.array([-.022,.345,.5905]);exitPoint=np.array([-.038,.345,.5905])
lr=rad/r;lt=np.cross([1,0,0],lr);wr=(entryPoint-center)/r
Rn=np.column_stack([wr,np.cross([0,1,0],wr),[0,1,0]])@np.column_stack([lr,lt,[1,0,0]]).T
entry=np.eye(4);entry[:3,:3]=Rn;entry[:3,3]=center-Rn@cl
sweep=2*np.arcsin(.008/r);emerge=np.radians(20);extract=np.radians(170)
def needle(theta,lift=(0,0,0)):
    t=entry.copy();t[:3,:3]=ry(theta)@Rn;t[:3,3]=center-t[:3,:3]@cl+lift;return t
def gripPoint(deg):
    angle=np.arctan2(rad[2],rad[1])-np.radians(deg)
    return cl+r*np.array([0,np.cos(angle),np.sin(angle)])
g={'R':gripPoint(125),'L':gripPoint(5)}
def tcpOrient(theta):
    z=np.array([-np.sin(theta),0,-np.cos(theta)]);y=np.array([0,1,0]);return np.column_stack([np.cross(y,z),y,z])
rel={}
for a,theta,start in [('R',np.radians(-55),0),('L',np.radians(-75),sweep+emerge)]:
    tc=np.eye(4)
    tc[:3,:3]=tcpOrient(theta)
    tc[:3,3]=needle(start)[:3,:3]@g[a]+needle(start)[:3,3]
    rel[a]=np.linalg.inv(tc)@needle(start)
def residual(q,arm,target):
    t=cal[arm]@fk(q,arm)
    return np.r_[(t[:3,3]-target[:3,3]),.04*Rotation.from_matrix(t[:3,:3]@target[:3,:3].T).as_rotvec()]
def dock(arm,target,pitch):
    best=None
    for roll in [0,np.pi/2,-np.pi/2,np.pi]:
        ref=q0[arm].copy();ref[6]=.06;ref[7:]=[roll,pitch,0,0]
        def fun(x):
            q=ref.copy();q[:7]=x
            return np.r_[residual(q,arm,target),.0002*(x[:6]-q0[arm][:6]),.001*(x[6]-.08)]
        opt=least_squares(fun,ref[:7],bounds=(np.r_[q0[arm][:6]-np.pi,.06-1e-9],np.r_[q0[arm][:6]+np.pi,.06+1e-9]),xtol=1e-11,gtol=1e-11,ftol=1e-11,max_nfev=250)
        q=ref.copy();q[:7]=opt.x;e=residual(q,arm,target)
        score=np.linalg.norm(e)*1000+np.linalg.norm(q[:6]-q0[arm][:6])
        if best is None or score<best[0]:best=(score,q)
    return best[1]
def solve(arm,target,q,ref):
    lo=np.r_[ref[:6]-np.radians(8),0,-4*np.pi,-np.radians(85)]
    hi=np.r_[ref[:6]+np.radians(8),.16,4*np.pi,np.radians(85)]
    def fun(x):
        qq=np.r_[x,0,0]
        return np.r_[residual(qq,arm,target),.0008*(x[:6]-ref[:6]),.00005*(x[7:9]-q[7:9])]
    opt=least_squares(fun,np.clip(q[:9],lo+1e-10,hi-1e-10),bounds=(lo,hi),max_nfev=100,xtol=1e-10,gtol=1e-10,ftol=1e-10)
    q=np.r_[opt.x,0,0]
    e=residual(q,arm,target)
    if np.linalg.norm(e[:3])>.00008 or np.linalg.norm(e[3:])/.04>.002:
        raise RuntimeError(f'{arm} unreachable: {e}, delta={np.degrees(q[:6]-ref[:6])}')
    return q
if __name__=='__main__':
    raise SystemExit('Legacy geometry module. Use plan_roll_motion.py for the validated roll-led cycle.')
    approach=needle(0,[0,0,.025]);exitN=needle(sweep+emerge)
    init={'R':approach@np.linalg.inv(rel['R']),'L':exitN@np.linalg.inv(rel['L'])}
    init['L'][:3,3]+=[-.025,.01,.035]
    docking={a:dock(a,init[a],np.radians(p)) for a,p in [('R',-77),('L',-84.4)]}
    print('docks', {a:np.round(q,6).tolist() for a,q in docking.items()},flush=True)
    initial={a:q.copy() for a,q in docking.items()}
    # Start with the needle already aligned above the holder-near pad.
    # Both TCP frames have the same world orientation; the mirrored tool
    # mounts do not generally have equal numerical wrist angles.
    for a in ['L','R']:
        start=init[a].copy();start[:3,:3]=init['R'][:3,:3]
        if a=='R':start[:3,3]+=[0,0,.015]
        initial[a]=solve(a,start,docking[a],docking[a])
    state={a:q.copy() for a,q in initial.items()};current={a:cal[a]@fk(state[a],a) for a in state};phases=[]
    def phase(name,arm,poses):
        traj=[]
        for target in poses:
            state[arm]=solve(arm,target,state[arm],docking[arm]);traj.append(state[arm].tolist())
        current[arm]=cal[arm]@fk(state[arm],arm)
        phases.append(dict(name=name,arm=arm,targets=[p.tolist() for p in poses],q=traj,needleAngles=[]))
        print(name,'max arm',np.round(np.max(np.abs(np.array(traj)[:,:6]-docking[arm][:6]),axis=0)*180/np.pi,2),'q7',round(state[arm][6]*1000,2),'pitch',round(np.degrees(state[arm][8]),2),flush=True)
    def line(name,a,target,n=24):phase(name,a,list(interpolate(current[a],target,n)))
    def prepare(a):
        traj=np.array([(1-t)*initial[a]+t*docking[a] for t in np.linspace(0,1,45)])
        targets=[cal[a]@fk(q,a) for q in traj]
        phases.append(dict(name=('right' if a=='R' else 'left')+'_prepare',arm=a,targets=[t.tolist() for t in targets],q=traj.tolist(),needleAngles=[]))
        state[a]=docking[a].copy();current[a]=targets[-1]
    prepare('R')
    line('right_contact','R',entry@np.linalg.inv(rel['R']))
    phase('right_insert','R',[needle(t)@np.linalg.inv(rel['R']) for t in np.linspace(0,sweep,65)])
    phases[-1]['needleAngles']=np.linspace(0,sweep,65).tolist()
    phase('right_expose','R',[needle(t)@np.linalg.inv(rel['R']) for t in np.linspace(sweep,sweep+emerge,45)])
    phases[-1]['needleAngles']=np.linspace(sweep,sweep+emerge,45).tolist()
    prepare('L')
    line('left_grasp','L',exitN@np.linalg.inv(rel['L']))
    retreat=current['R'].copy();retreat[:3,3]+=[.025,0,.025];line('right_retreat','R',retreat)
    pull=np.array([-.012,0,.018]);nfinal=needle(sweep+emerge+extract,pull)
    extractionAlphas=np.r_[np.linspace(0,.95,85),np.linspace(.95,1,40)[1:]]
    phase('left_extract','L',[needle(sweep+emerge+t*extract,np.clip((t-.95)/.05,0,1)*pull)@np.linalg.inv(rel['L']) for t in extractionAlphas])
    phases[-1]['needleAngles']=(sweep+emerge+extractionAlphas*extract).tolist()
    handoff=needle(np.pi);handoff[:3,3]=[-.03,.345,.635]
    line('left_handoff','L',handoff@np.linalg.inv(rel['L']))
    rightGrasp=np.eye(4);rightGrasp[:3,:3]=tcpOrient(np.radians(30));rightGrasp[:3,3]=handoff[:3,:3]@g['R']+handoff[:3,3]
    rightPre=rightGrasp.copy();rightPre[:3,3]+=[.015,0,.02]
    line('right_pregrasp','R',rightPre)
    line('right_regrasp','R',rightGrasp)
    retreat=current['L'].copy();retreat[:3,3]+=[-.025,0,.025];line('left_retreat','L',retreat)
    holder=(entry@np.linalg.inv(rel['R']))[:3,3]
    assert np.linalg.norm(holder-entryPoint)<np.linalg.norm(holder-exitPoint),'Entry must be on the holder-near pad.'
    armBase=np.array(obj(roots['R'])['pose'][:3])
    assert np.linalg.norm(armBase-entryPoint)<np.linalg.norm(armBase-exitPoint),'Entry must also be on the holding arm base side.'
    output=dict(proximal_limit_deg=8,exposure_deg=float(np.degrees(emerge)),extraction_deg=float(np.degrees(extract)),entryPoint=entryPoint.tolist(),exitPoint=exitPoint.tolist(),initial={a:q.tolist() for a,q in initial.items()},dock={a:q.tolist() for a,q in docking.items()},relations={a:t.tolist() for a,t in rel.items()},calibration={a:t.tolist() for a,t in cal.items()},grips={a:x.tolist() for a,x in g.items()},phases=phases,needleEntry=entry.tolist(),sweep=float(sweep))
    (ROOT/'work/suture_review/wrist_plan.json').write_text(json.dumps(output))
