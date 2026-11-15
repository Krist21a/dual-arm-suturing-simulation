"""Roll-led needle motion, original jaws and fixed pad/cameras."""
from plan_wrist_motion import *
import os
from scipy.spatial.transform import Rotation
from suture_wrist_kinematics import fk
from posture_geometry import chain
from choose_grasp_wrist import choose_wrist

# Hard geometry constraint: needle plane is vertical to the pad surface.
# Shaft orientation and wrist optimisation must adapt to this, never tilt it.
planeTilt=np.eye(3)
normal=planeTilt@np.array([0.,1.,0.])
center=(entryPoint+exitPoint)/2+planeTilt@np.array([0,0,np.sqrt(r*r-.008**2)])
wr=(entryPoint-center)/r
Rn=np.column_stack([wr,np.cross(normal,wr),normal])@np.column_stack([lr,lt,[1,0,0]]).T
entry=np.eye(4);entry[:3,:3]=Rn;entry[:3,3]=center-Rn@cl
def needle(theta,lift=(0,0,0)):
    t=entry.copy();t[:3,:3]=Rotation.from_rotvec(normal*theta).as_matrix()@Rn;t[:3,3]=center-t[:3,:3]@cl+lift;return t
def residual(q,arm,target):
    t=cal[arm]@fk(q,arm)
    return np.r_[t[:3,3]-target[:3,3],.04*Rotation.from_matrix(t[:3,:3]@target[:3,:3].T).as_rotvec()]

def axis_pose(a,pitch,start,point,sign=1):
    q=q0[a].copy();q[7:]=[0,np.radians(pitch),0,0]
    T=cal[a]@fk(q,a);q[7]=1e-5;U=cal[a]@fk(q,a)
    axis=Rotation.from_matrix(U[:3,:3]@T[:3,:3].T).as_rotvec()/1e-5
    q[7]=np.pi;V=cal[a]@fk(q,a);offset=T[:3,3]-V[:3,3]
    radial=offset-axis*np.dot(offset,axis);radial/=np.linalg.norm(radial)
    source=np.column_stack([radial,np.cross(axis,radial),axis])
    targetAxis=np.array([.5 if a=='L' else -.5,.45,-np.sqrt(.5475)]);targetAxis/=np.linalg.norm(targetAxis)
    # Align the TCP's offset from the shaft with the needle radius at the
    # grasp. Roll then rotates both about nearly the same centre; it must
    # not sweep the shaft down through the pad as an arbitrary jaw tilt does.
    targetRadial=needle(start)[:3,:3]@(point-cl);targetRadial-=targetAxis*np.dot(targetRadial,targetAxis);targetRadial/=np.linalg.norm(targetRadial)
    targetRadial=Rotation.from_rotvec(targetAxis*np.radians(145 if a=='L' else 35)).as_matrix()@targetRadial
    dest=np.column_stack([targetRadial,np.cross(targetAxis,targetRadial),targetAxis])
    return dest@source.T@T[:3,:3]

def module_front(q,a):
    zero=q.copy();zero[7:]=0
    return -(cal[a]@fk(zero,a))[:3,0]

def desired_front(a):
    axis=np.array([.5 if a=='L' else -.5,.45,-np.sqrt(.5475)])
    front=np.array([.4 if a=='L' else -.4,1.,.8]);front-=axis*np.dot(front,axis)
    return front/np.linalg.norm(front)

def roll_dock(a,target,pitch):
    candidates=[]
    for k in range(10):
        ref=q0[a].copy();ref[6:]=[.06,0,np.radians(pitch),0,0]
        seed=ref[:8].copy()
        reach=np.radians([75,50,65,120,100,180])
        if k:seed[:6]+=np.random.default_rng(50+k).uniform(-1,1,6)*reach
        def fun(x):
            qq=ref.copy();qq[:8]=x
            elbow=chain(qq,a)[2];side=1 if a=='L' else -1
            outside=max(0,side*(elbow[0]-(-.35 if a=='L' else .338)))
            zero=qq.copy();zero[7:]=0;axis=(cal[a]@fk(zero,a))[:3,2]
            preferred=np.array([.5 if a=='L' else -.5,.45,-np.sqrt(.5475)])
            e=residual(qq,a,target)
            return np.r_[e[:3]*10,e[3:]*.015,.007*(axis-preferred),.05*max(0,.32-axis[1]),.0002*(module_front(qq,a)-desired_front(a)),.05*max(0,.55-module_front(qq,a)[1]),outside,max(0,.84-elbow[2]),.00003*(x[:6]-q0[a][:6]),.02*(x[6]-.06)]
        lower=np.r_[q0[a][:6]-reach,0,-np.pi];upper=np.r_[q0[a][:6]+reach,.09,np.pi]
        lower[2],upper[2]=np.radians([80,155] if a=='L' else [-155,-80])
        opt=least_squares(fun,np.clip(seed,lower+1e-9,upper-1e-9),bounds=(lower,upper),max_nfev=200,gtol=1e-11,ftol=1e-11,xtol=1e-11)
        qq=ref.copy();qq[:8]=opt.x;e=residual(qq,a,target)
        score=np.linalg.norm(e[:3])*1e4+np.linalg.norm((qq[:6]-q0[a][:6]+np.pi)%(2*np.pi)-np.pi)
        if k<4: print('dock fit',a,np.linalg.norm(e),np.degrees(qq[:6]-q0[a][:6]),module_front(qq,a),desired_front(a),flush=True)
        elbow=chain(qq,a)[2]
        zq=qq.copy();zq[7:]=0;shaft=(cal[a]@fk(zq,a))[:3,2]
        if np.linalg.norm(e[:3])<.00002 and (elbow[0]<-.33 if a=='L' else elbow[0]>.318) and elbow[2]>.82 and module_front(qq,a)[1]>.52 and shaft[1]>.30:
            sim.callScriptFunction('setSutureArm',auditHandle,1 if a=='L' else 2,qq.tolist())
            st=sim.callScriptFunction('getSutureAudit',auditHandle)
            side='left' if a=='L' else 'right'
            if st[side+'_table']['collision'] or st[side+'_pad']['collision']:continue
        else:continue
        if not any(np.linalg.norm(qq-c[1])<.01 for c in candidates):candidates.append((score,qq))
    assert candidates,(a,'no collision-free dock')
    print(a,'module_front',[(np.round(module_front(q,a),3)).tolist() for _,q in candidates],flush=True)
    return sorted(candidates,key=lambda x:x[0])

def roll_solve(a,target,q,ref,rollTarget=None):
    allowance=np.radians([5,5,5,5,5,5])
    lo=np.r_[ref[:6]-allowance,0,-np.pi,-np.radians(85),-np.radians(80)]
    hi=np.r_[ref[:6]+allowance,.16,np.pi,np.radians(85),np.radians(80)]
    def effective(x):return np.r_[x[:9],x[9],-x[9]]
    def fun(x):
        task=residual(effective(x),a,target)
        return np.r_[task,.002*(x[:6]-ref[:6]),.000015*(x[8]-ref[8]),.00001*(x[7]-q[7])]
    opt=least_squares(fun,np.clip(np.r_[q[:9],(q[9]-q[10])/2],lo+1e-10,hi-1e-10),bounds=(lo,hi),max_nfev=150,gtol=1e-11,xtol=1e-11,ftol=1e-11)
    qq=effective(opt.x);e=residual(qq,a,target)
    assert np.linalg.norm(e[:3])<.00008 and np.linalg.norm(e[3:])/.04<.002,(a,'unreachable',e,'delta_deg',np.degrees(qq[:6]-ref[:6]).round(2),'j7',qq[6],'wrist',np.degrees(qq[7:9]).round(2),'yaw',np.degrees(qq[9]).round(2))
    return qq

if __name__=='__main__':
    sys.path.insert(0,r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\python\src')
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    client=RemoteAPIClient(port=23001);client.initialTimeout=120;sim=client.require('sim')
    assert sim.getSimulationState()==sim.simulation_stopped
    auditHandle=sim.getScript(sim.scripttype_addonscript,'record_suture_stereo')
    sim.callScriptFunction('loadSutureAudit',auditHandle,(ROOT/'work/suture_review/audit_runtime.lua').read_text())
    sim.callScriptFunction('initSutureAudit',auditHandle)
    emerge=np.radians(35);extract=np.radians(170)
    g={'R':gripPoint(155),'L':gripPoint(5)}
    rel={};pitch={'L':35,'R':35}
    for a,start in [('R',0),('L',sweep+emerge)]:
        tc=np.eye(4);tc[:3,:3]=axis_pose(a,pitch[a],start,g[a]);tc[:3,3]=needle(start)[:3,:3]@g[a]+needle(start)[:3,3]
        rel[a]=np.linalg.inv(tc)@needle(start)
    entryN=needle(0);exitN=needle(sweep+emerge)
    docking={};initial={}
    if os.getenv('SUTURE_REUSE_DOCKS')=='1':
        validPairs=[{a:np.array(q) for a,q in pair.items()} for pair in json.loads((ROOT/'work/suture_review/natural_dock_candidates.json').read_text())]
    else:
        docking={};initial={};candidates={}
        for a in ['L','R']:
            pose=(entryN if a=='R' else exitN)@np.linalg.inv(rel[a]);pose[:3,3]+=[0,-.02,.03]
            candidates[a]=roll_dock(a,pose,pitch[a]);print(a,'candidates',len(candidates[a]),flush=True)
        pairs=sorted([(l[0]+r[0],l[1],r[1]) for l in candidates['L'] for r in candidates['R']],key=lambda x:x[0])
        validPairs=[]
        for _,ql,qr in pairs:
            sim.callScriptFunction('setSutureArm',auditHandle,1,ql.tolist());sim.callScriptFunction('setSutureArm',auditHandle,2,qr.tolist())
            st=sim.callScriptFunction('getSutureAudit',auditHandle)
            if st['arm_arm']['collision']==0:validPairs.append({'L':ql,'R':qr})
        assert validPairs,'No collision-free initial arm pair'
        print('safe initial pairs',len(validPairs),flush=True)
    def build(docking):
        docking={a:q.copy() for a,q in docking.items()}
        for a in ['L','R']:
            dock=docking[a];start=0 if a=='R' else sweep+emerge
            target=(needle(start)@np.r_[g[a],1])[:3]+[0,-.02,.03]
            travel=extract if a=='L' else sweep+emerge
            positions=np.array([(needle(start+t)@np.r_[g[a],1])[:3] for t in np.linspace(0,travel,41)])
            choices=choose_wrist(dock,a,travel,positions)
            choice=choices[min(int(os.getenv('SUTURE_WRIST_'+a,'0')),len(choices)-1)]
            _,delta,pitch0,yaw0,*_=choice
            dock[7]=(dock[7]+np.radians(delta)+np.pi)%(2*np.pi)-np.pi;dock[8]=np.radians(pitch0);dock[9:]=[np.radians(yaw0),-np.radians(yaw0)]
            reference=dock.copy()
            def adjust(x):
                t=reference.copy();t[:7]=x
                return np.r_[10*((cal[a]@fk(t,a))[:3,3]-target),.001*(x[:6]-reference[:6]),.01*(x[6]-reference[6])]
            fit=least_squares(adjust,reference[:7],bounds=(np.r_[reference[:6]-.06,0],np.r_[reference[:6]+.06,.1]),gtol=1e-12,xtol=1e-12,ftol=1e-12)
            dock[:7]=fit.x
            print('wrist coverage',a,choice,flush=True)
        grip={'L':.35,'R':0}
        for a in ['L','R']:
            qq=docking[a].copy();qq[9:]+=grip[a];sim.callScriptFunction('setSutureArm',auditHandle,1 if a=='L' else 2,qq.tolist())
        audit=sim.callScriptFunction('getSutureAudit',auditHandle)
        hits=[k for k,v in audit.items() if isinstance(v,dict) and v.get('collision',0)>0]
        assert not hits,('initial collision',hits)
        for a,start in [('R',0),('L',sweep+emerge)]:
            actual=cal[a]@fk(docking[a],a);actual[:3,3]-=[0,-.02,.03]
            rel[a]=np.linalg.inv(actual)@needle(start)
        for a in ['L','R']:
            initial[a]=docking[a].copy();print(a,'dock',np.round(docking[a],5).tolist(),'original arm delta',np.degrees(docking[a][:6]-q0[a][:6]).round(1).tolist(),flush=True)
        state={a:q.copy() for a,q in initial.items()};current={a:cal[a]@fk(state[a],a) for a in state};phases=[]
        def phase(name,a,poses,angles=None):
            nonlocal grip
            traj=[]
            for target in poses:state[a]=roll_solve(a,target,state[a],docking[a]);traj.append(state[a].tolist())
            physical=np.array(traj);physical[:,9:]+=grip[a]
            for begin in range(0,len(physical),15):
                audits=sim.callScriptFunction('auditSutureBatch',auditHandle,1 if a=='L' else 2,physical[begin:begin+15].tolist())
                for k,audit in enumerate(audits):
                    hits={key:v for key,v in audit.items() if isinstance(v,dict) and v.get('collision',0)>0}
                    assert not hits,(name,'mesh collision',begin+k,hits)
            current[a]=cal[a]@fk(state[a],a)
            if name in ['left_grasp','right_regrasp']:
                grip={'L':0,'R':.35} if name=='left_grasp' else {'L':.35,'R':0}
                for side in ['L','R']:
                    qq=state[side].copy();qq[9:]+=grip[side];sim.callScriptFunction('setSutureArm',auditHandle,1 if side=='L' else 2,qq.tolist())
            phases.append(dict(name=name,arm=a,targets=[t.tolist() for t in poses],q=traj,needleAngles=[] if angles is None else list(angles)))
            q=np.array(traj);print(name,'arm',round(np.degrees(abs(q[:,:6]-docking[a][:6])).max(),2),'roll',np.degrees(q[:,7])[[0,-1]].round(1),'pitch',np.degrees(q[:,8])[[0,-1]].round(1),'j7',q[:,6].min()*1000,q[:,6].max()*1000,flush=True)
        def line(name,a,target,n=30):phase(name,a,list(interpolate(current[a],target,n)))
        approach=entryN@np.linalg.inv(rel['R']);approach[:3,3]+=[0,0,.025]
        line('right_prepare','R',approach)
        line('right_contact','R',entryN@np.linalg.inv(rel['R']))
        for name,begin,end,n in [('right_insert',0,sweep,80),('right_expose',sweep,sweep+emerge,40)]:
            angles=np.linspace(begin,end,n);phase(name,'R',[needle(t)@np.linalg.inv(rel['R']) for t in angles],angles)
        leftTarget=exitN@np.linalg.inv(rel['L']);leftPre=leftTarget.copy();leftPre[:3,3]+=[-.015,0,.025]
        line('left_prepare','L',leftPre);line('left_grasp','L',leftTarget)
        retreat=current['R'].copy();retreat[:3,3]+=[.025,-.035,.035];line('right_retreat','R',retreat)
        alphas=np.r_[np.linspace(0,.95,100),np.linspace(.95,1,30)[1:]];angles=sweep+emerge+alphas*extract
        phase('left_extract','L',[needle(t,np.clip((a-.95)/.05,0,1)*np.array([0,0,.02]))@np.linalg.inv(rel['L']) for a,t in zip(alphas,angles)],angles)
        handoff=needle(sweep+emerge);handoff[:3,3]+=[0,-.01,.04]
        line('left_handoff','L',handoff@np.linalg.inv(rel['L']))
        rightGrasp=handoff@np.linalg.inv(rel['R']);rightPre=rightGrasp.copy();rightPre[:3,3]+=[.015,-.02,.01]
        line('right_pregrasp','R',rightPre);line('right_regrasp','R',rightGrasp)
        retreat=current['L'].copy();retreat[:3,3]+=[-.025,-.025,.03];line('left_retreat','L',retreat)
        assert abs(normal[2])<1e-12,'Needle plane must remain upright.'
        output=dict(mode='roll-led',arcCenter=center.tolist(),arcNormal=normal.tolist(),needle_plane_tilt_deg=0,proximal_limit_deg=5,exposure_deg=float(np.degrees(emerge)),extraction_deg=float(np.degrees(extract)),entryPoint=entryPoint.tolist(),exitPoint=exitPoint.tolist(),initial={a:q.tolist() for a,q in initial.items()},dock={a:q.tolist() for a,q in docking.items()},relations={a:t.tolist() for a,t in rel.items()},calibration={a:t.tolist() for a,t in cal.items()},grips={a:x.tolist() for a,x in g.items()},phases=phases,needleEntry=entry.tolist(),sweep=float(sweep))
        return output









    (ROOT/'work/suture_review/natural_dock_candidates.json').write_text(json.dumps([{a:q.tolist() for a,q in d.items()} for d in validPairs]))
    for attempt,docking in enumerate(validPairs[:20]):
        try:
            output=build(docking)
            (ROOT/'work/suture_review/wrist_plan.json').write_text(json.dumps(output))
            print('PLAN COMPLETE',attempt,flush=True);break
        except AssertionError as err:print('candidate failed',attempt,str(err)[:600],flush=True)
    else:raise RuntimeError('No feasible complete roll trajectory')
