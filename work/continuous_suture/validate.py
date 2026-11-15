"""Check the full five-stitch chain against actual scene meshes, including jaws."""
from common import *
import hashlib
c,s=connect();assert s.getSimulationState()==s.simulation_stopped
p=json.loads(PLAN.read_text());h,call=initialize(s,p)
q={a:np.array(p['initial'][a]) for a in ['L','R']};grip={'L':.35,'R':0}
fixed=np.array(call('getSutureAudit')['fixed']);rows=[];transfers=[]
checks=['arm_arm','left_pad','right_pad','left_table','right_table','left_self','right_self']
def audit(st,name,index,target=None,arm=None):
    hits={k:st[k] for k in checks if st[k]['collision']}
    assert not hits,(name,index,hits)
    assert np.max(abs(np.array(st['fixed'])-fixed))<1e-9
    assert abs(np.array(st['needle']).reshape(3,4)[2,0])<np.sin(np.radians(.5)),(name,'tilt')
    for i,a in enumerate(['L','R']):
        live=np.array(st['joints'][i]);assert 0<=live[6]<=.16000001 and abs(live[7])<=np.pi+1e-8 and abs(live[8])<=np.radians(85)+1e-8
    if target is not None:
        assert np.linalg.norm(np.array(st['tcps'][arm-1]).reshape(3,4)[:,3]-np.array(target)[:3,3])<.0001
    st.update(phase=name,step=index);rows.append(st)
for ph in p['phases']:
    a=ph['arm'];idx=1 if a=='L' else 2;name=str(ph['stitch'])+'_'+ph['name'];traj=np.array(ph['q'])
    assert np.max(abs(traj[:,7]))<=np.pi and np.max(abs(traj[:,8]))<=np.radians(85)
    physical=traj.copy();physical[:,9:]+=grip[a]
    for start in range(0,len(traj),10):
        batch=call('auditSutureBatch',idx,physical[start:start+10].tolist())
        for k,st in enumerate(batch):audit(st,name,start+k,ph['targets'][start+k],idx)
    q[a]=traj[-1]
    if ph['name'] in ['left_grasp','right_regrasp']:
        receiver='L' if ph['name']=='left_grasp' else 'R';donor='R' if receiver=='L' else 'L'
        for side,target in [(receiver,0),(donor,.35)]:
            if side==donor:
                pair=np.array(call('handoffSutureNeedle',1 if receiver=='L' else 2));assert np.max(abs(pair[0]-pair[1]))<1e-9
                transfers.append(dict(stitch=ph['stitch'],receiver=receiver,max_matrix_change=float(np.max(abs(pair[0]-pair[1])))))
            for g in np.linspace(grip[side],target,12):
                qs=q[side].copy();qs[9:]+=g;call('setSutureArm',1 if side=='L' else 2,qs.tolist());audit(call('getSutureAudit'),name+'_jaws',0)
            grip[side]=target
    print(name,'PASS',flush=True)
(ROOT/'work/continuous_suture/mesh_audit.json').write_text(json.dumps(rows))
(ROOT/'output/reports/continuous_suture_plan_validation.json').write_text(json.dumps(dict(result='PASS',checked_frames=len(rows),handoffs=transfers,collision_events=0,plan_sha256=hashlib.sha256(PLAN.read_bytes()).hexdigest()),indent=2))
initialize(s,p)
print('ALL PASS',len(rows),flush=True)
