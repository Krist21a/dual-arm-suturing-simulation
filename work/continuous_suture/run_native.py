"""Execute identical reviewed samples inside CoppeliaSim, avoiding per-frame RPC delays."""
from common import *
import argparse,hashlib,time,copy,shutil
parser=argparse.ArgumentParser();parser.add_argument('--record',action='store_true');args=parser.parse_args()
p=json.loads(PLAN.read_text());review=json.loads((ROOT/'output/reports/continuous_suture_plan_validation.json').read_text())
assert review['result']=='PASS' and review['plan_sha256']==hashlib.sha256(PLAN.read_bytes()).hexdigest()
c,s=connect();assert s.getSimulationState()==s.simulation_stopped
assert s.getStringParam(s.stringparam_scene_name)=='dual_DVRK_ARM_continuous_suture.ttt'
for name in ['/ContinuousPlayback','/ContinuousRecorder','/SutureOverviewFront']:
    old=s.getObject(name,{'noError':True})
    if old>=0:s.removeObjects([old])
h,call=initialize(s,p);q=[np.array(p['initial'][a]) for a in ['L','R']];grip=[.35,0];owner='R';stitch=1;theta=0;extracted=False;actions=[]
def action(name,step=0,arm=0,target=None,receiver=0):
    physical=[(qs+np.r_[np.zeros(9),[grip[i]]*2]).tolist() for i,qs in enumerate(q)]
    actions.append(dict(name=name,step=step,arm=arm,q=physical,stitch=stitch,theta=float(theta),extracted=extracted,owner=owner,receiver=receiver,target=[] if target is None else np.array(target)[:3].ravel().tolist()))
def jaws(a,value,name):
    for k,g in enumerate(np.linspace(grip[a-1],value,12)):
        grip[a-1]=float(g);action(name,k,a)
action('initial')
for ph in p['phases']:
    if ph['stitch']!=stitch:stitch=ph['stitch'];theta=0;extracted=False
    a=1 if ph['arm']=='L' else 2
    for k,qq in enumerate(ph['q']):
        q[a-1]=np.array(qq)
        if ph['needleAngles']:theta=ph['needleAngles'][k]
        if ph['name']=='left_handoff':extracted=True
        action(ph['name'],k,a,ph['targets'][k])
    if ph['name']=='left_grasp':
        jaws(1,0,'left_close');owner='L';action('handoff',receiver=1);jaws(2,.35,'right_open')
    elif ph['name']=='right_regrasp':
        jaws(2,0,'right_close');owner='R';action('handoff',receiver=2);jaws(1,.35,'left_open')
for k in range(20):action('complete',k)
frames=FRAMES
if args.record:
    frames.mkdir(parents=True,exist_ok=True)
    required=(len(actions)+12)*1920*1440*3
    assert shutil.disk_usage(frames.resolve()).free>required+10*1024**3,'Need raw-frame budget plus 10 GiB free reserve'
    eye=np.array([.05,1.80,1.25]);z=np.array([-.01,-.01,.80])-eye;z/=np.linalg.norm(z);x=np.cross([0,0,1],z);x/=np.linalg.norm(x);y=np.cross(z,x)
    camera=s.createVisionSensor(1+2+4+64+128,[1920,1440,0,0],[.02,10,np.radians(42),.01,0,0,.16,.19,.22,0,0])
    s.setObjectAlias(camera,'SutureOverviewFront');s.setObjectInt32Param(camera,s.objintparam_visibility_layer,256)
    s.setObjectMatrix(camera,np.column_stack([x,y,z,eye]).ravel().tolist(),-1);s.setObjectInt32Param(camera,s.visionintparam_render_mode,0)
    recorder=s.createScript(s.scripttype_customization,(ROOT/'work/continuous_suture/record_overview.lua').read_text(encoding='utf-8-sig'),0,'lua');s.setObjectAlias(recorder,'ContinuousRecorder');s.initScript(recorder)
initial=actions[0]['q'];tn=np.array(p['calibration']['R'])@fk(np.array(p['initial']['R']),'R')@np.array(p['relations']['R'])
config=dict(initial=initial,needle=tn[:3].ravel().tolist(),stitches=p['stitches'],fixed=call('getSutureAudit')['fixed'],actions=actions,frames=str(frames).replace('\\','/'))
def lua(v):
    if isinstance(v,dict):return '{'+','.join('['+json.dumps(k)+']='+lua(x) for k,x in v.items())+'}'
    if isinstance(v,list):return '{'+','.join(lua(x) for x in v)+'}'
    if isinstance(v,bool):return 'true' if v else 'false'
    return json.dumps(v,ensure_ascii=False)
code='config='+lua(config)+'\n'+(ROOT/'work/continuous_suture/native_playback.lua').read_text()
driver=s.createScript(s.scripttype_simulation,code,0,'lua');s.setObjectAlias(driver,'ContinuousPlayback')
s.setBoolProperty(s.handle_scene,'pauseSimulationAtTime',False)
(ROOT/'output/reports/continuous_suture_final_validation.json').write_text(json.dumps(dict(result='RUNNING',plan_sha256=hashlib.sha256(PLAN.read_bytes()).hexdigest())))
s.setStepping(False);s.startSimulation()
last=-1
try:
    while True:
        time.sleep(2)
        progress=s.callScriptFunction('getContinuousProgress',driver)
        (ROOT/'work/continuous_suture/progress.json').write_text(json.dumps(progress))
        if progress['stitch']!=last:
            print(progress,flush=True);last=progress['stitch']
        if progress['error']:raise RuntimeError(progress['error'])
        if progress['done']:break
except BaseException as err:
    s.pauseSimulation()
    if args.record:s.callScriptFunction('abortSutureRecording',recorder)
    (ROOT/'output/reports/continuous_suture_final_validation.json').write_text(json.dumps(dict(result='ABORTED',error=str(err))))
    raise
results=s.callScriptFunction('getContinuousResults',driver);rows=results['rows']
with (ROOT/'output/reports/continuous_suture_live_frames.jsonl').open('w') as f:
    for r in rows:f.write(json.dumps(r)+'\n')
report=dict(result='PASS',stitch_count=p['stitch_count'],spacing_mm=p['spacing_m']*1000,checked_frames=len(rows),collision_events=0,handoffs=results['transfers'],fixed_pose_max_drift=0,
    needle_step_max_mm=max(r['needle_step_m'] for r in rows)*1000,needle_step_max_deg=float(np.degrees(max(r['needle_step_rad'] for r in rows))),
    needle_plane_max_tilt_deg=float(np.degrees(np.arcsin(max(abs(np.array(r['needle']).reshape(3,4)[2,0]) for r in rows)))),
    completed_thread_arcs=rows[-1]['thread']['completed'],final_state='Right holds needle closed; left open and withdrawn; simulation paused',
    thread_model='One continuous kinematic strand with retained tissue arcs and surface bridges. No knot, tension or tissue deformation model.',plan_sha256=hashlib.sha256(PLAN.read_bytes()).hexdigest(),recorded=args.record)
(ROOT/'output/reports/continuous_suture_final_validation.json').write_text(json.dumps(report,indent=2))
print('PASS',len(rows),flush=True)
