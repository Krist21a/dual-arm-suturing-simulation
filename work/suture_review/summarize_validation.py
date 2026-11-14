import sys,json,csv,math
from pathlib import Path
ROOT=Path(r'D:\仿真环境');sys.path.insert(0,str(ROOT/'work/python_deps'))
import numpy as np
from scipy.spatial.transform import Rotation
p=json.loads((ROOT/'work/suture_review/wrist_plan.json').read_text());d=json.loads((ROOT/'work/suture_review/scene_live.json').read_text())
rows=[json.loads(x) for x in (ROOT/'output/reports/suture_live_frames.jsonl').read_text().splitlines()]
def matrix(x):return np.array(x).reshape(3,4)
q=np.array([x['joints'] for x in rows]);needle=np.array([matrix(x['needle']) for x in rows]);tips=np.array([x['tip'] for x in rows])
checks=['arm_arm','left_pad','right_pad','left_table','right_table'];hits=sum(x[k]['collision']>0 for x in rows for k in checks)
assert hits==0
phases={v['name']:v for v in p['phases']}
summary=[]
for name,phase in phases.items():
    sample=[x for x in rows if x['phase']==name];arm=0 if phase['arm']=='L' else 1;targets=np.array(phase['targets']);ep=[];eo=[]
    for x in sample:
        target=targets[x['step']-1];t=matrix(x['tcps'][arm]);ep.append(np.linalg.norm(t[:,3]-target[:3,3]));eo.append(Rotation.from_matrix(t[:,:3]@target[:3,:3].T).magnitude())
    qs=np.array([x['joints'][arm] for x in sample]);ref=np.array(p['dock'][phase['arm']]);delta=np.degrees(np.abs(qs[:,:6]-ref[:6])).max()
    assert max(ep)<.0001 and max(eo)<.002,(name,max(ep),max(eo))
    summary.append([name,phase['arm'],'PASS',len(sample),max(ep)*1000,math.degrees(max(eo)),qs[:,6].min()*1000,qs[:,6].max()*1000,np.degrees(np.abs(qs[:,8])).max(),delta])
with (ROOT/'output/reports/suture_trajectory_validation.tsv').open('w',encoding='utf-8',newline='') as f:
    w=csv.writer(f,delimiter='\t');w.writerow(['phase','arm','result','frames','tcp_error_mm','orientation_error_deg','joint7_min_mm','joint7_max_mm','joint9_abs_max_deg','proximal_max_delta_deg']);w.writerows(summary)
mesh=np.array(next(x for x in d if x['alias'].endswith('/Needle'))['mesh'][0]).reshape(-1,3)
corners=[]
for name in ['PadBase','TissueLeft','TissueRight']:
    o=next(x for x in d if x['alias'].endswith('/'+name));T=matrix(o['matrix']);corners.extend(np.array(o['mesh'][0]).reshape(-1,3)@T[:,:3].T+T[:,3])
vision=[]
for side in ['left','right']:
    o=next(x for x in d if x['alias']=='/visionSensor_'+side);cam=matrix(o['matrix']);w,h=o['resolution'];tx=math.tan(o['angle']/2);ty=tx/(w/h)
    def uv(world):
        v=(world-cam[:,3])@cam[:,:3];assert v[:,2].min()>0
        return np.column_stack([(1-v[:,0]/v[:,2]/tx)*w/2,(1-v[:,1]/v[:,2]/ty)*h/2])
    pad=uv(np.array(corners));allNeedle=np.concatenate([uv(mesh@T[:,:3].T+T[:,3]) for T in needle]);margin=lambda u:float(min(u[:,0].min(),w-u[:,0].max(),u[:,1].min(),h-u[:,1].max()))
    vision.append([side,'PASS' if margin(pad)>=0 and margin(allNeedle)>=0 else 'FAIL',margin(pad),margin(allNeedle),w,h]);assert margin(pad)>0 and margin(allNeedle)>0
with (ROOT/'output/reports/suture_stereo_visibility.tsv').open('w',encoding='utf-8',newline='') as f:
    w=csv.writer(f,delimiter='\t');w.writerow(['sensor','result','pad_min_border_px','needle_min_border_px','width','height']);w.writerows(vision)
contact=next(x for x in reversed(rows) if x['phase']=='right_contact');exit=next(x for x in reversed(rows) if x['phase']=='right_insert')
fixed=np.array([x['fixed'] for x in rows]);drift=np.abs(fixed-fixed[0]).max()
report={'result':'PASS','checked_frames':len(rows),'collision_events':hits,'fixed_pose_max_drift':float(drift),'entry_point_m':p['entryPoint'],'exit_point_m':p['exitPoint'],'entry_error_mm':float(np.linalg.norm(np.array(contact['tip'])-p['entryPoint'])*1000),'exit_error_mm':float(np.linalg.norm(np.array(exit['tip'])-p['exitPoint'])*1000),'joint7_max_mm':(q[:,:,6].max(axis=0)*1000).tolist(),'joint9_abs_max_deg':np.degrees(np.abs(q[:,:,8]).max(axis=0)).tolist(),'proximal_max_delta_deg':{a:max(x[-1] for x in summary if x[1]==a) for a in ['L','R']},'jaw_logic':'Original: both joint10 and joint11 go to 0.35 rad for open, 0 rad for closed. No geometry/axis/sign/yaw change.','table_surface_z_m':.57535001,'pad_bottom_z_m':.5755,'table_model_clearance_mm':.14999,'needle_radius_mm':11.375,'tip_entry_to_exit_rotation_deg':math.degrees(p['sweep']),'additional_exposure_deg':p['exposure_deg'],'extraction_rotation_deg':p['extraction_deg'],'thread_model':'Kinematic drawn strand, not deformable tissue/thread mechanics. No knot tying.','visibility':vision}
(ROOT/'output/reports/suture_final_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
