"""Recompute the coupled mechanics for the complete five-stitch arm motion."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from pathlib import Path
import argparse,json,hashlib,time
from dataclasses import asdict
import numpy as np
from mechanics import SoftSuture,Parameters
from trajectory import build_actions,ROOT

def run(output,dt=.025,pull=True,natural=False):
    plan_path=ROOT/'work/continuous_suture/continuous_plan.json'
    p=json.loads(plan_path.read_text());actions,ik=build_actions(p)
    model=SoftSuture(Parameters(dt=dt,strain_stiffening=4. if natural else 0.,edge_viscosity=.15 if natural else 0.,distributed_contact=natural))
    if natural:
        from natural_strand import strand
        previous=ROOT/'output/soft_suture_continuous'
        metadata=json.loads((previous/'manifest.json').read_text())
        corrected=np.fromfile(previous/'frames.bin',dtype='<f4').reshape(-1,metadata['state_width'])
        for a,row in zip(actions,corrected):
            assert np.max(abs(np.array(a['needle'])-row[22:34]))<1e-6
            a['q']=row[:22].reshape(2,11).tolist()
    rows=[];states=[];deployed=set();before=None
    tail_local=np.array([0.,.005708948958544,.01124811054846])
    # This is exactly the opposite endpoint of the accepted 11.375 mm needle.
    center=np.array([.005854474479272,-.000125944725770])
    tail_local[1:]=2*center-np.array([.006,-.0115])
    previous_tip=previous_tail=None;start=time.time()
    for i,a in enumerate(actions):
        T=np.array(a['needle']).reshape(3,4)
        tip=T[:,:3]@np.array([0,.006,-.0115])+T[:,3]
        tail=T[:,:3]@tail_local+T[:,3]
        if not model.route:model.start_strand(tail)
        if a['phase']=='left_handoff' and a['stitch'] not in deployed:
            if a['stitch'] not in model.punctured:
                raise RuntimeError(f'Stitch {a["stitch"]} did not puncture; no thread tunnel created')
            model.deploy(a['stitch'],tail);deployed.add(a['stitch'])
        pulling=a['phase'] in ['tighten','hold_tension']
        if pulling and before is None:
            before=dict(gap_mm=model.gap_mm().tolist(),tail=previous_tail.tolist(),frame=i)
        if pulling and not pull:tail=np.array(before['tail'])
        steps=round(.05/dt)
        for k in range(1,steps+1):
            frac=k/steps
            t=tip if previous_tip is None else previous_tip*(1-frac)+tip*frac
            end=tail if previous_tail is None else previous_tail*(1-frac)+tail*frac
            row=model.step(tip=t,puncture_key=a['stitch'],cable_end=end,feed=not pulling)
        previous_tip=tip;previous_tail=tail
        row.update(frame=i,phase=a['phase'],stitch=a['stitch'],deployed=len(deployed),
                   cable_length_m=model.cable_length(),cable_rest_m=model.cable_rest)
        rows.append(row)
        visible=model.display_cable(a['stitch'],a['theta'],a['phase'],len(deployed))
        if natural:visible=strand(model,visible)
        cable=np.zeros((512 if natural else 22,3));n=len(visible);cable[:n]=visible
        row['visible_thread_vertices']=n
        state=np.r_[np.array(a['q']).ravel(),a['needle'],model.x[:model.n_tissue].ravel(),n,cable.ravel()]
        states.append(state.astype('<f4'))
        if i%200==0:
            print(i,len(actions),a['phase'],'punctures',len(model.punctured),
                  'gap',np.round(model.gap_mm(),2),'elapsed',round(time.time()-start),flush=True)
    output.mkdir(parents=True,exist_ok=True)
    np.array(states,dtype='<f4').tofile(output/'frames.bin')
    (output/'metrics.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
    report=dict(result='COMPUTED',model='implicit elastic lattice + massless frictionless continuous cable',
        parameters=asdict(model.p),frame_count=len(states),frame_dt=.05,
        state_width=len(states[0]),tissue_nodes=model.n_tissue,faces=model.faces.tolist(),
        plan_sha256=hashlib.sha256(plan_path.read_bytes()).hexdigest(),pull_enabled=pull,
        puncture_events=model.puncture_events,before_tightening=before,
        final_gap_mm=model.gap_mm().tolist(),final_tension_N=model.tension,
        max_tension_N=max(r['tension_N'] for r in rows),
        completed_stitches=len(deployed),thread_present_from_first_frame=True,
        progressive_thread_feed=True,thread_surface_display_offset_mm=.15,
        ik=ik,elapsed_s=time.time()-start)
    if natural:
        report.update(natural_profile=True,grasp_correction=metadata['grasp_correction'],holder_clearance=metadata['holder_clearance'],left_holder_clearance=metadata['left_holder_clearance'])
    (output/'manifest.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print('COMPUTED',report['final_gap_mm'],'tension',model.tension,flush=True)
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--dt',type=float,default=.025)
    parser.add_argument('--no-pull',action='store_true')
    parser.add_argument('--output',type=Path,default=ROOT/'output/soft_suture')
    parser.add_argument('--natural',action='store_true')
    args=parser.parse_args();assert abs(round(.05/args.dt)*args.dt-.05)<1e-10
    run(args.output,args.dt,not args.no_pull,args.natural)
