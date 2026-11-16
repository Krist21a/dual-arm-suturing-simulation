"""Run the mechanics afresh in lockstep with CoppeliaSim (not stored replay).

Run after opening dual_DVRK_ARM_soft_suture.ttt. Both robot poses and computed
tissue/cable states are sent per simulation step. Material parameters are read
from Parameters; changing them changes the forces and deformation.
"""
import argparse,os,time,json
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from mechanics import SoftSuture,Parameters
from trajectory import build_actions,ROOT

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--frames',type=int)
    ap.add_argument('--stiffness',type=float,default=35.)
    ap.add_argument('--natural',action='store_true')
    ap.add_argument('--port',type=int,default=int(os.getenv('COPPELIA_ZMQ_PORT','23002')))
    args=ap.parse_args()
    actions,_=build_actions(json.loads((ROOT/'work/continuous_suture/continuous_plan.json').read_text()))
    corrected=ROOT/'output/soft_suture_continuous'
    if (corrected/'grasp_validation.json').exists():
        metadata=json.loads((corrected/'manifest.json').read_text())
        frames=np.fromfile(corrected/'frames.bin',dtype='<f4').reshape(-1,metadata['state_width'])
        assert len(frames)==len(actions)
        for action,row in zip(actions,frames):
            assert np.max(abs(np.array(action['needle'])-row[22:34]))<1e-6
            action['q']=row[:22].reshape(2,11).astype(float).tolist()
    model=SoftSuture(Parameters(dt=.025,spring_stiffness=args.stiffness,strain_stiffening=4. if args.natural else 0.,edge_viscosity=.15 if args.natural else 0.,distributed_contact=args.natural))
    client=RemoteAPIClient(port=args.port);s=client.require('sim')
    assert s.getSimulationState()==s.simulation_stopped,'Stop simulation before live run'
    assert 'soft_suture' in s.getStringParam(s.stringparam_scene_name)
    driver=s.getObject('/SoftSuturePlayback');s.setStepping(True);s.startSimulation()
    # Start callback runs on the first explicit step. It will show replay frame 1.
    s.step();s.callScriptFunction('setLiveMode',driver,True)
    deployed=set();rows=[];prev_tip=prev_tail=None
    try:
        for i,a in enumerate(actions[:args.frames]):
            T=np.array(a['needle']).reshape(3,4)
            tip=T[:,:3]@np.array([0,.006,-.0115])+T[:,3]
            tail=T[:,:3]@np.array([0,.005708948958544,.01124811054846])+T[:,3]
            if not model.route:model.start_strand(tail)
            if a['phase']=='left_handoff' and a['stitch'] not in deployed:
                assert a['stitch'] in model.punctured,'No puncture: refusing thread deployment'
                model.deploy(a['stitch'],tail);deployed.add(a['stitch'])
            for fraction in [.5,1.]:
                t=tip if prev_tip is None else prev_tip*(1-fraction)+tip*fraction
                e=tail if prev_tail is None else prev_tail*(1-fraction)+tail*fraction
                r=model.step(tip=t,puncture_key=a['stitch'],cable_end=e,
                             feed=a['phase'] not in ['tighten','hold_tension'])
            prev_tip=tip;prev_tail=tail
            visible=model.display_cable(a['stitch'],a['theta'],a['phase'],len(deployed))
            if args.natural:
                from natural_strand import strand
                visible=strand(model,visible)
            cable=np.zeros((512 if args.natural else 22,3));n=len(visible);cable[:n]=visible
            state=np.r_[np.array(a['q']).ravel(),a['needle'],model.x[:model.n_tissue].ravel(),n,cable.ravel()]
            s.callScriptFunction('applyLiveState',driver,state.tolist());s.step()
            r.update(frame=i,phase=a['phase']);rows.append(r)
            if i%200==0:print(i,a['phase'],r['gap_mm'],flush=True)
    finally:
        s.pauseSimulation();s.setStepping(False)
    out=ROOT/'output/soft_suture/live_validation.json'
    out.write_text(json.dumps(dict(frames=len(rows),final=rows[-1],parameters=vars(model.p)),indent=2))
    print('Live mechanics complete',len(rows),flush=True)

if __name__=='__main__':main()
