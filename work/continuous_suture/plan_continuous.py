"""Extend the accepted stitch without changing its shoulder/elbow branch."""
import sys,json,copy,argparse
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'work/suture_review'))
from plan_roll_motion import roll_solve,interpolate,np,cal
from suture_wrist_kinematics import fk

def build(count=5,spacing=.003):
    p=json.loads((ROOT/'work/suture_review/wrist_plan.json').read_text())
    assert count>=1 and spacing>0 and (count-1)*spacing<.0225
    for a in cal:cal[a][:]=p['calibration'][a]
    state={a:np.array(p['initial'][a]) for a in cal}
    refs={a:np.array(p['dock'][a]) for a in cal}
    phases=[]
    def add(name,a,targets,stitch,angles=None,known=None):
        trajectory=[]
        for k,t in enumerate(targets):
            state[a]=np.array(known[k]) if known is not None else roll_solve(a,t,state[a],refs[a])
            trajectory.append(state[a].tolist())
        phases.append(dict(name=name,arm=a,stitch=stitch,targets=[t.tolist() for t in targets],q=trajectory,needleAngles=angles or []))
        print(stitch,name,len(trajectory),flush=True)
    for i in range(count):
        offset=np.array([0,i*spacing,0])
        if i:
            # Clear the pad, restore entry orientation while still held by R,
            # then restore the receiving arm's standby pose. No needle reset.
            target=cal['R']@fk(np.array(p['initial']['R']),'R');target[:3,3]+=offset
            start=cal['R']@fk(state['R'],'R')
            add('next_stitch_right','R',list(interpolate(start,target,80)),i+1)
            target=cal['L']@fk(np.array(p['initial']['L']),'L');target[:3,3]+=offset
            add('next_stitch_left','L',list(interpolate(cal['L']@fk(state['L'],'L'),target,40)),i+1)
        for ph in p['phases']:
            targets=np.array(ph['targets']);targets[:,:3,3]+=offset
            add(ph['name'],ph['arm'],targets,i+1,ph['needleAngles'],ph['q'] if i==0 else None)
    out=copy.deepcopy(p);out.update(mode='continuous-running',stitch_count=count,spacing_m=spacing,phases=phases,
        stitches=[dict(index=i+1,entryPoint=(np.array(p['entryPoint'])+[0,i*spacing,0]).tolist(),exitPoint=(np.array(p['exitPoint'])+[0,i*spacing,0]).tolist(),arcCenter=(np.array(p['arcCenter'])+[0,i*spacing,0]).tolist()) for i in range(count)])
    (ROOT/'work/continuous_suture/continuous_plan.json').write_text(json.dumps(out))
    return out
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--count',type=int,default=5);parser.add_argument('--spacing-mm',type=float,default=3)
    args=parser.parse_args();build(args.count,args.spacing_mm/1000)
