"""Mechanical counterfactuals, timestep convergence, mesh validity and recovery."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import json
from pathlib import Path
import numpy as np
from mechanics import SoftSuture,Parameters
ROOT=Path(__file__).resolve().parents[2]

def main():
    out=ROOT/'output/soft_suture'
    main=json.loads((out/'manifest.json').read_text())
    control=json.loads((out/'control_no_pull/manifest.json').read_text())
    fine=json.loads((out/'control_half_dt/manifest.json').read_text())
    s=SoftSuture();data=np.fromfile(out/'frames.bin',dtype='<f4').reshape(-1,main['state_width'])
    minJ=1.;maxJ=1.
    for row in data:
        pos=row[34:34+s.n_tissue*3].reshape(-1,3).astype(float)
        z=pos[s.tets]
        J=np.einsum('ij,ij->i',z[:,1]-z[:,0],np.cross(z[:,2]-z[:,0],z[:,3]-z[:,0]))/6/s.volume0
        minJ=min(minJ,float(J.min()));maxJ=max(maxJ,float(J.max()))
        assert np.isfinite(row).all() and (J>0).all(),'Inverted/nonfinite tissue element'
        assert np.max(abs(pos[s.fixed]-s.rest[s.fixed]))<1e-7,'Fixed boundary drift'
    assert main['completed_stitches']==5 and len(main['puncture_events'])==5
    assert all(e['force_N']>=main['parameters']['puncture_force'] and e['indentation_mm']>0 for e in main['puncture_events'])
    assert max(main['final_gap_mm'])<.6 and min(main['final_gap_mm'])>=.299
    assert min(control['final_gap_mm'])>3.8 and control['final_tension_N']<1e-6
    gap_error=float(np.max(abs(np.array(main['final_gap_mm'])-fine['final_gap_mm'])))
    tension_error=abs(main['final_tension_N']-fine['final_tension_N'])/fine['final_tension_N']
    assert gap_error<.08 and tension_error<.03,'Timestep comparison outside tolerance'
    # A nonpuncturing indenter must deform the pad and it must recover on release.
    s=SoftSuture(Parameters(dt=.025,puncture_force=100.))
    for _ in range(20):s.step()
    baseline=s.x.copy();peak=0.
    for k in range(40):
        s.step(tip=[-.022,.345,.592-.004*k/39],puncture_key=1)
        peak=max(peak,float(np.max(np.linalg.norm(s.x-baseline,axis=1))*1000))
    assert not s.punctured and peak>.5
    for _ in range(60):s.step()
    residual=float(np.max(np.linalg.norm(s.x-baseline,axis=1))*1000)
    assert residual<.1,'Elastic recovery failed'
    report=dict(result='PASS',frames=len(data),punctures=5,
        final_gap_mm=main['final_gap_mm'],no_pull_gap_mm=control['final_gap_mm'],
        final_tension_N=main['final_tension_N'],minimum_element_volume_ratio=minJ,
        maximum_element_volume_ratio=maxJ,inverted_elements=0,
        half_timestep_max_gap_difference_mm=gap_error,
        half_timestep_tension_relative_difference=tension_error,
        blocked_puncture_peak_displacement_mm=peak,recovery_residual_mm=residual,
        scope='Reduced-order numerical model; parameters are not experimentally calibrated.')
    (out/'mechanics_validation.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
