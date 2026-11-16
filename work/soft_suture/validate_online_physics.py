"""Show that online tissue states respond to a changed material parameter."""
import json
from pathlib import Path
import numpy as np
from physics_worker import Solver
ROOT=Path(__file__).resolve().parents[2];d=ROOT/'output/delivery/matlab_suture_3x'
commands=json.loads((d/'commands.json').read_text())[:80]
solvers=[Solver(35.),Solver(70.)];maximum=0.
for a in commands:
 for solver in solvers:solver.step(a)
 maximum=max(maximum,float(np.max(np.linalg.norm(solvers[0].model.x[:540]-solvers[1].model.x[:540],axis=1))))
assert maximum>1e-6,'Changing material stiffness did not affect the online result'
report=dict(result='PASS',commands=len(commands),stiffness_values=[35,70],maximum_tissue_difference_mm=maximum*1000,precomputed_deformation_files_used=False)
(d/'material_response_validation.json').write_text(json.dumps(report,indent=2));print(report)
