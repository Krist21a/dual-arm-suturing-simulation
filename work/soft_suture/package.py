"""Package the independent mechanics prototype without recordings or caches."""
import json,hashlib,zipfile
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
out=ROOT/'output/soft_suture'
mechanics=json.loads((out/'mechanics_validation.json').read_text())
scene=json.loads((out/'scene_validation.json').read_text())
live=json.loads((out/'live_validation.json').read_text())
assert mechanics['result']=='PASS' and scene['result']=='PASS'
assert live['frames']==3506
error=float(np.max(abs(np.array(live['final']['gap_mm'])-mechanics['final_gap_mm'])))
assert error<.001
live.update(result='PASS',offline_final_gap_difference_mm=error)
(out/'live_validation.json').write_text(json.dumps(live,indent=2))
files=[]
for p in (ROOT/'work/soft_suture').iterdir():
    if p.suffix in ['.py','.lua','.md','.txt'] and p.name not in ['probe.py','package.py']:
        files.append(p)
files.extend([ROOT/'work/continuous_suture/continuous_plan.json',
              ROOT/'output/scenes/dual_DVRK_ARM_continuous_suture.ttt',
              ROOT/'output/scenes/dual_DVRK_ARM_soft_suture.ttt'])
files.extend(p for p in out.iterdir() if p.suffix in ['.json','.png','.bin'])
files.extend(out/d/'manifest.json' for d in ['control_no_pull','control_half_dt'])
target=ROOT/'output/packages/双臂缝合_软组织与张力原型_2026-11-16.zip'
manifest={str(p.relative_to(ROOT)).replace('\\','/'):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in files:z.write(p,'soft-suture-prototype/'+p.relative_to(ROOT).as_posix())
    readme=(ROOT/'work/soft_suture/README.md').read_text(encoding='utf-8')
    z.writestr('soft-suture-prototype/README.md',readme)
    z.writestr('soft-suture-prototype/PACKAGE_MANIFEST.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(target) as z:
    assert z.testzip() is None
    for name,digest in manifest.items():
        assert hashlib.sha256(z.read('soft-suture-prototype/'+name)).hexdigest()==digest
report=dict(result='PASS',archive=str(target),bytes=target.stat().st_size,
            sha256=hashlib.sha256(target.read_bytes()).hexdigest(),files=len(files)+2,
            live_offline_gap_difference_mm=error)
(ROOT/'output/packages/软组织原型_打包校验.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report,indent=2))
