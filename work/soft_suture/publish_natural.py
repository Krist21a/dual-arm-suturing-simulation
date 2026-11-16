"""Build the minimal handoff archive and stage full reproducible GitHub assets."""
import hashlib,json,shutil,zipfile,re,ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];REPO=ROOT/'github_upload/dual-arm-suturing-simulation'
D=ROOT/'output/delivery/natural_3x_demo'
assert json.loads((D/'standalone_validation.json').read_text())['result']=='PASS'
shutil.copy2(ROOT/'output/videos/soft_suture_natural_3x_clean.mp4',D/'demo.mp4')
(D/'README.md').write_text('''# 双臂缝合三倍速演示

我将五针、3 mm 针距的双臂连续缝合，加入垫子受力变形、连续可见缝线及夹爪内侧持针，并以三倍速度播放。视频为 59.15 秒、1920×1440，无文字，全景上方中央为左右目画面。

## 最快观看

直接播放 `demo.mp4`。

## 运行场景

1. 安装 CoppeliaSim Edu 4.10，建议将本包解压到英文路径，例如 `D:/suture_demo`。
2. 打开 `scene.ttt`，点击运行按钮，完整五针结束后自动暂停。
3. 要重播，先点击停止，再点击运行。

无需 Python、MATLAB、网络或额外数据文件。场景内已嵌入全部播放状态和脚本。

## 最少代码

`code/main.lua` 处理关节目标，`code/controller.lua` 控制双臂、夹爪与针并检查碰撞，`code/playback.lua` 更新组织与缝线、播放五针过程。三份源码直接从本包场景导出，供阅读；运行时无需另行导入。

这是已求解结果的独立演示场景。更改材料后重新计算需要完整仓库：https://github.com/Krist21a/dual-arm-suturing-simulation 。组织采用非线性黏弹性点阵，缝线采用带余量下垂表现的准静态缆索；不包含真实切割、摩擦、线自碰撞、打结或夹持力控制。

`standalone_validation.json` 记录本场景复制后独立运行的检查结果；`SHA256SUMS.json` 用于核对文件完整性。
''',encoding='utf-8')
files=sorted(p for p in D.rglob('*') if p.is_file() and p.name!='SHA256SUMS.json')
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
hashes={p.relative_to(D).as_posix():digest(p) for p in files}
(D/'SHA256SUMS.json').write_text(json.dumps(hashes,indent=2))
archive=ROOT/'output/packages/双臂缝合_三倍速新版_最小演示包.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in files+[D/'SHA256SUMS.json']:z.write(p,'natural_3x_demo/'+p.relative_to(D).as_posix())
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 for name,h in hashes.items():assert hashlib.sha256(z.read('natural_3x_demo/'+name)).hexdigest()==h
report=dict(result='PASS',archive=archive.name,bytes=archive.stat().st_size,sha256=digest(archive),files=len(files)+1,external_runtime='CoppeliaSim Edu 4.10',external_python_required=False)
(ROOT/'output/packages/三倍速最小演示包_校验.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
copied=[]
def stage(src,relative=None):
 relative=relative or src.relative_to(ROOT).as_posix();target=REPO/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(src,target);copied.append(relative)
# Complete current implementation, including investigation/validation helpers.
for p in (ROOT/'work/soft_suture').iterdir():
 if p.suffix in ['.py','.lua','.md','.txt','.json']:stage(p)
for folder in ['soft_suture','soft_suture_continuous','soft_suture_natural','soft_suture_natural_3x']:
 base=ROOT/'output'/folder
 for p in base.iterdir():
  if p.suffix in ['.json','.png'] or p.name=='frames.bin':stage(p)
 for sub in ['video','control_no_pull','control_half_dt']:
  if (base/sub).exists():
   for p in (base/sub).iterdir():
    if p.suffix in ['.json','.png'] and p.name!='progress.json':stage(p)
for name in ['dual_DVRK_ARM_soft_suture.ttt','dual_DVRK_ARM_soft_suture_continuous_thread.ttt','dual_DVRK_ARM_soft_suture_natural_3x.ttt']:stage(ROOT/'output/scenes'/name)
stage(ROOT/'work/suture_review/scene_live.json')
stage(ROOT/'output/videos/soft_suture_natural_3x_clean.mp4','docs/media/soft_suture_natural_3x_clean.mp4')
stage(ROOT/'output/soft_suture_natural_3x/video/initial.png','docs/media/natural_3x_preview.png')
for name in ['right_initial.png','left_extracted.png']:stage(ROOT/'output/soft_suture_continuous'/name,'docs/media/'+name)
for p in D.rglob('*'):
 if p.is_file() and p.suffix not in ['.ttt','.mp4']:stage(p,'demo/natural_3x/'+p.relative_to(D).as_posix())
stage(archive,'docs/downloads/natural_3x_demo.zip')
stage(ROOT/'output/packages/三倍速最小演示包_校验.json','output/reports/natural_3x_package_validation.json')
assets={name:digest(REPO/name) for name in sorted(copied)}
(REPO/'output/reports/natural_3x_published_assets.json').write_text(json.dumps(assets,indent=2))
for p in (REPO/'work/soft_suture').glob('*.py'):ast.parse(p.read_text(encoding='utf-8'))
print(json.dumps(report,ensure_ascii=False));print('Staged',len(copied),'files')
