"""Package the minimal online demo and stage complete implementation/evidence."""
import ast,hashlib,json,shutil,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];REPO=ROOT/'github_upload/dual-arm-suturing-simulation';D=ROOT/'output/delivery/matlab_suture_3x'
for name in ['matlab_online_validation.json','material_response_validation.json']:
 assert json.loads((D/name).read_text())['result']=='PASS'
readme='''# MATLAB 双臂连续缝合在线仿真

我将双臂五针、3 mm 针距的缝合做成了 MATLAB 控制的在线仿真：MATLAB 逐步发送双臂关节和针的目标，CoppeliaSim 执行动作，Python 物理求解器根据本次指令计算组织变形与缝线。场景没有预存帧播放器，也没有嵌入垫子或缝线的运动结果。

## 运行环境

Windows；MATLAB R2022a（已实测）；CoppeliaSim Edu 4.10；Python 3.10 或更高版本（已实测 3.13），安装 NumPy 与 SciPy。MATLAB 不需要配置 `pyenv`，Python 作为独立的本机求解进程运行。

## 第一次运行

1. 解压到可写目录，建议英文路径，例如 `D:/suture_demo`。
2. 在终端用自己的 Python 安装依赖：`python -m pip install -r physics/requirements.txt`。
3. 打开 CoppeliaSim，保存自己的工作并停止当前仿真。保持程序打开。
4. 在 MATLAB 中把当前文件夹切换到本包目录，运行：

```matlab
main_suture('C:/你的Python路径/python.exe', 23000, 35)
```

Python 已加入 PATH 时可直接运行 `main_suture`。23000 是 CoppeliaSim 默认远程端口；35 是组织网格弹簧刚度（N/m），修改它会在本次运行中重新计算组织响应。

MATLAB 会加载本包 `scene.ttt` 并执行 1170 条控制指令。结束后场景暂停，生成 `matlab_online_validation.json` 和 `online_metrics.json`。重跑前先在 CoppeliaSim 点击停止。Ctrl+C 中断时会释放在线求解进程并暂停仿真。

## 文件与职责

- `main_suture.m`：MATLAB 主循环，发送控制指令、驱动物理步进并核对实际关节与碰撞。
- `matlab/`：三份远程接口 MATLAB 文件及随包提供的 JeroMQ；运行时不需要下载接口库。
- `commands.json`：经检查的双臂与针目标轨迹，包含阶段和针次；不含组织或缝线结果。
- `physics/`：三份最小在线求解源码及依赖清单。每条指令内部进行六次力学积分。
- `scene.ttt`、`scene_code/`：场景与其中三份实际脚本的源码。
- `reference_video.mp4`：此前录制的三倍速参考视频；仿真程序不读取该视频。

1170 个 0.05 s 控制步对应约 58.5 s 仿真时间，即原动作的约三倍速度。实际墙钟运行耗时取决于机器求解和渲染性能。参考视频使用此前细步长计算结果加速播放；此版本按加速后的指令实时求解，因此受力数值和局部变形可以略有差异。

组织采用非线性黏弹性点阵，缝线采用有余量下垂和表面支撑的准静态缆索。机械臂为位置轨迹控制，针与夹爪为几何夹持约束；尚无机器人力反馈、真实切割、线组织摩擦、自碰撞或打结。不要将该原型当作经过材料实验标定的手术模型。

完整实现与证据：https://github.com/Krist21a/dual-arm-suturing-simulation
'''
(D/'README.md').write_text(readme,encoding='utf-8')
files=[p for p in D.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ['.png','.pyc'] and p.name not in ['online_metrics.json','SHA256SUMS.json']]
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
hashes={p.relative_to(D).as_posix():digest(p) for p in files};(D/'SHA256SUMS.json').write_text(json.dumps(hashes,indent=2))
archive=ROOT/'output/packages/双臂缝合_三倍速_MATLAB在线仿真_最小包.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for p in files+[D/'SHA256SUMS.json']:z.write(p,'matlab_suture_3x/'+p.relative_to(D).as_posix())
with zipfile.ZipFile(archive) as z:
 assert z.testzip() is None
 for name,h in hashes.items():assert hashlib.sha256(z.read('matlab_suture_3x/'+name)).hexdigest()==h
report=dict(result='PASS',archive=archive.name,bytes=archive.stat().st_size,sha256=digest(archive),files=len(files)+1,mode='MATLAB commands and online tissue/thread mechanics')
(D/'package_validation.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
copied=[]
def stage(p,relative=None):
 relative=relative or p.relative_to(ROOT).as_posix();target=REPO/relative;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target);copied.append(relative)
for p in (ROOT/'work/soft_suture').rglob('*'):
 if p.is_file() and '__pycache__' not in p.parts and p.suffix in ['.py','.lua','.md','.txt','.json','.m','.jar']:stage(p)
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
 if p.is_file() and '__pycache__' not in p.parts:stage(p,'demo/matlab_suture_3x/'+p.relative_to(D).as_posix())
stage(archive,'docs/downloads/matlab_suture_3x_minimal.zip')
stage(ROOT/'output/delivery/matlab_online_run.log','output/reports/matlab_online_run.log.txt')
stage(D/'package_validation.json','output/reports/matlab_online_package_validation.json')
stage(D/'matlab_online_validation.json','output/reports/matlab_online_validation.json')
stage(D/'material_response_validation.json','output/reports/online_material_response_validation.json')
entry=(D/'main_suture.m').read_text();entry=entry.replace("root=fileparts(mfilename('fullpath'));", "root=fullfile(fileparts(mfilename('fullpath')),'demo','matlab_suture_3x');")
(REPO/'main_suture.m').write_text(entry,encoding='utf-8');copied.append('main_suture.m')
stage(ROOT/'work/soft_suture/GITHUB_README.md','README.md')
(REPO/'docs/RUNNING.md').write_text('# 当前运行入口\n\n当前版本由 MATLAB 逐步控制并在线求解组织与缝线。请参阅 [完整运行说明](../demo/matlab_suture_3x/README.md)。\n\n仓库根目录运行 `main_suture`；此前运动学版本的入口为 `main_continuous_suture`，仅用于历史对照。\n',encoding='utf-8');copied.append('docs/RUNNING.md')
assets={name:digest(REPO/name) for name in sorted(copied)}
(REPO/'output/reports/matlab_online_published_assets.json').write_text(json.dumps(assets,indent=2))
for p in (REPO/'work/soft_suture').glob('*.py'):ast.parse(p.read_text(encoding='utf-8'))
print(json.dumps(report,ensure_ascii=False));print('Staged',len(copied),'files')
