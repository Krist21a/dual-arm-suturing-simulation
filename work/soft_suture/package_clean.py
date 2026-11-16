"""Bundle the corrected holder, continuous strand and standalone scene."""
import json,hashlib,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];DATA=ROOT/'output/soft_suture_continuous'
for relative in ['thread_validation.json','actual_grasp_validation.json','scene_validation.json','video/validation.json']:
    assert json.loads((DATA/relative).read_text())['result']=='PASS',relative
sources=['mechanics.py','kinematics.py','trajectory.py','simulate.py','live.py',
         'retarget_grasp.py','apply_grasp_clearance.py','build_scene.py','playback.lua',
         'check_centered_grasp.py','validate_continuous_thread.py','record_clean_video.py',
         'record_video.py','README.md','requirements.txt','package_clean.py','grasp_calibration.json']
files=[ROOT/'work/soft_suture'/name for name in sources]
files += [ROOT/'work/continuous_suture/continuous_plan.json',ROOT/'work/suture_review/scene_live.json',
          ROOT/'output/scenes/dual_DVRK_ARM_continuous_suture.ttt',
          ROOT/'output/scenes/dual_DVRK_ARM_soft_suture_continuous_thread.ttt']
files += [DATA/name for name in ['frames.bin','manifest.json','metrics.json','thread_validation.json',
          'grasp_validation.json','actual_grasp_validation.json','scene_validation.json',
          'clearance_validation.json','clearance_validation_L.json',
          'right_initial.png','left_extracted.png','right_regrasp.png','video/validation.json',
          'video/capture_validation.json','video/initial.png','video/closed.png']]
target=ROOT/'output/packages/双臂缝合_夹持修正与连续缝线_2026-11-16.zip'
manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
readme='''# 双臂连续缝合：夹持修正与连续缝线

我在双臂五针、3 mm 针距的普通连续缝合演示上，加入了组织变形和缝线张力传递。这个版本将针夹在夹爪内侧约 3 mm 处，并保持缝线从第一帧起持续存在、随针运动；最终拉线使切口靠拢。

直接在 CoppeliaSim Edu 4.10 打开 `output/scenes/dual_DVRK_ARM_soft_suture_continuous_thread.ttt` 并运行即可。场景内嵌了计算结果，不需要先安装 Python。若要重新求解、调整参数或录制，见 [代码说明](work/soft_suture/README.md)。

| 右臂持针 | 左臂持针 |
| --- | --- |
| ![](output/soft_suture_continuous/right_initial.png) | ![](output/soft_suture_continuous/left_extracted.png) |

录像采用全景上方中央并排放置左右目画面，无说明文字。视频单独提供，压缩包中保留视频校验报告及首尾预览。

这是简化力学原型：组织由隐式弹性点阵求解，缝线为准静态无质量缆索；机械臂按预定轨迹运动，夹持关系是几何约束。尚未实现真实切割、线组织摩擦、夹持接触力、打结或机器人力控。
'''
with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in files:z.write(p,'dual-arm-suture/'+p.relative_to(ROOT).as_posix())
    z.writestr('dual-arm-suture/README.md',readme)
    z.writestr('dual-arm-suture/PACKAGE_MANIFEST.json',json.dumps(manifest,indent=2))
with zipfile.ZipFile(target) as z:
    assert z.testzip() is None
    for name,digest in manifest.items():assert hashlib.sha256(z.read('dual-arm-suture/'+name)).hexdigest()==digest
report=dict(result='PASS',archive=str(target),files=len(files)+2,bytes=target.stat().st_size,sha256=hashlib.sha256(target.read_bytes()).hexdigest())
(ROOT/'output/packages/夹持修正与连续缝线_打包校验.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(report)
