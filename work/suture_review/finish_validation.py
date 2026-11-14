"""Check action geometry from recorded live measurements and write delivery notes."""
import sys,json,math
from pathlib import Path
ROOT=Path(r'D:\仿真环境');sys.path.insert(0,str(ROOT/'work/python_deps'))
import numpy as np
from scipy.spatial.transform import Rotation
p=json.loads((ROOT/'work/suture_review/wrist_plan.json').read_text())
reportPath=ROOT/'output/reports/suture_final_validation.json'
r=json.loads(reportPath.read_text(encoding='utf-8'))
rows=[json.loads(x) for x in (ROOT/'output/reports/suture_live_frames.jsonl').read_text().splitlines()]
d=json.loads((ROOT/'work/suture_review/scene_live.json').read_text())
cl=np.array([0,.005854474479272,-.000125944725770])
center=(np.array(p['needleEntry'])@np.r_[cl,1])[:3]
ins=np.array([x['tip'] for x in rows if x['phase']=='right_insert'])
v=ins-center;angles=np.unwrap(np.arctan2(v[:,0],v[:,2]))
radius=np.linalg.norm(np.array([0,.006,-.0115])-cl)
radial=np.abs(np.linalg.norm(v[:,[0,2]],axis=1)-radius).max()
measured=math.degrees(angles[-1]-angles[0])
assert radial<.0001 and abs(measured-math.degrees(p['sweep']))<.1
first=rows[0];frames=np.array(first['tcps']).reshape(2,3,4)
initialAngle=math.degrees(Rotation.from_matrix(frames[0,:,:3]@frames[1,:,:3].T).magnitude())
assert initialAngle<.12
assert np.linalg.norm(np.array(first['tip'])-np.array(p['entryPoint'])-[0,0,.04])<.0001
base=np.array(next(x for x in d if x['alias']=='/500____[1]/UR5')['pose'][:3])
assert np.linalg.norm(base-p['entryPoint'])<np.linalg.norm(base-p['exitPoint'])
ex=next(x for x in reversed(rows) if x['phase']=='left_extract')
T=np.array(ex['needle']).reshape(3,4)
mesh=np.array(next(x for x in d if x['alias'].endswith('/Needle'))['mesh'][0]).reshape(-1,3)
clearance=float((mesh@T[:,:3].T+T[:,3])[:,2].min()-.5905)
assert clearance>0
finalQ=np.array(rows[-1]['joints'])
assert rows[-1]['phase']=='complete' and np.max(abs(finalQ[1,9:]))<1e-9 and np.max(abs(finalQ[0,9:]-.35))<1e-9
r.update(measured_insertion_rotation_deg=measured,arc_radial_error_mm=float(radial*1000),initial_tcp_orientation_difference_deg=initialAngle,initial_tip_above_entry_mm=40,entry_on_holding_arm_base_side=True,extracted_needle_min_tissue_clearance_mm=clearance*1000,final_state='Right jaws closed, left jaws open; paused with needle attached to right TCP.')
reportPath.write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
notes=f'''单个间断缝合：实际联调验证通过

方向：针在右臂所属的近侧垫子上方就位，入针 x=-0.022 m，出针 x=-0.038 m。
双目图像中表现为近侧（画面左侧）入针、对侧（画面右侧）出针。
初始两 TCP 世界朝向差 {initialAngle:.5f}°，针尖位于入针点正上方 40 mm。
两片夹爪保持原场景几何、关节轴、符号和控制命令逻辑。

实际检查 {r['checked_frames']} 帧，双臂之间及器械与训练垫/桌面网格碰撞为 0。
穿针实测转角 {measured:.5f}°，圆弧半径误差最大 {radial*1000:.5f} mm。
入针点误差 {r['entry_error_mm']:.5f} mm，出针点误差 {r['exit_error_mm']:.5f} mm。
两次交接位置/姿态连续，详见 suture_handoff_validation.tsv。
左/右 joint7 最大值 {r['joint7_max_mm'][0]:.3f}/{r['joint7_max_mm'][1]:.3f} mm。
左/右 joint9 绝对值最大 {r['joint9_abs_max_deg'][0]:.3f}/{r['joint9_abs_max_deg'][1]:.3f}°。
近端关节相对工作姿态最大变化：左 {r['proximal_max_delta_deg']['L']:.3f}°，右 {r['proximal_max_delta_deg']['R']:.3f}°。
训练垫与双目相机位姿变化为 0；双目完整包含训练垫与针操作区域。
训练垫保留原桌面位置及约 0.15 mm 的原模型间隙，没有抬高或移位。
完成后暂停于右臂持针、左臂松开撤离的状态。

修改：main_suture.m；output/scenes/dual_DVRK_ARM_suture.ttt；work/scene_inspection/record_suture_stereo.lua。
新增：RunSutureWristCycle.m；work/suture_review/wrist_plan.json 及规划/网格审查/录制/验证脚本。
更新：output/reports/suture_* 验证报告；新增 output/videos/single_interrupted_suture_stereo.mp4 与关键帧预览。
原 main_suture.m 和场景保存在 work/suture_review/baseline。
运行依赖：主程序、RunSutureWristCycle.m、work/suture_review/wrist_plan.json、
output/reports/suture_plan_validation.json，以及含 SutureController 的最终场景。

范围：这是针运动与交接的运动学演示。缝线为可视化曲线，不含组织变形、线张力或打结力学。
碰撞报告覆盖两臂之间、器械与训练垫/桌面。
'''
(ROOT/'output/reports/suture_validation_summary.txt').write_text(notes,encoding='utf-8')
print('Action geometry PASS',measured,'deg',radial*1000,'mm radial error',clearance*1000,'mm extracted clearance')
