# 双臂连续缝合仿真 / Dual-arm continuous suturing

默认演示现已更新为 **5 针、针距 3 mm 的普通连续缝合**。右臂圆弧入针、左臂接针拔出、回递给右臂，再沿切口前进至下一针。整段使用同一根针和连续保留的缝线，不在针间重设针位，不断线、不额外打结。

## 演示视频

[观看全景＋同步双目视频（MP4，2 分 47.9 秒）](docs/media/continuous_suture_5x3mm_overview_stereo.mp4) · [直接下载](https://github.com/Krist21a/dual-arm-suturing-simulation/raw/refs/heads/main/docs/media/continuous_suture_5x3mm_overview_stereo.mp4)

[纯固定全景原始录像](docs/media/continuous_suture_5x3mm_overview.mp4)

![全景上方中央并排显示左右双目相机画面](docs/media/continuous_suture_stereo_inset.png)

两个双目小画面使用原相机位姿，按原比例显示，3,346 个动作帧与全景逐帧核对一致。视频为 1920×1440、20 fps，无变速。

## 快速运行

验证环境为 CoppeliaSim Edu 4.10、Python 3.13；MATLAB 为可选入口。建议把仓库放在不含中文或空格的路径（例如 `D:/dual-arm-suturing-simulation`），避免部分 CoppeliaSim 文件接口的路径编码问题。

```bash
python -m pip install -r requirements.txt
```

1. 在 CoppeliaSim 打开 **`output/scenes/dual_DVRK_ARM_continuous_suture.ttt`**，保持仿真停止。
2. 指定仿真器的 ZeroMQ 端口：程序默认 `23002`。若 CoppeliaSim 使用默认端口 `23000`，设置环境变量 `COPPELIA_ZMQ_PORT=23000`。
3. 在仓库根目录执行：

```bash
python work/continuous_suture/run_native.py
```

Windows PowerShell 设置端口示例：`$env:COPPELIA_ZMQ_PORT="23000"`。

也可在 MATLAB 中运行 **`main_suture`** 或 `main_continuous_suture`。MATLAB 入口调用同一个 Python 执行器；用 `setenv('SUTURE_PYTHON','你的 python.exe 完整路径')` 选择已安装依赖的 Python，未指定时使用 MATLAB 的 Python 配置或 PATH 中的 `python`。不要求将 Python 加载进 MATLAB 进程。

程序校验轨迹 SHA-256 后，在一次仿真会话中完成全部五针，最后暂停在右臂持针、右爪闭合、左爪张开撤离的状态。再次运行前先停止仿真。场景已准备好，正常演示不需要运行构建或规划脚本。

## 录制与参数

纯全景录制：`python work/continuous_suture/run_native.py --record`，完成后运行 `python work/continuous_suture/export_video.py`。

双目补录：先运行 `python work/continuous_suture/prepare_stereo.py`，再运行 `python work/continuous_suture/run_stereo.py --record`，最后运行 `python work/continuous_suture/export_stereo_inset.py`。合成前会核对全景与双目两次运行的全部动作帧，帧位姿不一致时拒绝导出。完成后可运行 `python work/continuous_suture/finish_stereo.py` 移除临时录制对象。

原始帧默认保存在 `work/continuous_suture/raw_frames/` 和 `stereo_frames/`，不纳入 Git。全景约需 28 GB，双目约需 21 GB，程序额外要求保留 10 GiB 空间。可用 `SUTURE_FRAMES_DIR`、`SUTURE_STEREO_FRAMES_DIR` 指定其他目录；`SUTURE_STAGE_DIR` 指定场景临时目录。

`plan_continuous.py --count 5 --spacing-mm 3` 用于重新规划。修改参数后应在场景副本中运行 `prepare.py` 和 `validate.py`，再完整实跑检查。已有场景及报告对应的是本仓库交付的 **5×3 mm** 方案，不能直接用于声明其他参数已验证。

## 已验证结果

- 完整五针实跑检查 **3,346 帧**，检测到的双臂、桌面、训练垫及非相邻臂杆碰撞为 **0**。
- 10 次交接位姿连续；4 次针间过渡不重置针或缝线。
- 每帧仿真时间实际推进 0.05 秒，针平面最大偏离竖直约 **0.00104°**。
- 左右大臂相对实际初始姿态最大变化约 **2.35° / 3.19°**。
- joint7、joint8、joint9 均在允许范围内；训练垫和原双目相机位姿不变。
- 两片夹爪保留原几何、关节轴及开合配对：`q10=yaw+grip`、`q11=-yaw+grip`。

见 [验证摘要](output/reports/continuous_suture_validation_summary.txt)、[详细报告](output/reports/continuous_suture_final_validation.json)、[双目同步报告](output/reports/continuous_suture_stereo_inset_validation.json)。发布版入口也已在独立的无界面仿真实例中重新完成全部 3,346 帧检查，见 [发布版实跑报告](output/reports/continuous_suture_github_runtime_validation.json)。

无需仿真器的轨迹回归测试：`python -m unittest discover -s tests -v`，检查轨迹校验值、针距、全轨迹运动学、关节限位及竖直圆弧入针。

## 文件与模型范围

- `main_suture.m`：默认多针入口。
- `work/continuous_suture/`：多针规划、连续状态机、录制及合成代码。
- `output/scenes/dual_DVRK_ARM_continuous_suture.ttt`：默认多针场景。
- `docs/media/`：最新视频与预览。
- `main_single_suture.m` 与 `output/scenes/dual_DVRK_ARM_suture.ttt`：保留的单针对照版本；使用独立入口。
- `work/suture_review/`、`work/scene_inspection/`：单针基线及规划辅助代码；部分历史调试脚本保留原开发环境配置，不是默认运行入口。

这是规定关节轨迹的**运动学仿真**，网格碰撞检查开启，动力学求解关闭。缝线为连续可视化曲线，不含组织变形、线张力、力控制或打结力学。多针副本停用了引用旧 Clip 对象和重复旧端口的遗留脚本，并避免历史关节插值器叠加动作；未修改夹爪几何或移动训练垫来迁就机器人。
