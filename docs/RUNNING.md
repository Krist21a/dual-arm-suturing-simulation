# 运行、录制与参数配置

[返回项目首页](../README.md)

以下命令均在仓库根目录执行。

## 运行环境与入口


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
