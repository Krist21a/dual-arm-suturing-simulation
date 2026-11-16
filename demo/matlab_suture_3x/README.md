# MATLAB 双臂连续缝合在线仿真

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
