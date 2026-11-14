%% === 获取右臂 RCM (Kinematic Base) 坐标 ===
clc;
client = RemoteAPIClient();
sim = client.require('sim');

% 1. 获取右臂 RCM 附近的物体句柄
% dVRK 的 RCM 通常定义在 joint1 或 joint2 的位置
% 我们直接读取 joint1 (Yaw) 和 joint2 (Pitch) 的世界坐标
h_R_J1 = sim.getObject('/500____[1]/joint1');
h_R_J2 = sim.getObject('/500____[1]/joint2');

% 2. 读取坐标
pos_J1 = sim.getObjectPosition(h_R_J1, -1);
pos_J2 = sim.getObjectPosition(h_R_J2, -1);

% 转换为 double
if iscell(pos_J1), pos_J1 = cell2mat(pos_J1); end
if iscell(pos_J2), pos_J2 = cell2mat(pos_J2); end

fprintf('\n=== 右臂 RCM 坐标诊断 ===\n');
fprintf('Joint 1 (Yaw)   坐标: [%.4f, %.4f, %.4f]\n', pos_J1);
fprintf('Joint 2 (Pitch) 坐标: [%.4f, %.4f, %.4f]\n', pos_J2);

% RCM 理论上是这两者的交点或重合点
% 如果 J1 和 J2 几乎重合，那这就是 RCM
RCM_Right = pos_J1;

fprintf('\n>>> 建议使用的 T_base (平移部分): [%.4f, %.4f, %.4f] <<<\n', RCM_Right);
fprintf('    (请用这个坐标替换 GetFKarmDVRK.m 里右臂的 [0.6, 0, 0.5147])\n');
