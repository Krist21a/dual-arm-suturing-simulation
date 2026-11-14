% === 工具脚本：获取基座真实位姿 ===
clc;
client = RemoteAPIClient();
sim = client.require('sim');

% 使用你在 main.m 里定义的路径
h_L_Base = sim.getObject('/500____[0]');
h_R_Base = sim.getObject('/500____[1]');

% 获取世界坐标系下的变换矩阵 (4x4)
m_L = sim.getObjectMatrix(h_L_Base, -1);
m_R = sim.getObjectMatrix(h_R_Base, -1);

fprintf('=== 请将以下数据更新到 GetFKarmDVRK.m 中 ===\n');
fprintf('Left Arm Base T_base:\n');
disp(m_L);
fprintf('Right Arm Base T_base:\n');
disp(m_R);
