function drive_cmd = GetDVRK_Drive(joint_target)
% GetDVRK_Drive 根据图片参数计算驱动指令
%
% 输入:
%   joint_target: 结构体或向量，包含期望的末端角度 (单位: 弧度 rad)
%       .roll   (自转)
%       .pitch  (俯仰)
%       .yaw    (偏航)
%       .open   (夹爪张开角, total opening angle)
%
% 输出:
%   drive_cmd: 4x1 向量 [Axis4_Rot; Axis5_Len; Axis6_Len; Axis7_Len]
%       - Axis4: 驱动盘旋转角度 (rad)
%       - Axis5: 钢丝拉伸长度 (mm)
%       - Axis6: 钢丝拉伸长度 (mm)
%       - Axis7: 钢丝拉伸长度 (mm)

    %% 1. 定义核心参数 (来自图片)
    ratio_roll = 1.533;      % Axis 4 传动比 (23/15)
    r_pitch    = 3.25;       % Axis 5 半径 (mm)
    r_diff     = 2.25;       % Axis 6/7 半径 (mm)用于Yaw和Grip

    %% 2. 提取目标角度
    q_roll  = joint_target.roll;
    q_pitch = joint_target.pitch;
    q_yaw   = joint_target.yaw;
    q_open  = joint_target.open; % 夹爪总张开角

    %% 3. 计算驱动指令 (Kinematic Mapping)

    % --- Axis 4: Roll (齿轮传动) ---
    % 逻辑: 纯线性比例
    cmd_axis_4 = ratio_roll * q_roll;

    % --- Axis 5: Pitch (主驱动力臂) ---
    % 逻辑: 弧长公式 L = r * theta
    % 注意: 在真实 dVRK 中，Pitch 运动通常会带动 Axis 6/7 的钢丝。
    % 如果是纯解耦模型，直接计算如下：
    cmd_axis_5 = r_pitch * q_pitch;

    % --- Axis 6 & 7: Yaw & Grip (差动控制) ---
    % 逻辑:
    %   Yaw 运动: 两个轮子同向旋转 (带动整体偏转)
    %   Grip 运动: 两个轮子反向旋转 (一个开一个合)
    %   通常定义: Grip 角度由两片夹爪各分担一半 (q_open / 2)

    delta_yaw  = r_diff * q_yaw;        % 偏航引起的线位移
    delta_grip = r_diff * (q_open / 2); % 开合引起的线位移

    % 差动混合公式 (根据 dVRK 标准绕线逻辑)
    % Axis 6 和 Axis 7 一个负责左侧钢丝，一个负责右侧
    % 组合方式通常如下 (符号可能需根据实际电机方向微调):
    cmd_axis_6 = delta_yaw + delta_grip;
    cmd_axis_7 = delta_yaw - delta_grip;

    %% 4. 组装输出
    drive_cmd = [cmd_axis_4; cmd_axis_5; cmd_axis_6; cmd_axis_7];
end
