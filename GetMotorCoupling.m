function q_motor = GetMotorCoupling(theta_joint, pulley_params)
% GetMotorCoupling 基于线轮传动机制的运动解耦
% 参考文献: Intraoperative Kinematic Analysis... (Feng et al., 2021)
%
% 输入:
%   theta_joint: 3x1 向量 [theta_5; theta_6; theta_7] (单位: 弧度)
%       - theta_5: 手腕旋转 (Wrist Rotation / Roll)
%       - theta_6: 手腕俯仰 (Wrist Pitch)
%       - theta_7: 夹爪张开角度 (Gripper Open Angle)
%   pulley_params: 结构体，包含滑轮直径 a, b, c, d, e
%
% 输出:
%   q_motor: 3x1 向量 [q_5; q_6; q_7] (单位: 弧度)
%       - q_5: 电机 5 角度 (驱动 Pulley b)
%       - q_6: 电机 6 角度 (驱动 Pulley e1)
%       - q_7: 电机 7 角度 (驱动 Pulley e2)

    % 1. 提取滑轮参数 [cite: 328]
    a = pulley_params.a;
    b = pulley_params.b;
    c = pulley_params.c;
    d = pulley_params.d;
    e = pulley_params.e;

    % 2. 构建耦合矩阵 (Coupling Matrix)
    % 对应论文中的 Equation (7)
    % [ b/a,   0,       0    ]
    % [  0,   d/c,   e/(2c)  ]
    % [  0,   d/c,  -e/(2c)  ]

    T_coupling = [ ...
        b/a,    0,       0; ...
           0,  d/c,  e/(2*c); ...
           0,  d/c, -e/(2*c) ...
    ];

    % 3. 计算电机角度
    % q = T * theta
    q_motor = T_coupling * theta_joint;

end
