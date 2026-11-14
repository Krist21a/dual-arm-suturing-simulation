function T_out = GetFKarmDVRK(joint_pos, joint_index, arm_type)
%GETFKARMDVRK Forward kinematics for the calibrated dVRK arm model.
%
%   arm_type must be 'L' or 'R'. The TCP is the symmetric needle-holder
%   jaw centre, so joint 10/11 jaw opening does not change joint_index 11.

    if nargin < 3 || ~(strcmp(arm_type, 'L') || strcmp(arm_type, 'R'))
        error('GetFKarmDVRK:InvalidArmType', 'arm_type must be ''L'' or ''R''.');
    end

    joint_pos = joint_pos(:).';
    if numel(joint_pos) < 11
        error('GetFKarmDVRK:InvalidJointVector', 'joint_pos must contain all 11 joint values.');
    end

    if strcmp(arm_type, 'R')
        T_base = [ 1,  0,  0,  0.60000;
                   0,  1,  0,  0.00000;
                   0,  0,  1,  0.51465;
                   0,  0,  0,  1.0000];
    else
        T_base = [ 1,  0,  0,  0.00000;
                   0,  1,  0,  0.00000;
                   0,  0,  1,  0.51465;
                   0,  0,  0,  1.0000];
    end

    L010 = [1 0 0 0; 0 1 0 0; 0 0 1 0.0085; 0 0 0 1];
    M1 = rotz4(joint_pos(1));

    L120 = [0 0 -1 -0.0703; 0 1 0 0; 1 0 0 0.06605; 0 0 0 1];
    M2 = rotz4(joint_pos(2));

    L230 = [1 0 0 0.4251; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    M3 = rotz4(joint_pos(3));

    L340 = [1 0 0 0.39215; 0 1 0 0; 0 0 1 0; 0 0 0 1];
    M4 = rotz4(joint_pos(4));

    L450 = [0 0 1 0.04557; 0 1 0 0; -1 0 0 0.0397; 0 0 0 1];
    M5 = rotz4(joint_pos(5));

    L560 = [0 0 -1 0.01443; 0 1 0 0; 1 0 0 0.04918; 0 0 0 1];
    M6 = rotz4(joint_pos(6));

    L660 = [1 0 0 0; 0 1 0 0; 0 0 1 0.08938; 0 0 0 1];

    T_6 = T_base * L010 * M1 * L120 * M2 * L230 * M3 * ...
        L340 * M4 * L450 * M5 * L560 * M6 * L660;

    % Calibrated from dual_DVRK_ARM_new_instrument.ttt. L670 places the
    % prismatic axis at the redesigned bottom-mounted interface.
    if strcmp(arm_type, 'R')
        L670 = pose4(0.0442, -0.0682, 0.1175710689583, ...
            0, 0.707106781187, 0.707106781187, 0);
    else
        L670 = pose4(-0.0442, -0.0682, 0.1175710689583, ...
            -0.707106781187, 0, 0, 0.707106781187);
    end
    M7 = [1 0 0 0; 0 1 0 0; 0 0 1 joint_pos(7); 0 0 0 1];

    % The drive units are mirrored: left dVRK is rolled 90 deg clockwise,
    % right dVRK 90 deg counterclockwise about the instrument shaft.
    if strcmp(arm_type, 'R')
        L7Tool = pose4(0.0442, 0, 0, 0, 0, -0.707106781187, 0.707106781187);
    else
        L7Tool = pose4(0.0442, 0, 0, 0, 0, 0.707106781187, 0.707106781187);
    end
    LTool8 = pose4(0, 0, 0, 0, 0, -1, 0);
    M8 = rotz4(joint_pos(8));

    L8Link1 = pose4(0.000455975532532, -0.000129997730255, 0.226981401443, ...
        -0.707106781187, 0.707106781187, 0, 0);
    LLink19 = pose4(0.000007484114705, 0.000230443676010, -0.225139383808, ...
        0.707106781187, 0, 0.707106781187, 0);
    M9 = rotz4(joint_pos(9));

    L9Wrist = pose4(-0.003876566886900, -0.000133752822876, 0.000012136995792, ...
        -0.5, 0.5, 0.5, -0.5);
    LWrist10 = pose4(-0.000019473955035, 0.000121751800179, 0.005111404694620, ...
        0.5, 0.5, 0.5, 0.5);
    M10 = rotz4(joint_pos(10));

    LWristTip = pose4(-0.000006699866697, 0.000120966265910, 0.014496663479400, ...
        0, 0, 0, 1);

    T_tool = T_6 * L670 * M7 * L7Tool;
    T_joint8 = T_tool * LTool8 * M8;
    T_joint9 = T_joint8 * L8Link1 * LLink19 * M9;
    T_wrist = T_joint9 * L9Wrist;

    switch joint_index
        case 6
            T_out = T_6;
        case 7
            T_out = T_tool;
        case 8
            T_out = T_joint8;
        case 9
            T_out = T_joint9;
        case 10
            T_out = T_wrist * LWrist10 * M10;
        case 11
            T_out = T_wrist * LWristTip;
        otherwise
            error('GetFKarmDVRK:InvalidJointIndex', 'joint_index must be 6, 7, 8, 9, 10, or 11.');
    end
end

function T = pose4(x, y, z, qx, qy, qz, qw)
    n = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    qx = qx / n;
    qy = qy / n;
    qz = qz / n;
    qw = qw / n;

    R = [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw);
         2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw);
         2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)];
    T = [R, [x; y; z]; 0, 0, 0, 1];
end

function T = rotz4(theta)
    c = cos(theta);
    s = sin(theta);
    T = [c -s 0 0; s c 0 0; 0 0 1 0; 0 0 0 1];
end
