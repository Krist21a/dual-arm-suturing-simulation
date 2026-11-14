function J_out = GetJac_armins(joint_pos, joint_index, arm_type)
%GETJAC_ARMINS Numerical 6xN Jacobian based on FK finite differences.
%
%   The output order is [linear_velocity; angular_velocity].

    q_ = joint_pos(:).';
    inc_ = 1e-4;
    num_joints = numel(q_);
    J_ele = zeros(6, num_joints);

    T_ = GetFKarmDVRK(q_, joint_index, arm_type);
    R_ = T_(1:3, 1:3);
    x_ = T_(1:3, 4);
    q_rot_ = rotm2quat(R_);

    for j = 1:num_joints
        q_inc_ = q_;
        q_inc_(j) = q_inc_(j) + inc_;

        T_inc_ = GetFKarmDVRK(q_inc_, joint_index, arm_type);
        R_inc_ = T_inc_(1:3, 1:3);
        x_inc_ = T_inc_(1:3, 4);
        q_rot_inc_ = rotm2quat(R_inc_);

        dx = (x_inc_ - x_) / inc_;

        dq = quatmultiply(q_rot_inc_, quatinv(q_rot_));
        dq = dq / norm(dq);

        if dq(1) < 0
            dq = -dq;
        end

        scalar_part = min(max(dq(1), -1), 1);
        angle = 2 * acos(scalar_part);

        if abs(angle) < 1e-6
            dw = [0; 0; 0];
        else
            axis_vec = dq(2:4) / sin(angle / 2);
            dtheta = axis_vec(:) * angle;
            dw = dtheta / inc_;
        end

        J_ele(:, j) = [dx; dw];
    end

    J_out = J_ele;
end
