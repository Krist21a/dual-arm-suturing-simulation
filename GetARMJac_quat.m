function J_out = GetARMJac_quat(joint_pos, joint_idx, arm_type)
%GETARMJAC_QUAT Backward-compatible numerical Jacobian helper.
%
%   Prefer GetJac_armins(joint_pos, joint_idx, arm_type) in new code.

    if nargin < 3 || isempty(arm_type)
        arm_type = 'L';
        warning('GetARMJac_quat:MissingArmType', ...
            'arm_type was not provided; defaulting to left arm.');
    end

    q_ = joint_pos(:).';
    inc_ = 1e-4;
    num_cols = min(joint_idx, numel(q_));
    J_ele = zeros(6, num_cols);

    T_ = GetFKarmDVRK(q_, joint_idx, arm_type);
    R_ = T_(1:3, 1:3);
    x_ = T_(1:3, 4);
    q_rot_ = rotm2quat(R_);

    for j = 1:num_cols
        q_inc_ = q_;
        q_inc_(j) = q_inc_(j) + inc_;

        T_inc_ = GetFKarmDVRK(q_inc_, joint_idx, arm_type);
        R_inc_ = T_inc_(1:3, 1:3);
        x_inc_ = T_inc_(1:3, 4);
        q_rot_inc_ = rotm2quat(R_inc_);

        dx = (x_inc_ - x_) / inc_;

        dq = quatmultiply(q_rot_inc_, quatinv(q_rot_));
        dq = dq / norm(dq);

        if dq(1) < 0
            dq = -dq;
        end

        angle = 2 * acos(dq(1));
        if abs(angle) < 1e-6
            dtheta = [0; 0; 0];
        else
            axis_vec = dq(2:4) / sin(angle / 2);
            dtheta = axis_vec(:) * angle;
        end

        dw = dtheta / inc_;
        J_ele(:, j) = [dx; dw];
    end

    J_out = J_ele;
end
