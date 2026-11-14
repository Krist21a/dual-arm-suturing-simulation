function [q_sol, info] = GetIK_armins( ...
        T_target, q_init, arm_type, solver_mode, solver_options)
%GETIK_ARMINS Hierarchical damped-least-squares IK for the dVRK arm.
%
%   q_sol = GetIK_armins(T_target, q_init, arm_type)
%   [q_sol, info] = GetIK_armins(...)
%
%   Task hierarchy:
%     1. q1-q5 and prismatic q7 realize TCP position.
%     2. q6, q8 and q9 realize TCP orientation.
%     3. Position joints compensate the small TCP translation introduced by
%        the orientation joints, so orientation is solved in the position
%        task's numerical null space.
%
%   q10 and q11 are jaw joints and never participate in TCP IK.

    if nargin < 4
        solver_mode = 'hierarchical';
    end
    if nargin < 5
        solver_options = struct();
    end
    qp_prefix = 'qp:';
    if strncmp(solver_mode, qp_prefix, numel(qp_prefix))
        task_mode = solver_mode(numel(qp_prefix)+1:end);
        [q_sol, info] = GetIK_armins_QP( ...
            T_target, q_init, arm_type, task_mode, solver_options);
        return;
    end
    valid_modes = {'hierarchical', 'coupled', 'armPoseLockedInstrument'};
    if ~any(strcmp(solver_mode, valid_modes))
        error('GetIK_armins:InvalidSolverMode', ...
            ['solver_mode must be ''hierarchical'', ''coupled'', or ' ...
             '''armPoseLockedInstrument''.']);
    end

    max_iter = 150;
    pos_tol = 5e-4;
    orn_tol = 1e-3;
    step_scale = 0.5;
    lambda_position = 0.01;
    lambda_orientation = 0.01;
    lambda_compensation = 0.001;
    limits = GetDVRKJointLimits();
    target_link_index = 11;

    q_curr = q_init(:).';
    num_joints = numel(q_curr);
    if num_joints < 9
        error('GetIK_armins:InsufficientJoints', ...
            'The hierarchical solver requires at least q1 through q9.');
    end

    position_indices = [1 2 3 4 5 7];
    orientation_indices = [6 8 9];

    q_min = -2*pi * ones(1, num_joints);
    q_max =  2*pi * ones(1, num_joints);
    q_min(7) = limits.q7Min;
    q_max(7) = limits.q7Max;
    q_min(9) = limits.q9Min;
    q_max(9) = limits.q9Max;

    % Selection matrices embed each task's reduced joint vector in the full
    % q vector. Jaw columns remain zero.
    S_position = selectionMatrix(num_joints, position_indices);
    S_orientation = selectionMatrix(num_joints, orientation_indices);

    info = struct( ...
        'converged', false, ...
        'positionConverged', false, ...
        'orientationConverged', false, ...
        'iterations', 0, ...
        'positionError', inf, ...
        'orientationError', inf, ...
        'primaryTaskLeak', 0, ...
        'solverMode', solver_mode, ...
        'positionJointIndices', position_indices, ...
        'orientationJointIndices', orientation_indices);

    for iteration = 1:max_iter
        T_curr = GetFKarmDVRK(q_curr, target_link_index, arm_type);
        position_error = T_target(1:3,4) - T_curr(1:3,4);
        orientation_error = rotationErrorVector( ...
            T_target(1:3,1:3), T_curr(1:3,1:3));

        info.iterations = iteration;
        info.positionError = norm(position_error);
        info.orientationError = norm(orientation_error);
        info.positionConverged = info.positionError < pos_tol;
        info.orientationConverged = info.orientationError < orn_tol;

        if info.positionConverged && info.orientationConverged
            info.converged = true;
            q_sol = q_curr;
            return;
        end

        J = GetJac_armins(q_curr, target_link_index, arm_type);
        J_position = J(1:3,:);
        J_orientation = J(4:6,:);

        if strcmp(solver_mode, 'hierarchical')
            % Primary task: position uses arm q1-q5 and screw q7.
            J_position_reduced = J_position * S_position;
            J_position_pinv = dampedPseudoInverse( ...
                J_position_reduced, lambda_position);
            dq_position = S_position * ...
                (J_position_pinv * position_error);

            % q1-q5/q7 cancel the translation introduced by q6/q8/q9.
            compensation_pinv = dampedPseudoInverse( ...
                J_position_reduced, lambda_compensation);
            position_preserving_map = S_orientation - S_position * ...
                (compensation_pinv * (J_position * S_orientation));

            orientation_residual = orientation_error - ...
                J_orientation * dq_position;
            J_orientation_hierarchical = ...
                J_orientation * position_preserving_map;
            dq_orientation = position_preserving_map * ...
                (dampedPseudoInverse(J_orientation_hierarchical, ...
                lambda_orientation) * orientation_residual);

            info.primaryTaskLeak = norm(J_position * dq_orientation);
            dq = dq_position + dq_orientation;
        elseif strcmp(solver_mode, 'coupled')
            % A single damped solve is more robust for the large coupled
            % wrist/arm rotations used by the curved-needle trajectory.
            % Jaw joints remain excluded and hard insertion/wrist limits
            % are applied below after every iteration.
            active_indices = 1:9;
            S_active = selectionMatrix(num_joints, active_indices);
            J_active = J*S_active;
            task_error = [position_error; orientation_error];
            dq = S_active*(dampedPseudoInverse(J_active,0.006)*task_error);

            % Trust-region caps prevent a near-singular waypoint from
            % jumping to a distant inverse-kinematics branch.
            revolute_indices = [1:6 8:9];
            dq(revolute_indices) = max(min( ...
                dq(revolute_indices),0.12),-0.12);
            dq(7) = max(min(dq(7),0.004),-0.004);
            dq = 1.3*dq; % 0.5 global scale below -> effective 0.65
            info.primaryTaskLeak = 0;
        else
            % After the instrument has rotated the ring horizontal, freeze
            % q8/q9. q1-q7 make the small alignment/lowering correction while
            % maintaining the attained TCP attitude.
            arm_pose_indices = 1:7;
            S_arm_pose = selectionMatrix(num_joints, arm_pose_indices);
            J_position_arm = J_position * S_arm_pose;
            % During placement, favour the linear screw over large arm
            % rotations for translational corrections. The hard q7 limit
            % below still takes precedence over this soft preference.
            arm_position_weights = ones(numel(arm_pose_indices),1);
            arm_position_weights(7) = 25;
            J_position_arm_pinv = weightedDampedPseudoInverse( ...
                J_position_arm, arm_position_weights, lambda_position);
            dq_position = S_arm_pose * ...
                (J_position_arm_pinv * position_error);

            % Maintain the requested attitude only in the position task's
            % numerical null space. This avoids sacrificing millimetres of
            % peg alignment near an arm singularity.
            position_null_map = S_arm_pose - S_arm_pose * ...
                (J_position_arm_pinv * J_position_arm);
            orientation_residual = orientation_error - ...
                J_orientation * dq_position;
            J_orientation_null = J_orientation * position_null_map;
            dq_orientation = position_null_map * ...
                (dampedPseudoInverse(J_orientation_null, ...
                lambda_orientation) * orientation_residual);
            dq = dq_position + dq_orientation;
            info.primaryTaskLeak = norm(J_position * dq_orientation);
        end

        % Jaw joints are explicitly frozen even if future FK/Jacobian changes
        % accidentally introduce small nonzero columns.
        if num_joints >= 11
            dq(10:11) = 0;
        end

        q_next = q_curr + step_scale * dq.';
        q_curr = max(min(q_next, q_max), q_min);
    end

    q_sol = q_curr;
    if nargout < 2
        warning('GetIK_armins:DidNotConverge', ...
            ['Hierarchical IK did not converge after %d iterations. ' ...
             'Position error %.6f m, orientation error %.6f rad.'], ...
            info.iterations, info.positionError, info.orientationError);
    end
end

function S = selectionMatrix(num_joints, indices)
    S = zeros(num_joints, numel(indices));
    for column = 1:numel(indices)
        S(indices(column), column) = 1;
    end
end

function inverse = dampedPseudoInverse(matrix, damping)
    rows = size(matrix,1);
    inverse = matrix' / ...
        (matrix*matrix' + damping^2*eye(rows));
end

function inverse = weightedDampedPseudoInverse(matrix, weights, damping)
    weight_matrix = diag(weights(:));
    rows = size(matrix,1);
    inverse = weight_matrix * matrix' / ...
        (matrix*weight_matrix*matrix' + damping^2*eye(rows));
end

function error_vector = rotationErrorVector(target_rotation, current_rotation)
    error_rotation = target_rotation * current_rotation.';
    try
        axis_angle = rotm2axang(error_rotation);
    catch err
        error('GetIK_armins:MissingRoboticsToolbox', ...
            'rotm2axang is required for orientation error: %s', err.message);
    end
    error_vector = axis_angle(1:3).' * axis_angle(4);
end
