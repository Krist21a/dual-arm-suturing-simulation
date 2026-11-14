function [q_solution, info] = GetIK_armins_QP( ...
        T_target, q_init, arm_type, task_mode, solver_options)
%GETIK_ARMINS_QP Lexicographic hierarchical-QP IK for the dVRK arm.
%
% Levels:
%   1. TCP position.
%   2. TCP orientation while preserving the achieved level-1 motion.
%   3. Standby posture while preserving the achieved level-1/2 motion.
%
% Joint bounds are enforced inside every QP. q10/q11 are jaw joints and
% never participate in TCP inverse kinematics.

    if nargin < 4 || isempty(task_mode)
        task_mode = 'hierarchical';
    end
    if nargin < 5
        solver_options = struct();
    end
    valid_modes = {'hierarchical', 'armPoseLockedInstrument'};
    if ~any(strcmp(task_mode, valid_modes))
        error('GetIK_armins_QP:InvalidTaskMode', ...
            ['task_mode must be ''hierarchical'' or ' ...
             '''armPoseLockedInstrument''.']);
    end
    if isempty(which('quadprog'))
        error('GetIK_armins_QP:MissingQuadprog', ...
            'The hierarchical-QP solver requires quadprog.');
    end

    max_iterations = 100;
    position_tolerance = 5e-4;
    orientation_tolerance = 1e-3;
    position_regularization = 1e-5;
    orientation_regularization = 1e-5;
    limits = GetDVRKJointLimits();
    controlled_joints = 1:9;

    q_current = q_init(:).';
    if numel(q_current) < 11
        error('GetIK_armins_QP:InsufficientJoints', ...
            'q_init must contain all 11 joint values.');
    end

    q_minimum = -2*pi*ones(9,1);
    q_maximum = 2*pi*ones(9,1);
    q_minimum(7) = limits.q7Min;
    q_maximum(7) = limits.q7Max;
    q_minimum(9) = limits.q9Min;
    q_maximum(9) = limits.q9Max;

    trust_region = deg2rad(8)*ones(9,1);
    trust_region(7) = 0.008;

    if strcmp(task_mode, 'hierarchical')
        position_active = [1 2 3 4 5 7];
        pose_active = 1:9;
        position_cost = ones(9,1);
        position_cost(7) = 0.10;
    else
        % Placement: instrument roll/pitch q8/q9 remain exactly fixed.
        position_active = 1:7;
        pose_active = 1:7;
        position_cost = ones(9,1);
        position_cost(7) = 0.04;
    end

    posture_reference = q_init(1:9).';
    if strcmp(task_mode, 'hierarchical')
        if isfield(solver_options, 'postureReference') && ...
                numel(solver_options.postureReference) >= 9
            posture_reference = solver_options.postureReference(1:9).';
        end
        posture_weights = 0.15*ones(9,1);
        posture_weights(7) = 4.0;
        posture_weights(8:9) = 0.25;
    else
        % During placement keep the arm close to the configuration at the
        % start of this Cartesian waypoint and leave q7 inexpensive. This
        % makes axial translation use the screw instead of bending q1-q6.
        posture_weights = 2.0*ones(9,1);
        posture_weights(7) = 0.005;
        posture_weights(8:9) = 0.0;
    end

    qp_options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'Algorithm', 'active-set', ...
        'ConstraintTolerance', 1e-9, ...
        'OptimalityTolerance', 1e-9, ...
        'MaxIterations', 250);

    info = struct( ...
        'converged', false, ...
        'positionConverged', false, ...
        'orientationConverged', false, ...
        'iterations', 0, ...
        'positionError', inf, ...
        'orientationError', inf, ...
        'primaryTaskLeak', 0, ...
        'solverMode', ['qp:' task_mode], ...
        'positionJointIndices', position_active, ...
        'orientationJointIndices', pose_active, ...
        'qpExitflag', 0, ...
        'activeBoundCount', 0, ...
        'minimumJointMargin', inf);

    for iteration = 1:max_iterations
        current_pose = GetFKarmDVRK(q_current, 11, arm_type);
        position_error = T_target(1:3,4) - current_pose(1:3,4);
        orientation_error = rotationErrorVectorQP( ...
            T_target(1:3,1:3), current_pose(1:3,1:3));

        info.iterations = iteration;
        info.positionError = norm(position_error);
        info.orientationError = norm(orientation_error);
        info.positionConverged = ...
            info.positionError < position_tolerance;
        info.orientationConverged = ...
            info.orientationError < orientation_tolerance;
        if info.positionConverged && info.orientationConverged
            info.converged = true;
            q_solution = q_current;
            info.minimumJointMargin = minimumJointMargin( ...
                q_current(1:9).', q_minimum, q_maximum);
            return;
        end

        jacobian = GetJac_armins(q_current, 11, arm_type);
        J_position = jacobian(1:3,controlled_joints);
        J_orientation = jacobian(4:6,controlled_joints);

        lower_bound = max(q_minimum-q_current(1:9).', -trust_region);
        upper_bound = min(q_maximum-q_current(1:9).', trust_region);

        primary_lower = lower_bound;
        primary_upper = upper_bound;
        frozen_primary = setdiff(1:9, position_active);
        primary_lower(frozen_primary) = 0;
        primary_upper(frozen_primary) = 0;

        H_position = J_position.'*J_position + ...
            position_regularization*diag(position_cost);
        f_position = -J_position.'*position_error;
        [dq_position, ~, exit_position] = quadprog( ...
            symmetrize(H_position), f_position, [], [], [], [], ...
            primary_lower, primary_upper, zeros(9,1), qp_options);
        if exit_position <= 0
            info.qpExitflag = exit_position;
            q_solution = q_current;
            return;
        end
        achieved_position = J_position*dq_position;

        secondary_lower = lower_bound;
        secondary_upper = upper_bound;
        frozen_pose = setdiff(1:9, pose_active);
        secondary_lower(frozen_pose) = 0;
        secondary_upper(frozen_pose) = 0;
        [Aeq_position, beq_position] = independentEqualities( ...
            J_position, achieved_position);
        H_orientation = J_orientation.'*J_orientation + ...
            orientation_regularization*eye(9);
        f_orientation = -J_orientation.'*orientation_error;
        [dq_orientation, ~, exit_orientation] = quadprog( ...
            symmetrize(H_orientation), f_orientation, [], [], ...
            Aeq_position, beq_position, secondary_lower, ...
            secondary_upper, dq_position, qp_options);
        if exit_orientation <= 0
            info.qpExitflag = exit_orientation;
            q_solution = q_current;
            return;
        end
        achieved_orientation = J_orientation*dq_orientation;

        % Tertiary posture QP. The first two achieved task motions are hard
        % equalities, so posture improvement cannot degrade them.
        [Aeq_tasks, beq_tasks] = independentEqualities( ...
            [J_position; J_orientation], ...
            [achieved_position; achieved_orientation]);
        posture_delta = posture_reference-q_current(1:9).';
        H_posture = diag(posture_weights) + 1e-6*eye(9);
        f_posture = -posture_weights.*posture_delta;
        [dq, ~, exit_posture] = quadprog( ...
            symmetrize(H_posture), f_posture, [], [], ...
            Aeq_tasks, beq_tasks, secondary_lower, secondary_upper, ...
            dq_orientation, qp_options);
        if exit_posture <= 0
            % The level-2 result is always a valid hierarchical result.
            dq = dq_orientation;
            exit_posture = exit_orientation;
        end

        info.qpExitflag = min([exit_position, ...
            exit_orientation, exit_posture]);
        info.primaryTaskLeak = norm( ...
            J_position*dq-achieved_position);
        active_tolerance = 1e-7;
        info.activeBoundCount = sum( ...
            abs(dq-lower_bound) < active_tolerance | ...
            abs(dq-upper_bound) < active_tolerance);

        q_current(controlled_joints) = ...
            q_current(controlled_joints) + dq.';
        q_current(1:9) = max(min(q_current(1:9), ...
            q_maximum.'), q_minimum.');
        q_current(10:11) = q_init(10:11);
    end

    q_solution = q_current;
    info.minimumJointMargin = minimumJointMargin( ...
        q_current(1:9).', q_minimum, q_maximum);
end

function matrix = symmetrize(matrix)
    matrix = (matrix+matrix.')/2;
end

function [A_reduced, b_reduced] = independentEqualities(A, b)
    if isempty(A)
        A_reduced = A;
        b_reduced = b;
        return;
    end
    [U,S,~] = svd(A, 'econ');
    singular_values = diag(S);
    tolerance = max(size(A))*eps(max([singular_values;1]));
    rank_A = sum(singular_values > tolerance);
    if rank_A == 0
        A_reduced = zeros(0,size(A,2));
        b_reduced = zeros(0,1);
    else
        basis = U(:,1:rank_A).';
        A_reduced = basis*A;
        b_reduced = basis*b;
    end
end

function error_vector = rotationErrorVectorQP( ...
        target_rotation, current_rotation)
    error_rotation = target_rotation*current_rotation.';
    axis_angle = rotm2axang(error_rotation);
    error_vector = axis_angle(1:3).'*axis_angle(4);
end

function margin = minimumJointMargin(q, q_minimum, q_maximum)
    finite_margin = min(q-q_minimum, q_maximum-q);
    margin = min(finite_margin);
end
