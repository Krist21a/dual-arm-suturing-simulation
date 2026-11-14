function [q_traj, info] = PlanPath_Circle( ...
        center, radius, axis_norm, angle_range, steps, T_ref_rot, ...
        q_start_guess, arm_type, ik_mode, solver_options)
%PLANPATH_CIRCLE Generate a joint trajectory for a Cartesian circular arc.

    if nargin < 9
        ik_mode = 'hierarchical';
    end
    if nargin < 10
        solver_options = struct();
    end

    if steps < 2
        error('PlanPath_Circle:InvalidSteps', 'steps must be at least 2.');
    end
    if norm(axis_norm) == 0
        error('PlanPath_Circle:InvalidAxis', 'axis_norm must be nonzero.');
    end

    num_joints = numel(q_start_guess);
    q_traj = zeros(steps, num_joints);
    info = struct( ...
        'converged', false(steps, 1), ...
        'positionConverged', false(steps, 1), ...
        'orientationConverged', false(steps, 1), ...
        'iterations', zeros(steps, 1), ...
        'positionError', inf(steps, 1), ...
        'orientationError', inf(steps, 1), ...
        'primaryTaskLeak', inf(steps, 1), ...
        'qpExitflag', zeros(steps, 1), ...
        'activeBoundCount', zeros(steps, 1));

    center = center(:);
    axis_norm = axis_norm(:) / norm(axis_norm);
    temp_vec = [1; 0; 0];
    if abs(dot(temp_vec, axis_norm)) > 0.9
        temp_vec = [0; 1; 0];
    end

    u = cross(axis_norm, temp_vec);
    u = u / norm(u);
    v = cross(axis_norm, u);

    theta_start = angle_range(1);
    theta_end = angle_range(2);
    q_last = q_start_guess(:).';

    for i = 1:steps
        alpha = (i - 1) / (steps - 1);
        theta = theta_start + alpha * (theta_end - theta_start);

        pos_curr = center + radius * cos(theta) * u + radius * sin(theta) * v;

        T_curr = eye(4);
        T_curr(1:3, 1:3) = T_ref_rot;
        T_curr(1:3, 4) = pos_curr;

        [q_sol, ik_info] = GetIK_armins( ...
            T_curr, q_last, arm_type, ik_mode, solver_options);

        q_traj(i, :) = q_sol;
        q_last = q_sol;

        info.converged(i) = ik_info.converged;
        info.positionConverged(i) = ik_info.positionConverged;
        info.orientationConverged(i) = ik_info.orientationConverged;
        info.iterations(i) = ik_info.iterations;
        info.positionError(i) = ik_info.positionError;
        info.orientationError(i) = ik_info.orientationError;
        info.primaryTaskLeak(i) = ik_info.primaryTaskLeak;
        if isfield(ik_info, 'qpExitflag')
            info.qpExitflag(i) = ik_info.qpExitflag;
            info.activeBoundCount(i) = ik_info.activeBoundCount;
        end
    end

    if nargout < 2 && any(~info.converged)
        firstFailed = find(~info.converged, 1, 'first');
        warning('PlanPath_Circle:IKDidNotConverge', ...
            'IK did not converge at trajectory step %d. Position error %.6f m, orientation error %.6f rad.', ...
            firstFailed, info.positionError(firstFailed), info.orientationError(firstFailed));
    end
end
