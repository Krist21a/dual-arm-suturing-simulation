function [q_traj, info] = PlanPath_Line( ...
        T_start, T_end, steps, q_start_guess, arm_type, ...
        ik_mode, solver_options)
%PLANPATH_LINE Generate a joint trajectory for a Cartesian line segment.
%
%   q_traj = PlanPath_Line(T_start, T_end, steps, q_start_guess, arm_type)
%   [q_traj, info] = PlanPath_Line(...)

    if nargin < 6
        ik_mode = 'hierarchical';
    end
    if nargin < 7
        solver_options = struct();
    end

    if steps < 2
        error('PlanPath_Line:InvalidSteps', 'steps must be at least 2.');
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

    pos_start = T_start(1:3, 4);
    pos_end = T_end(1:3, 4);

    q_rot_start = rotm2quat(T_start(1:3, 1:3));
    q_rot_end = rotm2quat(T_end(1:3, 1:3));
    if dot(q_rot_start, q_rot_end) < 0
        q_rot_end = -q_rot_end;
    end

    q_last = q_start_guess(:).';

    for i = 1:steps
        alpha = (i - 1) / (steps - 1);

        pos_curr = pos_start * (1 - alpha) + pos_end * alpha;

        q_rot_curr = q_rot_start * (1 - alpha) + q_rot_end * alpha;
        q_rot_curr = q_rot_curr / norm(q_rot_curr);

        T_curr = eye(4);
        T_curr(1:3, 1:3) = quat2rotm(q_rot_curr);
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
        warning('PlanPath_Line:IKDidNotConverge', ...
            'IK did not converge at trajectory step %d. Position error %.6f m, orientation error %.6f rad.', ...
            firstFailed, info.positionError(firstFailed), info.orientationError(firstFailed));
    end
end
