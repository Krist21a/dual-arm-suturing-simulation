function [q_traj, info] = PlanPath_PoseSequence( ...
        pose_sequence, q_start_guess, arm_type, ik_mode, solver_options)
%PLANPATH_POSESEQUENCE Solve a joint trajectory for arbitrary TCP poses.
%
% pose_sequence is 4-by-4-by-N. Each IK solution seeds the next waypoint,
% which is important for the coupled rotation/translation of a curved needle.

    if nargin < 4 || isempty(ik_mode)
        ik_mode = 'hierarchical';
    end
    if nargin < 5
        solver_options = struct();
    end
    if size(pose_sequence,1) ~= 4 || size(pose_sequence,2) ~= 4
        error('PlanPath_PoseSequence:InvalidPoseArray', ...
            'pose_sequence must be a 4-by-4-by-N array.');
    end

    steps = size(pose_sequence,3);
    num_joints = numel(q_start_guess);
    q_traj = zeros(steps, num_joints);
    info = struct( ...
        'converged', false(steps,1), ...
        'positionConverged', false(steps,1), ...
        'orientationConverged', false(steps,1), ...
        'iterations', zeros(steps,1), ...
        'positionError', inf(steps,1), ...
        'orientationError', inf(steps,1), ...
        'primaryTaskLeak', inf(steps,1), ...
        'qpExitflag', zeros(steps,1), ...
        'activeBoundCount', zeros(steps,1));

    q_last = q_start_guess(:).';
    for step = 1:steps
        [q_solution, ik_info] = GetIK_armins( ...
            pose_sequence(:,:,step), q_last, arm_type, ...
            ik_mode, solver_options);
        q_traj(step,:) = q_solution;
        q_last = q_solution;

        info.converged(step) = ik_info.converged;
        info.positionConverged(step) = ik_info.positionConverged;
        info.orientationConverged(step) = ik_info.orientationConverged;
        info.iterations(step) = ik_info.iterations;
        info.positionError(step) = ik_info.positionError;
        info.orientationError(step) = ik_info.orientationError;
        info.primaryTaskLeak(step) = ik_info.primaryTaskLeak;
        if isfield(ik_info, 'qpExitflag')
            info.qpExitflag(step) = ik_info.qpExitflag;
            info.activeBoundCount(step) = ik_info.activeBoundCount;
        end
    end
end
