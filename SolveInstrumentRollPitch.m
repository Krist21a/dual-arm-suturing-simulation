function [q_solution, info] = SolveInstrumentRollPitch( ...
        q_start, arm_type, ring_rotation_in_tip, target_normal)
%SOLVEINSTRUMENTROLLPITCH Align a held ring using only q8 and q9.
%
%   q1-q7 and jaw joints are held exactly fixed. The objective aligns the
%   ring-plane normal with target_normal; rotation about that normal (yaw)
%   and the sign of the plane normal are deliberately unconstrained.

    max_iterations = 100;
    normal_tolerance = deg2rad(0.2);
    damping = 0.01;
    step_scale = 0.5;
    instrument_indices = [8 9];
    pitch_limit = deg2rad(85);

    q_current = q_start(:).';
    target_normal = target_normal(:) / norm(target_normal);
    info = struct('converged', false, 'iterations', 0, ...
        'normalError', inf, 'instrumentJointIndices', instrument_indices);

    for iteration = 1:max_iterations
        T_tip = GetFKarmDVRK(q_current, 11, arm_type);
        ring_rotation = T_tip(1:3,1:3) * ring_rotation_in_tip;
        ring_normal = ring_rotation(:,1);
        ring_normal = ring_normal / norm(ring_normal);

        signed_cosine = min(max(dot(ring_normal, target_normal), -1), 1);
        angle = acos(abs(signed_cosine));
        info.iterations = iteration;
        info.normalError = angle;
        if angle < normal_tolerance
            info.converged = true;
            q_solution = q_current;
            return;
        end

        effective_target = target_normal;
        if signed_cosine < 0
            effective_target = -target_normal;
        end
        axis_vector = cross(ring_normal, effective_target);
        axis_norm = norm(axis_vector);
        if axis_norm < 1e-10
            error('SolveInstrumentRollPitch:AntiparallelNormal', ...
                'Ring normal is antiparallel to the target normal.');
        end
        orientation_error = axis_vector / axis_norm * angle;

        J = GetJac_armins(q_current, 11, arm_type);
        J_roll_pitch = J(4:6, instrument_indices);
        dq = J_roll_pitch' / ...
            (J_roll_pitch*J_roll_pitch' + damping^2*eye(3)) * ...
            orientation_error;

        q_current(instrument_indices) = q_current(instrument_indices) + ...
            step_scale*dq.';
        q_current(instrument_indices) = max(min( ...
            q_current(instrument_indices), 2*pi), -2*pi);
        q_current(9) = max(min(q_current(9), pitch_limit), -pitch_limit);
    end

    % The two wrist axes can be close to a singular configuration after a
    % handoff.  A Jacobian iteration may then stop at a nonzero normal
    % error even though another q8/q9 branch is feasible.  Search the
    % complete two-dimensional periodic space and refine the best branch.
    search_values = linspace(-pi, pi, 9);
    best_angles = q_current(instrument_indices);
    best_cost = normalAlignmentCost(best_angles, q_start, arm_type, ...
        ring_rotation_in_tip, target_normal);
    for roll_offset = search_values
        for pitch_offset = search_values
            candidate = q_start(instrument_indices) + ...
                [roll_offset pitch_offset];
            candidate(2) = max(min(candidate(2), ...
                pitch_limit), -pitch_limit);
            candidate_cost = normalAlignmentCost(candidate, q_start, ...
                arm_type, ring_rotation_in_tip, target_normal);
            if candidate_cost < best_cost
                best_cost = candidate_cost;
                best_angles = candidate;
            end
        end
    end

    options = optimset('Display', 'off', 'TolX', 1e-10, ...
        'TolFun', 1e-12, 'MaxIter', 300, 'MaxFunEvals', 1000);
    refined_angles = fminsearch(@(angles) normalAlignmentCost( ...
        angles, q_start, arm_type, ring_rotation_in_tip, target_normal), ...
        best_angles, options);
    refined_angles = q_start(instrument_indices) + mod( ...
        refined_angles-q_start(instrument_indices)+pi, 2*pi)-pi;
    refined_angles(2) = max(min(refined_angles(2), ...
        pitch_limit), -pitch_limit);
    q_current = q_start(:).';
    q_current(instrument_indices) = refined_angles;

    T_tip = GetFKarmDVRK(q_current, 11, arm_type);
    ring_rotation = T_tip(1:3,1:3) * ring_rotation_in_tip;
    ring_normal = ring_rotation(:,1) / norm(ring_rotation(:,1));
    cosine = min(max(dot(ring_normal, target_normal), -1), 1);
    info.normalError = acos(abs(cosine));
    info.iterations = info.iterations + 1;
    info.converged = info.normalError < normal_tolerance;
    q_solution = q_current;
end

function cost = normalAlignmentCost(instrument_angles, q_reference, ...
        arm_type, ring_rotation_in_tip, target_normal)
    pitch_limit = deg2rad(85);
    instrument_angles(2) = max(min(instrument_angles(2), ...
        pitch_limit), -pitch_limit);
    q_candidate = q_reference(:).';
    q_candidate(8:9) = instrument_angles;
    T_tip = GetFKarmDVRK(q_candidate, 11, arm_type);
    ring_rotation = T_tip(1:3,1:3) * ring_rotation_in_tip;
    ring_normal = ring_rotation(:,1) / norm(ring_rotation(:,1));
    signed_alignment = min(max(dot(ring_normal, target_normal), -1), 1);
    alignment_cost = 1 - abs(signed_alignment);

    % Resolve equivalent periodic solutions in favour of the least wrist
    % travel, without materially changing the normal-alignment objective.
    travel = instrument_angles(:) - q_reference(8:9).';
    cost = alignment_cost + 1e-4 * dot(travel, travel);
end
