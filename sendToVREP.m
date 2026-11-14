function sendToVREP(sim, jointHandles, q)
%SENDTOVREP Send a joint target vector to CoppeliaSim.

    n = min(numel(jointHandles), numel(q));
    for i = 1:n
        sim.setJointTargetPosition(jointHandles(i), q(i));
    end
end
