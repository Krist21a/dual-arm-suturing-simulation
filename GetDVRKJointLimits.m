function limits = GetDVRKJointLimits()
%GETDVRKJOINTLIMITS Central joint limits used by IK and execution checks.
% joint7 has a temporary 0.16 m hard upper limit pending calibration.

    limits.q7Min = 0.0;
    limits.q7Max = 0.16;
    limits.q9Min = deg2rad(-85);
    limits.q9Max = deg2rad(85);
end
