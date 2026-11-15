function T=GetFKSutureWrist(q,arm)
% Actual grasp centre under common physical jaw yaw. Original FK unchanged.
    q=q(:).';yaw=(q(10)-q(11))/2;
    c=cos(yaw);s=sin(yaw);R=[1 0 0;0 c -s;0 s c];
    tcp=[-.000006699866697;.000120966265910;.014496663479400];
    pivot=[-.000019473955035;.000121751800179;.005111404694620];
    U=[R,pivot-tcp+R*(tcp-pivot);0 0 0 1];
    T=GetFKarmDVRK(q,11,arm)*U;
end
