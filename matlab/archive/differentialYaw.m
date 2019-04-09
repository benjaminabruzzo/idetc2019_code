function diff_yaw = differentialYaw(A,B)
    %% Computes yaw angle of vector B from vector A
    % positive angles are CCW rotation of B wrt z axis
    Bx = [B(:,2), -B(:,1), B(:,3)];
    diff_yaw = atan2(dot(Bx', A')',dot(B', A')') * 180/pi;
end


% A = vicon.TBKobuki_.x_axis;
% B = vicon.UAV.x_axis(:,1:3);
% Bx = [B(:,2), -B(:,1), B(:,3)];            
% vicon.diff_yaw.uav_ugv = atan2(dot(Bx', A')',dot(B', A')') * 180/pi;
% vicon.diff_yaw.mean_uav_ugv = mean(vicon.diff_yaw.uav_ugv);
