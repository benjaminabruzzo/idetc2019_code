function out = crossOrientation(RedP, BlueP, GreenP, RedJJt, BlueJJt, GreenJJt)
%% compute orientation of UAV from vectors in ugv frame
    GmB = GreenP - BlueP;
    RmB =   RedP - BlueP;
    BmR =  BlueP - RedP;
    RmG =   RedP - GreenP;
 
    znum = cross(GmB, RmB);
    z_axis = znum/norm(znum);
    y_axis = BmR / norm(BmR);
    x_axis = cross(y_axis, z_axis);

    out.yaw_ugv = atan2(x_axis(1, 2), x_axis(1, 1)); %//heading angle in radians in ugv frame
    
    out.z_axis = z_axis;
    out.y_axis = y_axis;
    out.x_axis = x_axis;    
    
%% propagate covariances from JJt to yaw
    GandB_JJt = GreenJJt + BlueJJt;
    RandB_JJt = RedJJt + BlueJJt;
    
    z_JJt = wedgevector(BmR) * GandB_JJt * wedgevector(BmR)' + ...
            wedgevector(GmB) * RandB_JJt * wedgevector(GmB)';

    out.z_jacobianOfNorm = jacobianOfNorm(znum);
    out.z_JJt = out.z_jacobianOfNorm * z_JJt * out.z_jacobianOfNorm';
    
    out.y_jacobianOfNorm = jacobianOfNorm(BmR);
    out.y_JJt = out.y_jacobianOfNorm * RandB_JJt * out.y_jacobianOfNorm';
    
    out.x_JJt = wedgevector(-z_axis) * out.y_JJt * wedgevector(-z_axis)' + ...
                wedgevector( y_axis) * out.z_JJt * wedgevector( y_axis)';

    out.x_JJt2x2 = out.x_JJt(1:2,1:2);
    out.x_jacobianOfAtan2 = jacobianOfAtan2(x_axis(1,1),x_axis(1,2));
    
    out.yaw_JJt = out.x_jacobianOfAtan2 * out.x_JJt2x2 * out.x_jacobianOfAtan2';
            
end