function out = pixel_uncert_from_ugvStereo(ugvStereo)
%% Use ugvStereo data to recalculate the pixel uncertainties
    red_left_x = ugvStereo.Red.left.xy(:,1);   red_left_y  = ugvStereo.Red.left.xy(:,2);
    red_right_x = ugvStereo.Red.right.xy(:,1); red_right_y = ugvStereo.Red.right.xy(:,2);
    
    blue_left_x = ugvStereo.Blue.left.xy(:,1);   blue_left_y  = ugvStereo.Blue.left.xy(:,2);
    blue_right_x = ugvStereo.Blue.right.xy(:,1); blue_right_y = ugvStereo.Blue.right.xy(:,2);
    
    green_left_x = ugvStereo.Green.left.xy(:,1);   green_left_y  = ugvStereo.Green.left.xy(:,2);
    green_right_x = ugvStereo.Green.right.xy(:,1); green_right_y = ugvStereo.Green.right.xy(:,2);
    
    try cam2ugv = ugvStereo.cam2tb; catch; end
    try cam2ugv = ugvStereo.cam2ugv; catch; end
    try cam2ugv33 = cam2ugv(1:3,1:3); catch; end
    
    for i = 1:length(red_left_x)
        
        RedP_cam(i,:) = calcVec(red_left_x(i), red_left_y(i), red_right_x(i), red_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        RedP_ugv(i,:) = (cam2ugv * RedP_cam(i,:)')';
        BlueP_cam(i,:) = calcVec(blue_left_x(i), blue_left_y(i), blue_right_x(i), blue_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        BlueP_ugv(i,:) = (cam2ugv * BlueP_cam(i,:)')';
        GreenP_cam(i,:) = calcVec(green_left_x(i), green_left_y(i), green_right_x(i), green_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        GreenP_ugv(i,:) = (cam2ugv * GreenP_cam(i,:)')';

                          
        RedJ_cam(:,:,i) = calcJ(red_left_x(i), red_left_y(i), red_right_x(i), red_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        RedJJt_ugv(:,:,i) = cam2ugv33 * RedJ_cam(:,:,i)*RedJ_cam(:,:,i)' * cam2ugv33';
        BlueJ_cam(:,:,i) = calcJ(blue_left_x(i), blue_left_y(i), blue_right_x(i), blue_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        BlueJJt_ugv(:,:,i) = cam2ugv33 * BlueJ_cam(:,:,i)*BlueJ_cam(:,:,i)' * cam2ugv33';
        GreenJ_cam(:,:,i) = calcJ(green_left_x(i), green_left_y(i), green_right_x(i), green_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        GreenJJt_ugv(:,:,i) = cam2ugv33 * GreenJ_cam(:,:,i) *GreenJ_cam(:,:,i)' * cam2ugv33';

        cO = crossOrientation(RedP_ugv(i,1:3), BlueP_ugv(i,1:3), GreenP_ugv(i,1:3), ...
                            RedJJt_ugv(:,:,i), BlueJJt_ugv(:,:,i), GreenJJt_ugv(:,:,i));

        z_axis(i,:) = cO.z_axis;
        y_axis(i,:) = cO.y_axis;
        x_axis(i,:) = cO.x_axis;
        yaw_ugv(i,1) = cO.yaw_ugv;

        z_jacobianOfNorm(:,:,i) = cO.z_jacobianOfNorm;
        z_JJt(:,:,i) = cO.z_JJt;
        y_jacobianOfNorm(:,:,i) = cO.y_jacobianOfNorm;
        y_JJt(:,:,i) = cO.y_JJt;
        x_JJt(:,:,i) = cO.x_JJt;
        x_JJt2x2(:,:,i) = cO.x_JJt2x2;
        x_jacobianOfAtan2(i,:) = cO.x_jacobianOfAtan2;
        yaw_JJt(i,1) = cO.yaw_JJt;

    end
    out = yaw_JJt;
end
%% function out = calcVec(left_xy, right_xy, Baseline, FocalLength)
% convert pixels into vectors
%     disparity = abs(left_xy(1) - right_xy(1));
%     out = zeros(1,4);
% 
%     out(1,4) =  1;
%     out(1,3) =  (Baseline*FocalLength)/disparity;
%     out(1,2) =  out(1,3) * (left_xy(2)+right_xy(2)) / (2 * FocalLength);
%     out(1,1) = -out(1,3) * (left_xy(1)+right_xy(1)) / (2 * FocalLength);
% end
%% function J = calcJ(left_xy, right_xy, Baseline, FocalLength)
% convert pixels to jacobian
%     disparity = abs(left_xy(1) - right_xy(1));
%     J = zeros(3,3);
%     
%     J(1,1) = -(Baseline * right_xy(1)) / (2 * disparity * disparity);
%     J(1,2) = (Baseline * left_xy(1)) / (2 * disparity * disparity);
%     J(1,3) = 0;
%          
%     J(2,1) = -(Baseline * left_xy(2)) / (disparity * disparity);
%     J(2,2) = (Baseline * left_xy(2)) / (disparity * disparity);
%     J(2,3) = Baseline / disparity;
% 
%     J(3,1) = -(Baseline * FocalLength) / (disparity * disparity);
%     J(3,2) = (Baseline * FocalLength) / (disparity * disparity);
%     J(3,3) = 0;
% 
% end
%% function out = crossOrientation(RedP, BlueP, GreenP, RedJJt, BlueJJt, GreenJJt)
% compute orientation of UAV from vectors in ugv frame
%     GmB = GreenP - BlueP;
%     RmB =   RedP - BlueP;
%     BmR =  BlueP - RedP;
%     RmG =   RedP - GreenP;
%  
%     znum = cross(GmB, RmB);
%     z_axis = znum/norm(znum);
%     y_axis = BmR / norm(BmR);
%     x_axis = cross(y_axis, z_axis);
% 
%     out.yaw_ugv = atan2(x_axis(1, 2), x_axis(1, 1)); %//heading angle in radians in ugv frame
%     
%     out.z_axis = z_axis;
%     out.y_axis = y_axis;
%     out.x_axis = x_axis;    
%     
% propagate covariances from JJt to yaw
%     GandB_JJt = GreenJJt + BlueJJt;
%     RandB_JJt = RedJJt + BlueJJt;
%     
%     z_JJt = wedgevector(BmR) * GandB_JJt * wedgevector(BmR)' + ...
%             wedgevector(GmB) * RandB_JJt * wedgevector(GmB)';
% 
%     out.z_jacobianOfNorm = jacobianOfNorm(znum);
%     out.z_JJt = out.z_jacobianOfNorm * z_JJt * out.z_jacobianOfNorm';
%     
%     out.y_jacobianOfNorm = jacobianOfNorm(BmR);
%     out.y_JJt = out.y_jacobianOfNorm * RandB_JJt * out.y_jacobianOfNorm';
%     
%     out.x_JJt = wedgevector(-z_axis) * out.y_JJt * wedgevector(-z_axis)' + ...
%                 wedgevector( y_axis) * out.z_JJt * wedgevector( y_axis)';
% 
%     out.x_JJt2x2 = out.x_JJt(1:2,1:2);
%     out.x_jacobianOfAtan2 = jacobianOfAtan2(x_axis(1,1),x_axis(1,2));
%     
%     out.yaw_JJt = out.x_jacobianOfAtan2 * out.x_JJt2x2 * out.x_jacobianOfAtan2';
%             
% end
%% function wV = wedgevector(Vt)
% V= vector4x1
% V = Vt';
% wV = [0      V(3,1) -V(2,1); ...
%      -V(3,1) 0       V(1,1); ...
%      V(2,1) -V(1,1)  0];
% end
%% function JofNorm = jacobianOfNorm(pt)
%     p = pt';
%     x = p(1,1);
%     y = p(2,1); 
%     z = p(3,1);
%     A = 1/(x^2+y^2+z^2)^(1.5);
%     B = [...
%      y*y+z*z,     -x*y,    -x*z;...
%         -x*y,  x*x+z*z,    -y*z;...
%         -x*z,     -y*z, x*x+y*y];
%     JofNorm = A*B;
% 
% end
%% function px = jacobianOfAtan2(a, b)
%     A = 1 / (a*a+b*b);
%     px = A*[-b a];
% end
