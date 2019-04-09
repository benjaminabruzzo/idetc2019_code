function out = matlabStereo(ugvStereo, vicon, out)
% data.matlabStereo = matlabStereo(data.ugvStereo, data.matlabStereo);
azimuth = out.azimuth{1};
elevation = out.elevation{1};
%% Populate consistent fields:
out.time = ugvStereo.time;
out.cam2ugv = ugvStereo.cam2ugv;
out.Projection = ugvStereo.Projection;

P_l = ugvStereo.Projection.Left;
P_r = ugvStereo.Projection.Right;

%% Use ugvStereo data to recalculate the pixel uncertainties
    Cx = ugvStereo.Original.Cx;
    Cy = ugvStereo.Original.Cy;

    red_left_x = ugvStereo.Red.left.rawxy(:,1) - Cx;   red_left_y  = Cy - ugvStereo.Red.left.rawxy(:,2);
    red_right_x = ugvStereo.Red.right.rawxy(:,1) - Cx; red_right_y = Cy - ugvStereo.Red.right.rawxy(:,2);
    
    blue_left_x = ugvStereo.Blue.left.rawxy(:,1) - Cx;   blue_left_y  = Cy - ugvStereo.Blue.left.rawxy(:,2);
    blue_right_x = ugvStereo.Blue.right.rawxy(:,1) - Cx; blue_right_y = Cy - ugvStereo.Blue.right.rawxy(:,2);
    
    green_left_x = ugvStereo.Green.left.rawxy(:,1) - Cx;   green_left_y  = Cy - ugvStereo.Green.left.rawxy(:,2);
    green_right_x = ugvStereo.Green.right.rawxy(:,1) - Cx; green_right_y = Cy - ugvStereo.Green.right.rawxy(:,2);

    out.Red.left.xy = [red_left_x red_left_y];       out.Red.right.xy   = [red_right_x red_right_y];
    out.Blue.left.xy = [blue_left_x blue_left_y];    out.Blue.right.xy  = [blue_right_x blue_right_y];
    out.Green.left.xy = [green_left_x green_left_y]; out.Green.right.xy = [green_right_x green_right_y];
    
%     try cam2ugv = ugvStereo.cam2tb; catch; end
%     try cam2ugv = ugvStereo.cam2ugv; catch; end
%     try cam2ugv33 = cam2ugv(1:3,1:3); catch; end
    

%     azimuth = -1.15;
%     elevation = -1.6;
    wedge = 20+elevation;

%     try 
        t = ugvStereo.cam2ugv(1:3,4);
        cam2ugv33 = Rz(-(90-azimuth)*pi/180)*Rx(-(90-wedge)*pi/180);
        cam2ugv = [cam2ugv33 t; 0 0 0 1]; 
%     end
    
    
    for i = 1:length(red_left_x)
        
        Red.P_cam(i,:) = calcVec(red_left_x(i), red_left_y(i), red_right_x(i), red_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        Red.P_ugv(i,:) = (cam2ugv * Red.P_cam(i,:)')';
        Blue.P_cam(i,:) = calcVec(blue_left_x(i), blue_left_y(i), blue_right_x(i), blue_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        Blue.P_ugv(i,:) = (cam2ugv * Blue.P_cam(i,:)')';
        Green.P_cam(i,:) = calcVec(green_left_x(i), green_left_y(i), green_right_x(i), green_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        Green.P_ugv(i,:) = (cam2ugv * Green.P_cam(i,:)')';

                          
        RedJ_cam(:,:,i) = calcJ(red_left_x(i), red_left_y(i), red_right_x(i), red_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        RedJJt_ugv(:,:,i) = cam2ugv33 * RedJ_cam(:,:,i)*RedJ_cam(:,:,i)' * cam2ugv33';
        BlueJ_cam(:,:,i) = calcJ(blue_left_x(i), blue_left_y(i), blue_right_x(i), blue_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        BlueJJt_ugv(:,:,i) = cam2ugv33 * BlueJ_cam(:,:,i)*BlueJ_cam(:,:,i)' * cam2ugv33';
        GreenJ_cam(:,:,i) = calcJ(green_left_x(i), green_left_y(i), green_right_x(i), green_right_y(i), ugvStereo.Baseline, ugvStereo.FocalLength);
        GreenJJt_ugv(:,:,i) = cam2ugv33 * GreenJ_cam(:,:,i) *GreenJ_cam(:,:,i)' * cam2ugv33';

        cO = crossOrientation(Red.P_ugv(i,1:3), Blue.P_ugv(i,1:3), Green.P_ugv(i,1:3), ...
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
    out.uav.yaw_JJt = yaw_JJt;
    out.Red.P_cam = Red.P_cam;
    out.Blue.P_cam = Blue.P_cam;
    out.Green.P_cam = Green.P_cam;
    out.cam2ugv = cam2ugv;
    out.cam2ugv33 = cam2ugv33;
    out.Red.P_ugv = Red.P_ugv;
    out.Blue.P_ugv = Blue.P_ugv;
    out.Green.P_ugv = Green.P_ugv;
    
    
    %% Calculate camera-frame positions of markers from vicon data
    try
        H = invertH(cam2ugv);
        out.red_target.P.ugv = [vicon.red_target.P.ugv ones(length(vicon.red_target.P.ugv),1)];
        out.blue_target.P.ugv = [vicon.blue_target.P.ugv ones(length(vicon.red_target.P.ugv),1)];
        out.green_target.P.ugv = [vicon.green_target.P.ugv ones(length(vicon.red_target.P.ugv),1)];
        for i = 1:size(out.red_target.P.ugv,1)
            out.red_target.P.cam(i,:) = H * out.red_target.P.ugv(i,:)';
            out.blue_target.P.cam(i,:) = H * out.blue_target.P.ugv(i,:)';
            out.green_target.P.cam(i,:) = H * out.green_target.P.ugv(i,:)';
        end
        out.uav.P.cam = 0.5*(out.red_target.P.cam + out.blue_target.P.cam);

    catch
    end
    clear i P_blue P_green P_red H

%% Spline the cam positions of markers

    out.red_target.splines.P.cam.x = csapi(vicon.red_target.time,out.red_target.P.cam(:,1));
    out.red_target.splines.P.cam.y = csapi(vicon.red_target.time,out.red_target.P.cam(:,2));
    out.red_target.splines.P.cam.z = csapi(vicon.red_target.time,out.red_target.P.cam(:,3));
    out.red_target.splines.P.cam.x_ugvStereotime = fnval(out.red_target.splines.P.cam.x, ugvStereo.time);
    out.red_target.splines.P.cam.y_ugvStereotime = fnval(out.red_target.splines.P.cam.y, ugvStereo.time);
    out.red_target.splines.P.cam.z_ugvStereotime = fnval(out.red_target.splines.P.cam.z, ugvStereo.time);    
    out.red_target.splines.P_cam = [...
        out.red_target.splines.P.cam.x_ugvStereotime ...
        out.red_target.splines.P.cam.y_ugvStereotime ...
        out.red_target.splines.P.cam.z_ugvStereotime ...
        ones(length(out.red_target.splines.P.cam.x_ugvStereotime),1)];

    out.blue_target.splines.P.cam.x = csapi(vicon.blue_target.time,out.blue_target.P.cam(:,1));
    out.blue_target.splines.P.cam.y = csapi(vicon.blue_target.time,out.blue_target.P.cam(:,2));
    out.blue_target.splines.P.cam.z = csapi(vicon.blue_target.time,out.blue_target.P.cam(:,3));
    out.blue_target.splines.P.cam.x_ugvStereotime = fnval(out.blue_target.splines.P.cam.x, ugvStereo.time);
    out.blue_target.splines.P.cam.y_ugvStereotime = fnval(out.blue_target.splines.P.cam.y, ugvStereo.time);
    out.blue_target.splines.P.cam.z_ugvStereotime = fnval(out.blue_target.splines.P.cam.z, ugvStereo.time);
    out.blue_target.splines.P_cam = [...
        out.blue_target.splines.P.cam.x_ugvStereotime ...
        out.blue_target.splines.P.cam.y_ugvStereotime ...
        out.blue_target.splines.P.cam.z_ugvStereotime ...
        ones(length(out.red_target.splines.P.cam.x_ugvStereotime),1)];

    out.green_target.splines.P.cam.x = csapi(vicon.green_target.time,out.green_target.P.cam(:,1));
    out.green_target.splines.P.cam.y = csapi(vicon.green_target.time,out.green_target.P.cam(:,2));
    out.green_target.splines.P.cam.z = csapi(vicon.green_target.time,out.green_target.P.cam(:,3));
    out.green_target.splines.P.cam.x_ugvStereotime = fnval(out.green_target.splines.P.cam.x, ugvStereo.time);
    out.green_target.splines.P.cam.y_ugvStereotime = fnval(out.green_target.splines.P.cam.y, ugvStereo.time);
    out.green_target.splines.P.cam.z_ugvStereotime = fnval(out.green_target.splines.P.cam.z, ugvStereo.time);
    out.green_target.splines.P_cam = [...
        out.green_target.splines.P.cam.x_ugvStereotime ...
        out.green_target.splines.P.cam.y_ugvStereotime ...
        out.green_target.splines.P.cam.z_ugvStereotime ...
        ones(length(out.red_target.splines.P.cam.x_ugvStereotime),1)];
    
  %% Map spline positions to pixels
    % red_left_x = - Cx + ugvStereo.Red.left.rawxy(:,1) ;   
    % red_left_y  = Cy - ugvStereo.Red.left.rawxy(:,2);
    Cx = ugvStereo.Original.Cx;
    Cy = ugvStereo.Original.Cy;

    for i = 1:length(out.red_target.splines.P_cam)
        uvw_l = ugvStereo.Projection.Left*out.red_target.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
        uvw_r = ugvStereo.Projection.Right*out.red_target.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
        out.red_target.left.rawxy(i,:) =  uvw_l;
        out.red_target.right.rawxy(i,:) = uvw_r;
        out.red_target.left.xy(i,:) =  [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
        out.red_target.right.xy(i,:) = [(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
        clear uvw_l uvw_r

        uvw_l = ugvStereo.Projection.Left*out.blue_target.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
        uvw_r = ugvStereo.Projection.Right*out.blue_target.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
        out.blue_target.left.rawxy(i,:) =  uvw_l;
        out.blue_target.right.rawxy(i,:) = uvw_r;
        out.blue_target.left.xy(i,:) = [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
        out.blue_target.right.xy(i,:) =[(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
        clear uvw_l uvw_r

        uvw_l = ugvStereo.Projection.Left*out.green_target.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
        uvw_r = ugvStereo.Projection.Right*out.green_target.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
        out.green_target.left.rawxy(i,:) =  uvw_l;
        out.green_target.right.rawxy(i,:) = uvw_r;
        out.green_target.left.xy(i,:) = [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
        out.green_target.right.xy(i,:) =[(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
        clear uvw_l uvw_r

    end
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
