function [out]= ugvStereoPostProc(ugvStereo)
%% data.ugvStereo.Red.left.xy

    try 
        camparams = ugvStereo.Original; 
    catch
        camparams = ugvStereo.pgryaml;
    end
    
    camparams.H_cam2lo = ugvStereo.cam2ugv;
    camparams.R_cam2lo = camparams.H_cam2lo(1:3,1:3);

   % Camera frame positions of vectors
   out.Red = calcvec(ugvStereo.Red, camparams);
   out.Blue = calcvec(ugvStereo.Blue, camparams);
   out.Green = calcvec(ugvStereo.Green, camparams);

  % compute uav orientation
    out.uav.p_cam = (out.Red.p_cam+out.Blue.p_cam)/2;
    for i = 1:length(out.uav.p_cam)
        out.uav.p_ugv(i,:) = (camparams.H_cam2lo * out.uav.p_cam(i,:)')';
    end
    out.uav.p_ugv_JJt = (out.Red.JJt_ugv + out.Blue.JJt_ugv)/2;
    out = crossOrientation(out);

    out.params = camparams;
end

% data.ugvStereo.Original
%     RightOffset: -101.3446
%              Cy: 229.3841
%              Cx: 326.6189
%        Baseline: 0.1493
%     FocalLength: 683.5781


% data.ugvStereo.pgryaml
%             RightOffset: -156.7691
%                      Cy: 486.3225
%                      Cx: 562.7035
%                Baseline: 0.1447
%             FocalLength: 1.0321e+03

%      left_shutter_speed: 0.0020
%             left_wb_red: 270
%            left_wb_blue: 525
%               left_gain: 8.3838
%     right_shutter_speed: 0.0020
%            right_wb_red: 270
%           right_wb_blue: 525
%              right_gain: 8.3838
%              camXOffset: 0
%              camYOffset: 0
%              camZOffset: 0.2200
%             image_width: 1288
%            image_height: 964
%                   Wedge: 20


function [out] = calcvec(pixels, params)
%% convert 2P2 to R3
    b = params.Baseline;
    f = params.FocalLength;

    xl = pixels.left.xy(:,1);
    yl = pixels.left.xy(:,2);
    xr = pixels.right.xy(:,1);
    yr = pixels.right.xy(:,2);

    d = abs(xl - xr);
    for m = 1:length(d)
        out.p_cam(m,4) =  1;
        out.p_cam(m,3) =  b*f / d(m);
        out.p_cam(m,2) =  out.p_cam(m,3) *(yr(m)+yl(m)) /(2*f);
        out.p_cam(m,1) = -out.p_cam(m,3) *(xr(m)+xl(m)) /(2*f);
        
        out.p_ugv(m,:) = params.H_cam2lo * out.p_cam(m,:)';
        out.J_cam(:,:,m) = calcJ(pixels.left.xy(m,:), pixels.right.xy(m,:), b, f);
        out.JJt_cam(:,:,m) = out.J_cam(:,:,m) * out.J_cam(:,:,m)';
        out.JJt_ugv(:,:,m) = params.R_cam2lo * out.JJt_cam(:,:,m) * params.R_cam2lo';

    end
end

function [J] = calcJ(left, right, b, f)
   xl = left(1);
   yl = left(2);
   xr = right(1);
   yr = right(2);

  d = abs(xl - xr);
  dd = d*d;
  %column 1                     % column 2                  % column 3
  J(1,1) = (-b * xr) / (2*dd);  J(1,2) = (b * xl) / (2*dd); J(1,3) = 0;
  J(2,1) = (-b * yl )/ dd;      J(2,2) = (b * yl) / dd;     J(2,3) = b/d;
  J(3,1) = (-b * f)  / dd;      J(3,2) = (b * f)  / dd;     J(3,3) = 0;
  
end

function [out] = crossOrientation(out)
% calculate relative vectors
    GmB = out.Green.p_ugv - out.Blue.p_ugv;
    GmB_JJt = out.Green.JJt_ugv + out.Blue.JJt_ugv;
    out.uav.GmB = GmB;
    out.uav.GmB_JJt = GmB_JJt;
    
    RmB = out.Red.p_ugv - out.Blue.p_ugv;
    RmB_JJt = out.Red.JJt_ugv + out.Blue.JJt_ugv;
    out.uav.RmB = RmB;
    out.uav.RmB_JJt = RmB_JJt;

    BmR = out.Blue.p_ugv - out.Red.p_ugv;
    BmR_JJt = out.Blue.JJt_ugv + out.Red.JJt_ugv;
    out.uav.BmR = BmR;
    out.uav.BmR_JJt = BmR_JJt;    
    
    for i = 1:length(GmB)
        % uav 'up'
        out.uav.z.axis(i,:) = cross(GmB(i,1:3), RmB(i,1:3));
        out.uav.z.gradofnorm(:,:,i) = gradofnorm(out.uav.z.axis(i,:));
        out.uav.z.gradofcross(:,:,i) = [-wedgevector(RmB(i,1:3)) wedgevector(GmB(i,1:3))];
        out.uav.z.axis_JJt(:,:,i) = ...
            out.uav.z.gradofcross(:,:,i) * ...
            [GmB_JJt(:,:,i) zeros(3); zeros(3) BmR_JJt(:,:,i)] * ...
            out.uav.z.gradofcross(:,:,i)';
        out.uav.z.axis_JJt(:,:,i) = out.uav.z.gradofnorm(:,:,i) * out.uav.z.axis_JJt(:,:,i) * out.uav.z.gradofnorm(:,:,i)';
        out.uav.z.axis(i,:) = out.uav.z.axis(i,:)/norm(out.uav.z.axis(i,:));

        % uav 'left'
        out.uav.y.axis(i,:) = BmR(i,1:3);
        out.uav.y.axis_JJt(:,:,i) = gradofnorm(out.uav.y.axis(i,:)) * BmR_JJt(:,:,i) * gradofnorm(out.uav.y.axis(i,:))';
        out.uav.y.gradofnorm(:,:,i) = gradofnorm(out.uav.y.axis(i,:));
        out.uav.y.axis(i,:) = out.uav.y.axis(i,:) / norm(out.uav.y.axis(i,:));

        % uav 'fore'
        out.uav.x.axis(i,:) = cross(out.uav.y.axis(i,:), out.uav.z.axis(i,:));
        out.uav.x.gradofcross(:,:,i) = [-wedgevector(out.uav.z.axis(i,:)) wedgevector(out.uav.y.axis(i,:))];
        out.uav.x.axis_JJt(:,:,i) = ...
            out.uav.x.gradofcross(:,:,i) * ...
            [out.uav.y.axis_JJt(:,:,i) zeros(3); zeros(3) out.uav.z.axis_JJt(:,:,i)] * ...
            out.uav.x.gradofcross(:,:,i)';

        % uav 'yaw'
        x = out.uav.x.axis(i,1);
        y = out.uav.x.axis(i,2);
        out.uav.yaw.ugv_rad(i,1) = atan2(y, x);
        out.uav.yaw.ugv_deg(i,1) = out.uav.yaw.ugv_rad(i,1) * 180 / pi; %heading angle in degrees in ugv frame
        out.uav.yaw.ugv_JJt(i,1) = gradofatan2(x,y) * out.uav.x.axis_JJt(1:2,1:2,i) * gradofatan2(x,y)';
        out.uav.yaw.gradofatan(i,:) = gradofatan2(x,y);

    end
    
    out.uav.yaw_var = out.uav.yaw.ugv_JJt;

end


function Vx = wedgevector(V)
%% Vx is only generated by the first three elements of V
    try 
        Vx = [0     -V(3,1) V(2,1);...
              V(3,1) 0     -V(1,1);...
             -V(2,1) V(1,1) 0];
    catch
        Vx = [0     -V(3) V(2);...
              V(3) 0     -V(1);...
             -V(2) V(1) 0];
    end
end

function px = gradofnorm(p)
%% px is composed of only the first three elemnts of V
x = p(1);
y = p(2);
z = p(3);


A = 1 / (x^2+y^2+z^2)^(3/2);
B(1,1) =  y^2 + z^2;
B(2,1) = -x*y;
B(3,1) = -x*z;
B(1,2) = -x*y;
B(2,2) = x^2+z^2;
B(3,2) = -y*z;
B(1,3) = -x*z;
B(2,3) = -y*z;
B(3,3) = x^2+y^2;

px = A*B;


end

function px = gradofatan2(x,y)
%% px is composed of only the first three elemnts of V
A = 1/(x^2+y^2);
px = A*[-y x];

end
