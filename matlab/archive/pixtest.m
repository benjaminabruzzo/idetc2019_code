function [out]= pixtest(RedPixels, BluePixels, GreenPixels)
% data.ugvStereo.Red.left.xy

    out.Red.left = RedPixels.left;
    out.Red.right = RedPixels.right;
    out.Blue.left = BluePixels.left;
    out.Blue.right = BluePixels.right;
    out.Green.left = GreenPixels.left;
    out.Green.right = GreenPixels.right;

    f = 694.3584;
    b = 0.1537;
%     f = 700;
%     b = 0.155;
    out.f = f;
    out.b = b;

   Wedge = 20-90; %//degrees of blue wedge supporting cameras
   Pi = pi;
   camZOffset = 0.24; % //meters [m]
   camXOffset = 0.075; %//meters [m]
   cw  = cos(Wedge*Pi/180.0);
   sw  = sin(Wedge*Pi/180.0);

   % Camera frame positions of vectors
   out.Red.p_cam = calcvec(RedPixels, b, f);
   out.Blue.p_cam = calcvec(BluePixels, b, f);
   out.Green.p_cam = calcvec(GreenPixels, b, f);


   % rotation converting points in camera frame to points in tb frame
   cam2lo = [	0, -cw,-sw, camXOffset;
               1,   0,  0, 0;
               0, -sw, cw, camZOffset;
               0,   0,  0, 1];
    out.H.cam2lo = cam2lo;
   % Convert camera frame positions to tb frame positions
    out.Red.p_ugv = cam2lo * out.Red.p_cam;
    out.Blue.p_ugv = cam2lo * out.Blue.p_cam;
    out.Green.p_ugv = cam2lo * out.Green.p_cam;

  % compute jacobians
    out.Red.J_cam = calcJ(RedPixels, b, f);
    out.Blue.J_cam = calcJ(BluePixels, b, f);
    out.Green.J_cam = calcJ(GreenPixels, b, f);
    
    out.Red.JJt_cam = out.Red.J_cam * out.Red.J_cam';
    out.Blue.JJt_cam = out.Blue.J_cam * out.Blue.J_cam';
    out.Green.JJt_cam = out.Green.J_cam * out.Green.J_cam';

    out.Red.J_ugv = cam2lo(1:3,1:3) * out.Red.J_cam * cam2lo(1:3,1:3)';
    out.Blue.J_ugv = cam2lo(1:3,1:3) * out.Blue.J_cam * cam2lo(1:3,1:3)';
    out.Green.J_ugv = cam2lo(1:3,1:3) * out.Green.J_cam * cam2lo(1:3,1:3)';

    out.Red.JJt_ugv = cam2lo(1:3,1:3) * out.Red.JJt_cam * cam2lo(1:3,1:3)';
    out.Blue.JJt_ugv = cam2lo(1:3,1:3) * out.Blue.JJt_cam * cam2lo(1:3,1:3)';
    out.Green.JJt_ugv = cam2lo(1:3,1:3) * out.Green.JJt_cam * cam2lo(1:3,1:3)';

  % compute UAV orientation
    out.UAV.p_cam = (out.Red.p_cam+out.Blue.p_cam)/2;
    out.UAV.p_ugv = cam2lo * out.UAV.p_cam;
    out.UAV.p_ugv_JJt = (out.Red.JJt_ugv + out.Blue.JJt_ugv)/2;
    out = crossOrientation(out);

end


function [p_cam] = calcvec(pixels, b, f)
  %% convert 2P2 to R3
    try    
        xl = pixels.left.x;
        yl = pixels.left.y;
        xr = pixels.right.x;
        yr = pixels.right.y;
    catch end

    try 
       xl = pixels.left(1);
       yl = pixels.left(2);
       xr = pixels.right(1);
       yr = pixels.right(2);
    catch
    end

    d = abs(xl - xr);
    p_cam(4,1) =  1;
    p_cam(3,1) =  b*f / d;
    p_cam(2,1) =  p_cam(3,1) *(yr+yl) /(2*f);
    p_cam(1,1) = -p_cam(3,1) *(xr+xl) /(2*f);
end

function [out] = crossOrientation(out)
% calculate relative vectors
    GmB = out.Green.p_ugv(1:3) - out.Blue.p_ugv(1:3);
    GmB_JJt = out.Green.JJt_ugv + out.Blue.JJt_ugv;
    out.UAV.GmB = GmB;
    out.UAV.GmB_JJt = GmB_JJt;
    
    RmB = out.Red.p_ugv(1:3) - out.Blue.p_ugv(1:3);
    RmB_JJt = out.Red.JJt_ugv + out.Blue.JJt_ugv;
    out.UAV.RmB = RmB;
    out.UAV.RmB_JJt = RmB_JJt;

    BmR = out.Blue.p_ugv(1:3) - out.Red.p_ugv(1:3);
    BmR_JJt = out.Blue.JJt_ugv + out.Red.JJt_ugv;
    out.UAV.BmR = BmR;
    out.UAV.BmR_JJt = BmR_JJt;    

% UAV 'up'
    out.UAV.z.axis = cross(GmB, RmB);
    out.UAV.z.gradofcross = [-wedgevector(RmB) wedgevector(GmB)];
    out.UAV.z.axis_JJt = ...
        [-wedgevector(RmB) wedgevector(GmB)] * ...
        [GmB_JJt zeros(3); zeros(3) BmR_JJt] * ...
        [-wedgevector(RmB) wedgevector(GmB)]';
    out.UAV.z.axis_JJt = gradofnorm(out.UAV.z.axis) * out.UAV.z.axis_JJt * gradofnorm(out.UAV.z.axis)';
    out.UAV.z.gradofnorm = gradofnorm(out.UAV.z.axis);
    out.UAV.z.axis = out.UAV.z.axis/norm(out.UAV.z.axis);

    out.UAV.y.axis = BmR;
    out.UAV.y.axis_JJt = gradofnorm(out.UAV.y.axis) * BmR_JJt * gradofnorm(out.UAV.y.axis)';
    out.UAV.y.gradofnorm = gradofnorm(out.UAV.y.axis);
    out.UAV.y.axis = BmR / norm(BmR);

    out.UAV.x.axis = cross(out.UAV.y.axis, out.UAV.z.axis);
    out.UAV.x.gradofcross = [-wedgevector(out.UAV.z.axis) wedgevector(out.UAV.y.axis)];
    out.UAV.x.axis_JJt = ...
        [-wedgevector(out.UAV.z.axis) wedgevector(out.UAV.y.axis)] * ...
        [out.UAV.y.axis_JJt zeros(3); zeros(3) out.UAV.z.axis_JJt] * ...
        [-wedgevector(out.UAV.z.axis) wedgevector(out.UAV.y.axis)]';


    x = out.UAV.x.axis(1, 1);
    y = out.UAV.x.axis(2, 1);
    out.UAV.yaw.tb_rad = atan2(y, x);
    out.UAV.yaw.tb_deg = out.UAV.yaw.tb_rad * 180 / pi; %heading angle in degrees in tb frame
    out.UAV.yaw.tb_JJt = gradofatan2(x,y) * out.UAV.x.axis_JJt(1:2,1:2) * gradofatan2(x,y)';
    out.UAV.yaw.gradofatan = gradofatan2(x,y);
    

end

function [J] = calcJ(pixels, b, f)
    try    
        xl = pixels.left.x;
        yl = pixels.left.y;
        xr = pixels.right.x;
        yr = pixels.right.y;
    catch 
    end

    try 
       xl = pixels.left(1);
       yl = pixels.left(2);
       xr = pixels.right(1);
       yr = pixels.right(2);
    catch
    end

  d = abs(xl - xr);
  dd = d*d;
  %column 1                     % column 2                  % column 3
  J(1,1) = (-b * xr) / (2*dd);  J(1,2) = (b * xl) / (2*dd); J(1,3) = 0;
  J(2,1) = (-b * yl )/ dd;      J(2,2) = (b * yl) / dd;     J(2,3) = b/d;
  J(3,1) = (-b * f)  / dd;      J(3,2) = (b * f)  / dd;     J(3,3) = 0;
  
end

function JJt = yawJ(out)
  z = cross(out.Green.p_ugv(1:3) - out.Blue.p_ugv(1:3), out.Red.p_ugv(1:3) - out.Blue.p_ugv(1:3));
  normz = norm(z);

  Rx = out.Red.p_ugv(1,1);
  Ry = out.Red.p_ugv(2,1);
  Rz = out.Red.p_ugv(3,1);

  Bx = out.Blue.p_ugv(1,1);
  By = out.Blue.p_ugv(2,1);
  Bz = out.Blue.p_ugv(3,1);

  Gx = out.Green.p_ugv(1,1);
  Gy = out.Green.p_ugv(2,1);
  Gz = out.Green.p_ugv(3,1);

  Jcross = [0 (Bz-Gz) (Gy-By)   0 (Gz-Rz) (Ry-Gy)   0 (Rz-Bz) (By-Ry);...
            (Gz-Bz) 0 (Bx-Gx)   (Rz-Gz) 0 (Gx-Rx)   (Bz-Rz) 0 (Rx-Bx);...
            (By-Gy) (Gx-Bx) 0   (Gy-Ry) (Rx-Gx) 0   (Ry-By) (Bx-Rx) 0];

  J9x9 = [out.Red.JJt zeros(3) zeros(3);...
          zeros(3) out.Blue.JJt zeros(3);...
          zeros(3) zeros(3) out.Green.JJt];

  JJt = (1/normz^2) * Jcross * J9x9 * Jcross';
end


function Vx = wedgevector(V)
%% Vx is only generated by the first three elements of V
Vx = [0     -V(3,1) V(2,1);...
      V(3,1) 0     -V(1,1);...
     -V(2,1) V(1,1) 0];

end

function px = gradofnorm(p)
%% px is composed of only the first three elemnts of V
x = p(1,1);
y = p(2,1);
z = p(3,1);


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
