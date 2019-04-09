% 	cv::Matx<double,4,1> calcVec(cv::Vec<double,4> Pix)
% 	{
% 		cv::Matx<double,4,1> Vecpix;
% 		double disparity = std::abs(Pix[2] - Pix[0]);
% 		// w = 0; 4th dimension veci[x y z w]
% 		Vecpix(3,0) = 1;
% 		// z position
% 		Vecpix(2,0) = (Baseline*FocalLength) / disparity;
% 		//Non-Matthies left-y
% 		Vecpix(1,0) =  Vecpix(2,0) * (Pix[1] + Pix[3] ) / ( 2 * FocalLength );
% 		// x values are negative becuase of the way tirangulation is derived
% 		Vecpix(0,0) = -Vecpix(2,0) * (Pix[0] + Pix[2] ) / ( 2 * FocalLength );		
% 		return Vecpix;
%    }


% 	 *  pix[0 1 2 3] = pix[rx ry lx ly]
% 	 *  xr = Pix[0]
% 	 *  yr = Pix[1]
% 	 *  xl = Pix[2]
% 	 *  yl = Pix[3]

function [droneVec_tb droneVec_cam]= calc3D(RedLEDs, BlueLEDs, GreenLEDs, f, b, cxy)
%% Calculates 3d positions given rectified pixel correspondences
% RedLEDs, BlueLEDs, GreenLEDs are Nx4 matrixes of correspondences
% f is focal length
% P is projection matrix


   Cx = 337.947754;
   Cy = 244.307550;
   Cx = 330;
   Cy = 240;
%    RedLEDs = [(RedLEDs(:,1) + Cx) (Cy + RedLEDs(:,2)) (RedLEDs(:,3) + Cx) (Cy + RedLEDs(:,4))];
%    BlueLEDs = [(BlueLEDs(:,1) + Cx) (Cy + BlueLEDs(:,2)) (BlueLEDs(:,3) + Cx) (Cy + BlueLEDs(:,4))];
%    GreenLEDs = [(GreenLEDs(:,1) + Cx) (Cy + GreenLEDs(:,2)) (GreenLEDs(:,3) + Cx) (Cy + GreenLEDs(:,4))];
   
   LeftCx = 322.297941;
   LeftCy = 233.094001;
   RightCx = 324.380559;
   RightCy = 233.090969;
   
   Wedge = 20-90; %//degrees of blue wedge supporting cameras
   Pi = pi;
   camZOffset = 0.225; % //meters [m]
   camXOffset = 0.055; %//meters [m]
   cw  = cos(Wedge*Pi/180.0);
   sw  = sin(Wedge*Pi/180.0);
   
   % Camera frame positions of vectors
   RedVec_cam = calcvec(RedLEDs, b, f);
   BlueVec_cam = calcvec(BlueLEDs, b, f);
   GreenVec_cam = calcvec(GreenLEDs, b, f);

   droneVec_cam = (RedVec_cam+BlueVec_cam)./2;

   % rotation converting points in camera frame to points in tb frame
   cam2tb = [	0, -cw,-sw, camXOffset;
               1,   0,  0, 0;
               0, -sw, cw, camZOffset;
               0,   0,  0, 1];
   % Convert camera frame positions to tb frame positions   
   for i = 1:length(RedVec_cam)
      droneVec_tb(i,:) = cam2tb*droneVec_cam(i,:)';
   end

end


function [vector, LR] = calcvec(fourpixels, b, f)
   xr = fourpixels(:,1);
   yr = fourpixels(:,2);
   xl = fourpixels(:,3);
   yl = fourpixels(:,4);
   d = abs(xl - xr);
   vector(:,3) = b*f ./ d;
   vector(:,2) = vector(:,3) .*(yr+yl) /(2*f);
%    vector(:,1) = -vector(:,3) .*(xr) /(f);
   vector(:,1) = -vector(:,3) .*(xr+xl) /(2*f);
   vector(:,4) = ones(1,length(vector));

end


