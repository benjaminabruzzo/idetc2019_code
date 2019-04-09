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


% calc_data = pixelsTo3D(data.ugvStereo.Red, data.ugvStereo.Blue, data.ugvStereo.Green, data.ugvStereo.Baseline, data.ugvStereo.FocalLength, data.ugvStereo.cam2ugv)


% function [data_out]= pixelsTo3D(Red, Blue, Green, b, f, cam2ugv)
function [data_out]= pixelsTo3D(ugvStereo_in)
%% Calculates 3d positions given rectified pixel correspondences
% RedLEDs, BlueLEDs, GreenLEDs are vectors of correspondences
% f is focal length
% P is projection matrix
% cam2tb is rotation converting points in camera frame to points in tb frame


    Red = ugvStereo_in.Red;
    Blue = ugvStereo_in.Blue;
    Green = ugvStereo_in.Green;
    
    b = ugvStereo_in.Baseline;
    f = ugvStereo_in.FocalLength;
    cam2ugv = ugvStereo_in.cam2ugv;

    % Camera frame positions of vectors
    RedVec_cam = calcvec(Red.left, Red.right, b, f);
    BlueVec_cam = calcvec(Blue.left, Blue.right, b, f);
    GreenVec_cam = calcvec(Green.left, Green.right, b, f);

    % same vectors, but with an n pixel shift on the left-x pixel
    n=1;
    data_out.RedVecN_cam = nPixelError(Red.left, Red.right, b, f, n);
    data_out.BlueVecN_cam = nPixelError(Blue.left, Blue.right, b, f, n);
    data_out.GreenVecN_cam = nPixelError(Green.left, Green.right, b, f, n);

    
    uav_in_cam = (RedVec_cam+BlueVec_cam)./2;

    % Convert camera frame positions to tb frame positions   
    for i = 1:length(RedVec_cam)
      Red_ugv(i,:) = cam2ugv*RedVec_cam(i,:)';
      Blue_ugv(i,:) = cam2ugv*BlueVec_cam(i,:)';
      Green_ugv(i,:) = cam2ugv*GreenVec_cam(i,:)';
      uav_in_ugv(i,:) = cam2ugv*uav_in_cam(i,:)';

      data_out.RedN_ugv(i,:) = cam2ugv*data_out.RedVecN_cam(i,:)';
      data_out.BlueN_ugv(i,:) = cam2ugv*data_out.BlueVecN_cam(i,:)';
      data_out.GreenN_ugv(i,:) = cam2ugv*data_out.GreenVecN_cam(i,:)';

    end


    data_out.Red_cam = RedVec_cam;
    data_out.Blue_cam = BlueVec_cam;
    data_out.Green_cam = GreenVec_cam;

    data_out.Red_ugv = Red_ugv;
    data_out.Blue_ugv = Blue_ugv;
    data_out.Green_ugv = Green_ugv;
    
    data_out.uav_in_cam = uav_in_cam;
    data_out.uav_in_ugv = uav_in_ugv;

    
    data_out.Red_nDiff = bignorm(RedVec_cam - data_out.RedVecN_cam);
    data_out.Blue_nDiff = bignorm(RedVec_cam - data_out.RedVecN_cam);
    data_out.Green_nDiff = bignorm(RedVec_cam - data_out.RedVecN_cam);
    
%     plot_calc_data(data_out, ugvStereo_in);
end


function [vector] = nPixelError(left, right, b, f, n)
   xl = left.xy(:,1)+n;
   yl = left.xy(:,2);
   xr = right.xy(:,1);
   yr = right.xy(:,2);
   d = abs(xl - xr);
   vector(:,3) = b*f ./ d;
   vector(:,2) = vector(:,3) .*(yr+yl) /(2*f);
   vector(:,1) = -vector(:,3) .*(xr+xl) /(2*f);
   vector(:,4) = ones(1,length(vector));

end

function [vector] = calcvec(left, right, b, f)
   xl = left.xy(:,1);
   yl = left.xy(:,2);
   xr = right.xy(:,1);
   yr = right.xy(:,2);
   d = abs(xl - xr);
   vector(:,3) = b*f ./ d;
   vector(:,2) = vector(:,3) .*(yr+yl) /(2*f);
   vector(:,1) = -vector(:,3) .*(xr+xl) /(2*f);
   vector(:,4) = ones(1,length(vector));

end

function [norm_vec] = bignorm(big_vector)

[m ,n] = size(big_vector);

square_vec = zeros(m,1);

for i = 1:n
    square_vec = square_vec + big_vector(:,i).*big_vector(:,i);
end
norm_vec = sqrt(square_vec);



end


function plot_calc_data(data_out, ugvStereo_in)
%% figure(201); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(201); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,1,1)
        hold on
            try plot(ugvStereo_in.time, data_out.Red_ugv(:,1), 'r+', 'displayname', 'red-stereo-x-ugv'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
    subplot(3,1,2)
        hold on
            try plot(ugvStereo_in.time, data_out.Blue_ugv(:,1), 'b+', 'displayname', 'blue-stereo-x-ugv'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
    subplot(3,1,3)
        hold on
            try plot(ugvStereo_in.time, data_out.Green_ugv(:,1), 'g+', 'displayname', 'green-stereo-x-ugv'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
        xlabel('t [sec]')
%% figure(202); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(202); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,1,1)
        hold on
            try plot(ugvStereo_in.time, data_out.Red_ugv(:,1), 'r+', 'displayname', 'red-stereo-x-ugv'); catch; end
            try plot(ugvStereo_in.time, data_out.RedN_ugv(:,1), 'ro', 'displayname', 'redN-stereo-x-ugv'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
    subplot(3,1,2)
        hold on
            try plot(ugvStereo_in.time, data_out.Blue_ugv(:,1), 'b+', 'displayname', 'blue-stereo-x-ugv'); catch; end
            try plot(ugvStereo_in.time, data_out.BlueN_ugv(:,1), 'bo', 'displayname', 'blueN-stereo-x-ugv'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
    subplot(3,1,3)
        hold on
            try plot(ugvStereo_in.time, data_out.Green_ugv(:,1), 'g+', 'displayname', 'green-stereo-x-ugv'); catch; end
            try plot(ugvStereo_in.time, data_out.GreenN_ugv(:,1), 'g+', 'displayname', 'greenN-stereo-x-ugv'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
        xlabel('t [sec]')
%% figure(203); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(203); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
            try plot(ugvStereo_in.time, data_out.Red_nDiff, 'r+', 'displayname', 'npixel-diff'); catch; end
            try plot(ugvStereo_in.time, data_out.Blue_nDiff, 'b+', 'displayname', 'npixel-diff'); catch; end
            try plot(ugvStereo_in.time, data_out.Green_nDiff, 'g+', 'displayname', 'npixel-diff'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')



end

