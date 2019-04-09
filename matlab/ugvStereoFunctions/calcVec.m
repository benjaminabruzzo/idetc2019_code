function [V] = calcVec(left_x, left_y, right_x, right_y, baseline, focallength)
%% convert pixels to vector
xl = left_x;
yl = left_y;
xr = right_x;
yr = right_y;
disparity = abs(left_x - right_x);

V(4,1) = 1;
V(3,1) = baseline*focallength/disparity;
V(2,1) = V(3,1) * (yl+yr)/(2*focallength);
V(1,1) = -V(3,1) * (xl+xr)/(2*focallength);

end
%%   cv::Matx<double,3,3> calcJ(cv::Vec<double,4> Pix)
%   {
%     /*  Calculate Jacobian Matrix for one target
%      *  Pix := 4x1 vector
%      *  pix[0 1 2 3] = pix[rx ry lx ly]
%      *  xr = Pix[0]
%      *  yr = Pix[1]
%      *  xl = Pix[2]
%      *  yl = Pix[3]
%      */
%     cv::Matx33d J;
%     double disparity = (Pix[2] - Pix[0]);
%     // double disparity = std::abs(Pix[2] - Pix[0]);
%     J = cv::Matx33d(
%       // row 1
%         -(Baseline * Pix[0]) / (2 * disparity * disparity), // (-b xr / 2 d^2)
%          (Baseline * Pix[2]) / (2 * disparity * disparity), // (b xl / 2 d^2)
%          0, // 0
% 
%       // row 2
%        -(Baseline * Pix[3]) / (disparity * disparity), // - b yl / d^2
%         (Baseline * Pix[3]) / (disparity * disparity), //   b yl / d^2
%          Baseline / disparity, // b/d
% 
%       // row 3
%        -(Baseline * FocalLength) / (disparity * disparity), // -bf/d
%         (Baseline * FocalLength) / (disparity * disparity), // bf/d
%         0); //0
% 
%     return J;
%   }