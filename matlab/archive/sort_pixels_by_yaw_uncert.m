function [out] = sort_pixels_by_yaw_uncert(ugvStereo, ugvMatlab)

out.yaw_cov_sorted = sort(ugvMatlab.yaw_cov);
A = sortrows([ugvMatlab.yaw_cov ugvStereo.Red.left.xy]);  out.Red.left.xy = A(:,2:3); clear A
A = sortrows([ugvMatlab.yaw_cov ugvStereo.Red.right.xy]); out.Red.right.xy = A(:,2:3); clear A
A = sortrows([ugvMatlab.yaw_cov ugvStereo.Blue.left.xy]);  out.Blue.left.xy = A(:,2:3); clear A
A = sortrows([ugvMatlab.yaw_cov ugvStereo.Blue.right.xy]); out.Blue.right.xy = A(:,2:3); clear A
A = sortrows([ugvMatlab.yaw_cov ugvStereo.Green.left.xy]);  out.Green.left.xy = A(:,2:3); clear A
A = sortrows([ugvMatlab.yaw_cov ugvStereo.Green.right.xy]); out.Green.right.xy = A(:,2:3); clear A

end