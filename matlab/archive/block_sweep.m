function [out] = block_sweep(block_elevation, block_azimuth, vicon, ugvStereo)
%% [out] = block_sweep(loopout.run_012.vicon.red_target.P.ugv, loopout.run_012.vicon.blue_target.P.ugv,...
%     loopout.run_012.vicon.green_target.P.ugv, loopout.run_012.ugvStereo.cam2ugv)

% ugvStereo.Original.Cx
% ugvStereo.Original.Cy
% ugvStereo.Red.right.rawxy


Red_ugv = vicon.red_target.P.ugv;
Blue_ugv = vicon.blue_target.P.ugv;
Green_ugv = vicon.green_target.P.ugv;


v = ugvStereo.cam2ugv(1:3,4);

%%

azimuth = block_azimuth{1};

for i = 1:size(block_elevation,1)
%     disp(['i: ' num2str(i)])
    theta = 20 +block_elevation{i};
%     disp(['theta = ' num2str(theta)])
    HR = Rx((90-theta)*pi/180)*Rz((90-azimuth)*pi/180);
    H = [HR -HR*v; 0 0 0 1];
    % compute vicon position of markers using above matrix
    [out.(matlab.lang.makeValidName(['block_' num2str(i)]))] = rotateData(H, Red_ugv, Blue_ugv, Green_ugv);
    % spline position data and match to stereo data
    [out.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target] = ...
        splineData(vicon.red_target.time, out.(matlab.lang.makeValidName(['block_' num2str(i)])).red_target.P.cam, ugvStereo.time, ugvStereo.Red.P_cam);
    [out.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target] = ...
        splineData(vicon.blue_target.time, out.(matlab.lang.makeValidName(['block_' num2str(i)])).blue_target.P.cam, ugvStereo.time, ugvStereo.Blue.P_cam);
    [out.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target] = ...
        splineData(vicon.green_target.time, out.(matlab.lang.makeValidName(['block_' num2str(i)])).green_target.P.cam, ugvStereo.time, ugvStereo.Green.P_cam);
    
    
end


end

function [out] = rotateData(H, Red_ugv, Blue_ugv, Green_ugv)
    P_red = [Red_ugv ones(length(Red_ugv),1)];
    P_blue = [Blue_ugv ones(length(Blue_ugv),1)];
    P_green = [Green_ugv ones(length(Green_ugv),1)];
    for i = 1:size(P_red,1)
        out.red_target.P.cam(i,:) = H * P_red(i,:)';
        out.blue_target.P.cam(i,:) = H * P_blue(i,:)';
        out.green_target.P.cam(i,:) = H * P_green(i,:)';
    end
    out.uav.P.cam = 0.5*(out.red_target.P.cam + out.blue_target.P.cam);
end

function [splines] = splineData(target_time, target_P_cam, ugvStereotime, ugvStereo_P_cam)
    
    splines.P.cam.x = csapi(target_time,target_P_cam(:,1));
    splines.P.cam.y = csapi(target_time,target_P_cam(:,2));
    splines.P.cam.z = csapi(target_time,target_P_cam(:,3));

    splines.P.cam.atugvStereotime(:,1) = fnval(splines.P.cam.x, ugvStereotime);
    splines.P.cam.atugvStereotime(:,2) = fnval(splines.P.cam.y, ugvStereotime);
    splines.P.cam.atugvStereotime(:,3) = fnval(splines.P.cam.z, ugvStereotime);

    splines.P.cam.diff(:,1) = splines.P.cam.atugvStereotime(:,1) - ugvStereo_P_cam(:,1);
    splines.P.cam.diff(:,2) = splines.P.cam.atugvStereotime(:,2) - ugvStereo_P_cam(:,2);
    splines.P.cam.diff(:,3) = splines.P.cam.atugvStereotime(:,3) - ugvStereo_P_cam(:,3);

    
    splines.P.cam.mean_diff = mean(splines.P.cam.diff);
    splines.P.cam.std_diff = std(splines.P.cam.diff);
    
    splines.P.rms = sqrt(splines.P.cam.diff(:,1).*splines.P.cam.diff(:,1)+...
        splines.P.cam.diff(:,2).*splines.P.cam.diff(:,2)+...
        splines.P.cam.diff(:,3).*splines.P.cam.diff(:,3));
    
    splines.P.rms_mean = mean(splines.P.rms);
    splines.P.rms_std = std(splines.P.rms);
   
    
end


function [] = testblock()
%%
[out]= block_sweep(loopout.block_elevation, loopout.run_012.vicon,loopout.run_012.ugvStereo )%...
%     loopout.run_012.vicon.red_target.P.ugv, ...
%     loopout.run_012.vicon.blue_target.P.ugv, ...
%     loopout.run_012.vicon.green_target.P.ugv, ...
%     loopout.run_012.ugvStereo.cam2ugv)
 
 %%
end