function [] = plot_matlabStereo(meta, data)
data.vicon.red_target.splines.P_ugv = [...
    data.vicon.red_target.splines.P.ugv.x_ugvStereotime ...
    data.vicon.red_target.splines.P.ugv.y_ugvStereotime ...
    data.vicon.red_target.splines.P.ugv.z_ugvStereotime];
data.vicon.blue_target.splines.P_ugv = [...
    data.vicon.blue_target.splines.P.ugv.x_ugvStereotime ...
    data.vicon.blue_target.splines.P.ugv.y_ugvStereotime ...
    data.vicon.blue_target.splines.P.ugv.z_ugvStereotime];
data.vicon.green_target.splines.P_ugv = [...
    data.vicon.green_target.splines.P.ugv.x_ugvStereotime ...
    data.vicon.green_target.splines.P.ugv.y_ugvStereotime ...
    data.vicon.green_target.splines.P.ugv.z_ugvStereotime];


data.matlabStereo.Red.P_ugv_diff = data.vicon.red_target.splines.P_ugv - data.matlabStereo.Red.P_ugv(:,1:3);
data.matlabStereo.Blue.P_ugv_diff = data.vicon.blue_target.splines.P_ugv - data.matlabStereo.Blue.P_ugv(:,1:3);
data.matlabStereo.Green.P_ugv_diff = data.vicon.green_target.splines.P_ugv - data.matlabStereo.Green.P_ugv(:,1:3);

%% figure(499); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(499); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,1,1)
        hold on
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Red.P_ugv_diff(:,1), 'r.', 'displayname', 'red x diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Blue.P_ugv_diff(:,1), 'b.', 'displayname', 'blue x diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Green.P_ugv_diff(:,1), 'g.', 'displayname', 'green x diff'); catch; end
        hold off
        grid on
        legend('toggle')
    subplot(3,1,2)
        hold on
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Red.P_ugv_diff(:,2), 'r.', 'displayname', 'red y diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Blue.P_ugv_diff(:,2), 'b.', 'displayname', 'blue y diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Green.P_ugv_diff(:,2), 'g.', 'displayname', 'green y diff'); catch; end
        hold off
        grid on
        legend('toggle')
    subplot(3,1,3)
        hold on
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Red.P_ugv_diff(:,3), 'r.', 'displayname', 'red z diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Blue.P_ugv_diff(:,3), 'b.', 'displayname', 'blue z diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Green.P_ugv_diff(:,3), 'g.', 'displayname', 'green z diff'); catch; end
        hold off
        grid on
        legend('toggle')


%%
block_elevation = {...
1.5;...
};

block_azimuth = {-0.9};

[angleblocks] = block_sweep(block_elevation, block_azimuth, data.vicon, data.matlabStereo);

% figure(500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.matlabStereo.Red.P_cam(:,3), data.matlabStereo.Red.P_cam(:,1), 'ro', 'displayname', 'red cam (ugvStereo-matlab)'); catch; end
        try plot(data.vicon.red_target.P.cam(:,3), data.vicon.red_target.P.cam(:,1), 'r.', 'displayname', 'red cam (data.vicon)'); catch; end        
        hold off
    grid on
    ylabel('x [m]')
    legend('toggle')

%% figure(500+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
for i = 1:size(block_elevation,1)
    figure(500+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,1,1)
        hold on
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,1), 'r', 'displayname', 'red x diff'); catch; end
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,1), 'b', 'displayname', 'blue x diff'); catch; end
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,1), 'g', 'displayname', 'green x diff'); catch; end
        hold off
        grid on
        ylabel('x [m]')
        legend('toggle')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0 10 20 30 40 50 60];
        current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
        current_axes.YTick = [-0.5:0.05:0.5];
        current_axes.YLim = [-0.15 0.15];
        clear current_axes current_limits
        title(['stereo errors of RGB markers in camera frame ' meta.date meta.run ', elevation = ' num2str(block_elevation{i})]) 
    subplot(3,1,2)
        hold on
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,2), 'r', 'displayname', 'red y diff'); catch; end
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,2), 'b', 'displayname', 'blue y diff'); catch; end
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,2), 'g', 'displayname', 'green y diff'); catch; end
        hold off
        grid on
        ylabel('x [m]')
        legend('toggle')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0 10 20 30 40 50 60];
        current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
        current_axes.YTick = [-0.5:0.05:0.5];
        current_axes.YLim = [-0.15 0.15];
        clear current_axes current_limits
    subplot(3,1,3)
        hold on
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,3), 'r', 'displayname', 'red z diff'); catch; end
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,3), 'b', 'displayname', 'blue z diff'); catch; end
            try plot(data.ugvStereo.time, angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,3), 'g', 'displayname', 'green z diff'); catch; end
        hold off
        grid on
        ylabel('x [m]')
        legend('toggle')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0 10 20 30 40 50 60];
        current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
        current_axes.YTick = [-0.5:0.05:0.5];
        current_axes.YLim = [-0.15 0.15];
        clear current_axes current_limits
end
%% figure(600+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
for i = 1:size(block_elevation,1)
    figure(600+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    XLim = [1 5];
    YLim = [-0.45 0.45];
    subplot(3,1,1)
        hold on
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,1), 'r.', 'displayname', 'red x diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,1), 'b.', 'displayname', 'blue x diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,1), 'g.', 'displayname', 'green x diff'); catch; end
        hold off
        grid on
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0:0.5:5];
        current_axes.XLim = XLim;
%         current_axes.YTick = [-0.5:0.05:0.5];
        current_axes.YLim = YLim;
        clear current_axes current_limits
        ylabel('difference [m]')
        legend('toggle')
        title(['stereo errors of RGB markers in camera frame ' meta.date meta.run ', elevation = ' num2str(block_elevation{i})]) 
    subplot(3,1,2)
        hold on
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,2), 'r.', 'displayname', 'red y diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,2), 'b.', 'displayname', 'blue y diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,2), 'g.', 'displayname', 'green y diff'); catch; end
        hold off
        grid on
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0:0.5:5];
        current_axes.XLim = XLim;
%         current_axes.YTick = [-0.5:0.05:0.5];
        current_axes.YLim = YLim;
        clear current_axes current_limits
        ylabel('difference [m]')
        legend('toggle')
    subplot(3,1,3)
        hold on
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,3), 'r.', 'displayname', 'red z diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,3), 'b.', 'displayname', 'blue z diff'); catch; end
            try plot(data.matlabStereo.Red.P_cam(:,3), angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,3), 'g.', 'displayname', 'green z diff'); catch; end
        hold off
        grid on
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0:0.5:5];
        current_axes.XLim = XLim;
%         current_axes.YTick = [-0.5:0.05:0.5];
        current_axes.YLim = YLim;
        clear current_axes current_limits XLim
        ylabel('difference [m]')
        xlabel('z [m]')
        legend('toggle')

end

for i = 1:size(block_elevation,1)
    angleblocks.rms_mean(i,1)= angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.rms_mean;
    angleblocks.rms_mean(i,2)= angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.rms_mean;
    angleblocks.rms_mean(i,3)= angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.rms_mean;
    angleblocks.rms_std(i,1)= angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.rms_std;
    angleblocks.rms_std(i,2)= angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.rms_std;
    angleblocks.rms_std(i,3)= angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.rms_std;
end
%% figure(700); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(700); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,2,1)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Red.left.xy(:,2), 'ro', 'displayname', 'stereo y'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.red_target.left.xy(:,2), 'rx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.red_target.splines.left.xy(:,2), 'r.', 'displayname', 'vicon y'); catch; end
        hold off
        grid on
        ylabel('y pixel')
    subplot(3,2,2)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Red.right.xy(:,2), 'ro', 'displayname', 'stereo y'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.red_target.right.xy(:,2), 'rx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.red_target.splines.right.xy(:,2), 'r.', 'displayname', 'vicon y'); catch; end
        hold off
        grid on
        
    subplot(3,2,3)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'stereo y'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.blue_target.left.xy(:,2), 'bx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.blue_target.splines.left.xy(:,2), 'b.', 'displayname', 'vicon y'); catch; end
        hold off
        grid on
        ylabel('y pixel')
    subplot(3,2,4)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'stereo y'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.blue_target.right.xy(:,2), 'bx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.blue_target.splines.right.xy(:,2), 'b.', 'displayname', 'vicon y'); catch; end
        hold off
        grid on
    
    subplot(3,2,5)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Green.left.xy(:,2), 'go', 'displayname', 'stereo y'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.green_target.left.xy(:,2), 'gx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.green_target.splines.left.xy(:,2), 'g.', 'displayname', 'vicon y'); catch; end
        hold off
        grid on
        xlabel('left')
        ylabel('y pixel')
    subplot(3,2,6)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Green.right.xy(:,2), 'go', 'displayname', 'stereo y'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.green_target.right.xy(:,2), 'gx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.green_target.splines.right.xy(:,2), 'g.', 'displayname', 'vicon y'); catch; end
        hold off
        grid on
        xlabel('right')
%% figure(701); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(701); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,2,1)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Red.left.xy(:,1), 'ro', 'displayname', 'matlabStereo x'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.red_target.left.xy(:,1), 'rx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.red_target.splines.left.xy(:,1), 'r.', 'displayname', 'vicon x'); catch; end
        hold off
        grid on
        ylabel('x pixel')
    subplot(3,2,2)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Red.right.xy(:,1), 'ro', 'displayname', 'matlabStereo x'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.red_target.right.xy(:,1), 'rx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.red_target.splines.right.xy(:,1), 'r.', 'displayname', 'vicon x'); catch; end
        hold off
        grid on
        
    subplot(3,2,3)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Blue.left.xy(:,1), 'bo', 'displayname', 'matlabStereo x'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.blue_target.left.xy(:,1), 'bx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.blue_target.splines.left.xy(:,1), 'b.', 'displayname', 'vicon x'); catch; end
        hold off
        grid on
        ylabel('x pixel')
    subplot(3,2,4)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Blue.right.xy(:,1), 'bo', 'displayname', 'matlabStereo x'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.blue_target.right.xy(:,1), 'bx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.blue_target.splines.right.xy(:,1), 'b.', 'displayname', 'vicon x'); catch; end
        hold off
        grid on
    
    subplot(3,2,5)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Green.left.xy(:,1), 'go', 'displayname', 'matlabStereo x'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.green_target.left.xy(:,1), 'gx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.green_target.splines.left.xy(:,1), 'g.', 'displayname', 'vicon x'); catch; end
        hold off
        grid on
        ylabel('x pixel')
        xlabel('left')
    subplot(3,2,6)
        hold on
            try plot(data.matlabStereo.time, data.matlabStereo.Green.right.xy(:,1), 'go', 'displayname', 'matlabStereo x'); catch; end
            try plot(data.matlabStereo.time, data.matlabStereo.green_target.right.xy(:,1), 'gx', 'displayname', 'vicon using modified ugv2cam x'); catch; end
            try plot(data.matlabStereo.time, data.vicon.green_target.splines.right.xy(:,1), 'g.', 'displayname', 'vicon x'); catch; end
        hold off
        grid on
        xlabel('right')    
    
    
end