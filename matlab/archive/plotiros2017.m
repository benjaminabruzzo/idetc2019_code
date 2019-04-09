%% Turn plotting back on
set(0, 'DefaultFigureVisible', 'on');
figHandles = findall(0, 'Type', 'figure');
set(figHandles(:), 'visible', 'on')

try april = data.april;catch;end
try uavCon = data.uavCon;catch;end
try ugvStereo = data.ugvStereo;catch;end
try experiment= data.experiment;catch;end
try uavRecorder = data.uavRecorder;catch;end
try ugvRecorder = data.ugvRecorder;catch;end
try timers = data.timers;catch;end
try vicon = data.vicon;catch;end


close all
tic
disp('Plotting...')
%% figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
if strcmp([meta.date meta.run],'20170112/022') %meta.date = '20170112/'; meta.run = '022';
    figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title(['uav yaw and command ' [meta.date meta.run]])
        hold on
            try plot(vicon.uav.time, vicon.uav.yaw.global.degrees,'k-', 'LineWidth',5, 'displayname', 'uav yaw vicon'); catch; end        
            try plot(uavRecorder.ckf.time, uavRecorder.ckf.Yaw, 'ro', ...
                    'MarkerFaceColor', 'r', 'MarkerSize', 9, 'displayname', 'uav yaw ckf est'); catch; end
            try plot(uavRecorder.est.time, uavRecorder.est.Yaw, 'bo', ...
                    'MarkerFaceColor', 'b', 'MarkerSize', 3, 'displayname', 'uav yaw ref trajectory'); catch; end
    %         try plot(uavCon.time, uavCon.Desired.Yaw, 'r.', 'displayname', 'global desired yaw'); catch; end
    %         try plot(ugvRecorder.stereo.time, ugvRecorder.stereo.yaw_ugv, 'go', 'displayname', 'uav stereo yaw global'); catch; end
        hold off
        ylabel('degrees')
        xlabel('time, [s]')
        grid on
        legend('toggle')
        axis([0 10 -150 150])
        if 0
            %%
            current_fig = gcf;
            
            meta.filename = ['figure_' num2str(current_fig.Number)];
            meta.saveplotroot = '/media/benjamin/devsdb/hast/tex/iros2017/';

            saveas(gcf, [meta.saveplotroot '/figs/' meta.filename '.png']); 
            saveas(gcf, [meta.saveplotroot '/figs/' meta.filename '.fig']); 
            print('-depsc', [meta.saveplotroot '/eps/' meta.filename '.eps']); 
            clear current_fig;
        end    
    
end
%% figure(12); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(12); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(4,1,1)
        hold on
            try plot(experiment.uav.FlyTime, experiment.uav.Waypoint(:,1), 'rd', 'displayname', 'UAV x goal'); catch; end
            try plot(vicon.uav.time, vicon.uav.P.global(:,1),'k.', 'displayname', 'UAV x actual'); catch; end
            try plot(uavRecorder.est.time, uavRecorder.est.Position_gl(:,1), 'b.', 'displayname', 'UAV x estimated'); catch; end
            
        hold off
        ylabel('x [m]')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 uavRecorder.est.time(end)];
        current_axes.YTick = [0:2:6];
        current_axes.YLim = [0 6];
        clear current_axes current_limits

    subplot(4,1,2)
        hold on
            try plot(experiment.uav.FlyTime, experiment.uav.Waypoint(:,2), 'rd', 'displayname', 'UAV y goal'); catch; end
            try plot(vicon.uav.time, vicon.uav.P.global(:,2),'k.', 'displayname', 'UAV y actual'); catch; end
            try plot(uavRecorder.est.time, uavRecorder.est.Position_gl(:,2), 'b.', 'displayname', 'UAV y estimated'); catch; end
            
        hold off
        ylabel('y [m]')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 uavRecorder.est.time(end)];
        current_axes.YTick = [-1 0 1];
        current_axes.YLim = [-1 1];
        clear current_axes current_limits

    subplot(4,1,3)
        hold on
            try plot(experiment.uav.FlyTime, experiment.uav.DesiredState_gl(:,3), 'rd', 'displayname', 'UAV yaw goal'); catch; end
%             try plot(experiment.uav.FlyTime, experiment.uav.Waypoint(:,3), 'rd', 'displayname', 'UAV z goal'); catch; end
            try plot(vicon.uav.time, vicon.uav.P.ugv(:,3),'k.', 'displayname', 'UAV z actual'); catch; end
            try plot(uavRecorder.est.time, uavRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'UAV z estimated'); catch; end
        hold off
        ylabel('z [m]')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 uavRecorder.est.time(end)];
        current_axes.YTick = [0:2];
        current_axes.YLim = [0 2];
        clear current_axes current_limits

    subplot(4,1,4)
        hold on
            try plot(experiment.uav.FlyTime, experiment.uav.Waypoint(:,4), 'rd', 'displayname', 'UAV yaw goal'); catch; end
            try plot(vicon.uav.time, vicon.uav.yaw.global.degrees,'k.', 'displayname', 'UAV yaw actual'); catch; end
            try plot(uavRecorder.est.time, uavRecorder.est.Yaw, 'b.', 'displayname', 'UAV yaw estimated'); catch; end
        hold off
        ylabel('heading [deg]')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 uavRecorder.est.time(end)];
        current_axes.YTick = [-90:90:90];
        current_axes.YLim = [-90 90];
        clear current_axes current_limits
        xlabel('time [s]')

        
        if 0
            %%
            current_fig = gcf;
            
            meta.filename = ['figure_' num2str(current_fig.Number)];
            meta.saveplotroot = '/media/benjamin/devsdb/hast/tex/iros2017/';

            saveas(gcf, [meta.saveplotroot '/figs/' meta.filename '.png']); 
            saveas(gcf, [meta.saveplotroot '/figs/' meta.filename '.fig']); 
            print('-depsc', [meta.saveplotroot '/eps/' meta.filename '.eps']); 
            clear current_fig;
        end    
%% figure(13); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(13); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('estimated errors')
    subplot(2,1,1)
        hold on
            try plot(uavRecorder.est.time, vicon.uav.splines.P.global.x_diff, 'displayname', 'uav x error'); catch; end
            try plot(uavRecorder.est.time, vicon.uav.splines.P.global.y_diff, 'displayname', 'uav y error'); catch; end
            try plot(uavRecorder.est.time, vicon.uav.splines.P.global.z_diff, 'displayname', 'uav z error'); catch; end
        hold off
        grid on
        legend('toggle')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 uavRecorder.est.time(end)];
        current_axes.YTick = [-0.5:0.25:0.5];
        current_axes.YLim = [-0.5 0.5];
        clear current_axes current_limits
        ylabel('error [m]')

    subplot(2,1,2)
        hold on
            try plot(uavRecorder.est.time, vicon.est_yaw.spline_error, 'displayname', 'uav yaw error'); catch; end
        hold off
        grid on
        legend('toggle')
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 uavRecorder.est.time(end)];
        current_axes.YTick = [-10:2.5:10];
        current_axes.YLim = [-10 10];
        clear current_axes current_limits
        ylabel('error [deg]')
        xlabel('time [s]')
        
        if 0
            %%
            current_fig = gcf;
            
            meta.filename = ['figure_' num2str(current_fig.Number)];
            meta.saveplotroot = '/media/benjamin/devsdb/hast/tex/iros2017/';

            saveas(gcf, [meta.saveplotroot '/figs/' meta.filename '.png']); 
            saveas(gcf, [meta.saveplotroot '/figs/' meta.filename '.fig']); 
            print('-depsc', [meta.saveplotroot '/eps/' meta.filename '.eps']); 
            clear current_fig;
        end    
%% figure(14); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(14); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% [ugvMatlab] = yaw_cov_from_ugvStereo(ugvStereo);
    title(['uav yaw global frame ' [meta.date meta.run]])    
    ylabel('yaw-degrees')
    hold on
    
        try plot(ones(size(ugvStereo.uav.UAV_yaw_cov))*mean(vicon.uav.P.cam(:,3)), vicon.diff_yaw.spline_error - mean(vicon.diff_yaw.spline_error), ...
                'bo', 'displayname', 'uav yaw stereo error' ); catch; end
        try plot(ones(size(ugvStereo.uav.UAV_yaw_cov))*mean(vicon.uav.P.cam(:,3))+0.025, 180*sqrt(ugvMatlab.yaw_cov)/pi,...
                'rx', 'displayname', 'sqrt of propagated pixel covariances, experimental' ); catch; end

    hold off
    grid on
    legend('toggle')
    axis([0 5 -135 135])
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(15); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(15); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav yaw global frame ' [meta.date meta.run]])    
    ylabel('degrees')
    hold on
        try plot(vicon.uav.time, vicon.uav.yaw.global.degrees, 'kx', 'displayname', 'uav vicon global'); catch; end
        try plot(uavRecorder.est.time, uavRecorder.est.Yaw, 's', 'displayname', 'uav est yaw'); catch ;end
        try plot(uavRecorder.navdata.time, uavRecorder.navdata.CompassYaw, 'gd', 'displayname', 'navdata-compass-yaw' ); catch; end
        try plot(uavRecorder.est.time, vicon.est_yaw.spline_error, '.', 'displayname', 'uav yaw est error' ); catch; end

    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav yaw global frame ' [meta.date meta.run]])    
    ylabel('degrees')
    hold on
        try plot(vicon.uav.time, vicon.uav.yaw.global.degrees, 'kx', 'displayname', 'uav vicon global'); catch; end
        try plot(uavRecorder.est.time, uavRecorder.est.Yaw, 's', 'displayname', 'uav est yaw'); catch ;end
        try plot(uavRecorder.est.time, vicon.est_yaw.spline_error, '.', 'displayname', 'uav yaw est error' ); catch; end
        try plot(ugvStereo.time,180*sqrt(ugvStereo.uav.UAV_yaw_var)/pi, 'x', 'displayname', 'sqrt of propagated uav var'); catch; end

    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav yaw global frame ' [meta.date meta.run]])    
    ylabel('degrees')
    xlabel('range along optical axis, [m]')
    hold on
        try h1 = plot(mean(vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(vicon.uav.splines.P.cam.z_uavstereotime)), ...
                vicon.diff_yaw.spline_error, 'bo', 'displayname', 'stereo yaw error'); catch; end
        try h2 = plot(0.025+mean(vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(vicon.uav.splines.P.cam.z_uavstereotime)), ...
                180*sqrt(ugvStereo.uav.UAV_yaw_var)/pi, 'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
        try h2 = plot(0.025+mean(vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(vicon.uav.splines.P.cam.z_uavstereotime)), ...
                180*sqrt(ugvStereo.uav.UAV_yaw_cov)/pi, 'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
        try h3 = plot(0.025+mean(vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(vicon.uav.splines.P.cam.z_uavstereotime)), ...
                -180*sqrt(ugvStereo.uav.UAV_yaw_var)/pi, 'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
        try h3 = plot(0.025+mean(vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(vicon.uav.splines.P.cam.z_uavstereotime)), ...
                -180*sqrt(ugvStereo.uav.UAV_yaw_cov)/pi, 'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
    hold off
    grid on
    axis([0 5 -135 135])
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'northwest')
        clear h1 h2 h3
    catch
    end
        
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end    
%% figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
disp('uav command gains')
figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
    title('uav command gains x global')
    hold on
        try plot(uavCon.time, uavCon.Current.Position_g(:,1), '.', 'displayname', 'uav current p global'); catch; end
        try plot(uavCon.time, uavCon.Desired.Position_g(:,1), '.', 'displayname', 'uav desired p global'); catch; end
        try plot(uavCon.time, uavCon.cmd.linear_g(:,1), '.', 'displayname', 'uav cmd global'); catch; end
    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav command gains x local')
    hold on
        try plot(uavCon.time, uavCon.Current.Position_dr(:,1), '.', 'displayname', 'uav current p global'); catch; end
        try plot(uavCon.time, uavCon.Desired.Position_dr(:,1), '.', 'displayname', 'uav desired p global'); catch; end
        try plot(uavCon.time, uavCon.cmd.linear_dr(:,1), '.', 'displayname', 'uav cmd global'); catch; end
    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(29); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
disp('uav ckf debug plots')
figure(29); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uavRecorder.ckf.zk x y z')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.zk(:,1), 'displayname', 'zk x'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.zk(:,2), 'displayname', 'zk y'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.zk(:,3), 'displayname', 'zk z'); catch; end
    hold off
    legend('toggle')
    grid on
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(30); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(30); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uavRecorder.ckf.zk yaw yawbias')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.zk(:,4), 'displayname', 'zk yaw'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.zk(:,5), 'displayname', 'zk yaw bias'); catch; end
    hold off
    legend('toggle')
    grid on
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(31); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(31); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Yaw Bias')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.YawBias, 'displayname', 'EstYawBias'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.PosteriorEst(:,5), 'displayname', 'PosteriorEst5'); catch; end
    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(32); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(32); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav velocities')
    hold on
        try plot(uavRecorder.navdata.time, 0.001*uavRecorder.navdata.V); catch; end
    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(33); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(33); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    try 
        legs = [sqrt(sum((ugvStereo.Red.P_cam - ugvStereo.Green.P_cam)'.^2)') ...
        sqrt(sum((ugvStereo.Green.P_cam - ugvStereo.Blue.P_cam)'.^2)') ...
        sqrt(sum((ugvStereo.Blue.P_cam - ugvStereo.Red.P_cam)'.^2)')];
    catch
    end

%     title('ugv obs and est of uav in global')
    yyaxis left
        try plot(ugvStereo.time, vicon.diff_yaw.spline_error, 'k', 'displayname', 'vicon/stereo yaw difference'); catch; end
    yyaxis right
    hold on
        try plot(ugvStereo.time, legs(:,1), 'displayname', 'RG leg'); catch; end
        try plot(ugvStereo.time, legs(:,2), 'displayname', 'GB leg'); catch; end
        try plot(ugvStereo.time, legs(:,3), 'displayname', 'BR leg'); catch; end
    hold off
        
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    try
        X = [ugvRecorder.ckf.time(1) * ones(2,1), ugvRecorder.ckf.time(end) * ones(2,1)];
        Y = [current_axes(3) * ones(1,2); current_axes(4) * ones(1,2)]; 
        line(X,Y)
    catch
        warning('ugvRecorder.ckf.time missing..?')
    end
    clear current_axes X Y
        
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(34); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(34); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Rk_re(:,1), 'o-', 'displayname', 'Rk (1,1)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Qdk_re(:,1), 's-', 'displayname', 'Qdk(1,1)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.PosteriorCov_re(:,1), 'd-.', 'displayname', 'PosteriorCov(1,1)'); catch; end
        try plot(uavRecorder.navdata.time, uavRecorder.navdata.ckf.Qdk(:,1), 's-', 'displayname', 'Qdk_i(1,1)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([uavRecorder.ckf.time(1)-3 uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(35); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(35); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Rk_re(:,6), 'o-', 'displayname', 'Rk (2,2)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Qdk_re(:,7), 's-', 'displayname', 'Qdk(2,2)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.PosteriorCov_re(:,7), 'd-.', 'displayname', 'PosteriorCov(2,2)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([uavRecorder.ckf.time(1)-3 uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(36); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(36); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Rk_re(:,11), 'o-', 'displayname', 'Rk (3,3)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Qdk_re(:,13), 's-', 'displayname', 'Qdk(3,3)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.PosteriorCov_re(:,13), 'd-.', 'displayname', 'PosteriorCov(3,3)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([uavRecorder.ckf.time(1)-3 uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(37); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(37); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Rk_re(:,16), 'o-', 'displayname', 'Rk (4,4)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Qdk_re(:,19), 's-', 'displayname', 'Qdk(4,4)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.PosteriorCov_re(:,19), 'd-.', 'displayname', 'PosteriorCov(4,4)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.PosteriorCov_re(:,25), 'x-.', 'displayname', 'PosteriorCov(5,5)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([uavRecorder.ckf.time(1)-3 uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(38); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(38); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.Qdk_re(:,25), 's-', 'displayname', 'Qdk(4,4)'); catch; end
        try plot(uavRecorder.ckf.time, uavRecorder.ckf.PosteriorCov_re(:,25), 'd-.', 'displayname', 'PosteriorCov(5,5)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([uavRecorder.ckf.time(1)-3 uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
disp('ugv odometry figures')
figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['Estimated UGV Position XY' [meta.date meta.run]])
    hold on
    try plot(vicon.ugvk.P.global(:,1), vicon.ugvk.P.global(:,2), '.', 'displayname', 'ugv vicon global y') ; catch; end
    try plot(uavRecorder.wheel.P_odom(:,1), ugvRecorder.wheel.P_odom(:,2), 'r','LineWidth',4, 'displayname', 'ugv wheel odom y'); catch; end
%     try plot(stateRecorder.PosLinOdom_g(:,1) - stateRecorder.PosLinOdom_g(1,1), stateRecorder.PosLinOdom_g(:,2) - stateRecorder.PosLinOdom_g(1,2), 'ko-', 'displayname', 'Stereopsis'); catch; end
    try plot(ugvRecorder.ckf.Position_gl(:,1), ugvRecorder.ckf.Position_gl(:,2),'o', 'displayname', 'ugv ckf global y'); catch; end
    try plot(ugvRecorder.est.Position_gl(:,1), ugvRecorder.est.Position_gl(:,2),'s', 'displayname', 'ugv est global y'); catch; end

        
    hold off 
    grid on
    xlabel('x position (m)'); 
    ylabel('y position (m)'); 
    legend('toggle')
    legend('location', 'Southwest')
%     current_axes = axis;
%     try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%     clear current_axes
% 	axis([-3 .5 -1 1])
    axis([-0.5 1.5 -1 1])
%     axis([-0.25 1 -0.1 0.1])
    
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        saveas(gcf, [meta.figpath num2str(meta.run) '_UGV-posittion-est.png']); 
        clear current_fig;
    end
%% figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    title(['Estimated Position (x vs time)' [meta.date meta.run]])
        try plot(vicon.ugvk.time, vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'Global Vicon Position'); catch; end
        try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,1), '*-', 'displayname', 'Global Wheel Odometry'); catch; end    
        try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,1), 's', 'displayname', 'ugv x est'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Position_gl(:,1), 'x', 'displayname', 'ugv x ckf'); catch; end
    hold off 
    grid on
    xlabel('time (sec)'); 
    ylabel('x position (m)'); 
    legend('toggle')
    legend('location', 'northwest')
    current_axes = axis;
%     try axis([9 13 current_axes(3) current_axes(4)]); catch; end
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        saveas(gcf, [meta.figpath num2str(meta.run) '_UGV-x-vs-time.png']); 
        clear current_fig;
    end
%% figure(52); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(52); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    title(['Estimated Position (y vs time)' [meta.date meta.run]])
        try plot(vicon.ugvk.time, vicon.ugvk.P.global(:,2), 'd-', 'displayname', 'Global Vicon Position y'); catch; end
        try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,2), '*-', 'displayname', 'Global Wheel Odometry y'); catch; end    
        try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,2), 's', 'displayname', 'ugv y est'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Position_gl(:,2), 'x', 'displayname', 'ugv y ckf'); catch; end
        
    hold off 
    grid on
    xlabel('time (sec)'); 
    ylabel('y position (m)'); 
    legend('toggle')
    legend('location', 'northwest')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        saveas(gcf, [meta.figpath num2str(meta.run) '_UGV-y-vs-time.png']); 
        clear current_fig;
    end
%% figure(53); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(53); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    title(['Estimated Position (z vs time)' [meta.date meta.run]])
        try plot(vicon.ugvk.time, vicon.ugvk.P.global(:,3), 'd-', 'displayname', 'Global Vicon Position z'); catch; end
        try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,3), '*-', 'displayname', 'Global Wheel Odometry z'); catch; end    
        try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,3), 's', 'displayname', 'ugv z est'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Position_gl(:,3), 'x', 'displayname', 'ugv z ckf'); catch; end
        
    hold off 
    grid on
    xlabel('time (sec)'); 
    ylabel('z position (m)'); 
    legend('toggle')
    legend('location', 'northwest')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        saveas(gcf, [meta.figpath num2str(meta.run) '_UGV-z-vs-time.png']); 
        clear current_fig;
    end
%% figure(54); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(54); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    title(['Estimated Position (yaw vs time)' [meta.date meta.run]])
        try plot(vicon.ugvk.time, vicon.ugvk.yaw.global*180/pi, 'd-', 'displayname', 'Global Vicon Yaw'); catch; end
        try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.yaw*180/pi, '*-', 'displayname', 'ugv wheel odometry yaw'); catch; end
        try plot(ugvRecorder.est.time, ugvRecorder.est.Yaw, 's', 'displayname', 'ugv yaw est'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Yaw, 'x', 'displayname', 'ugv yaw ckf'); catch; end
    hold off 
    grid on
    legend('toggle')
    legend('location', 'northwest')
    xlabel('time (sec)'); 
    ylabel('yaw position (deg)'); 
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    
%     meta.saveplots = true; 
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        saveas(gcf, [meta.figpath num2str(meta.run) '_UGV-yaw-vs-time.png']); 
        print('-depsc', [meta.figpath '_UGV-yaw-vs-time.eps']); 
        clear current_fig;
    end
%     meta.saveplots = false; 
%% figure(55); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(55); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv Mech and Aiding, x')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Mech(:,1), 's-', 'displayname', 'ugv mech x'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Aiding(:,1), 'o-', 'displayname', 'ugv aid x'); catch; end
        try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,1), 'd-', 'displayname', 'ugv est x'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorEst(:,1), 's-', 'displayname', 'ugv PosteriorEst x'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv Mech and Aiding, y')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Mech(:,2), 's-', 'displayname', 'ugv mech y'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Aiding(:,2), 'o-', 'displayname', 'ugv aid y'); catch; end
        try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,2), 'd-', 'displayname', 'ugv est y'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorEst(:,2), 's-', 'displayname', 'ugv PosteriorEst y'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(57); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(57); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv Mech and Aiding, z')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Mech(:,3), 's-', 'displayname', 'ugv mech z'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Aiding(:,3), 'o-', 'displayname', 'ugv aid z'); catch; end
        try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,3), 'd-', 'displayname', 'ugv est z'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorEst(:,3), 's-', 'displayname', 'ugv PosteriorEst z'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(58); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(58); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv Mech and Aiding, yaw')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Mech(:,4), 's-', 'displayname', 'ugv mech yaw'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Aiding(:,4), 'o-', 'displayname', 'ugv aid yaw'); catch; end
        try plot(ugvRecorder.est.time, ugvRecorder.est.Yaw, 'd-', 'displayname', 'ugv est yaw'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorEst(:,4), 'h-', 'displayname', 'ugv PosteriorEst yaw'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.zk(:,4), 'x-', 'displayname', 'ugv zk yaw'); catch; end
        
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(59); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(59); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv Mech and Aiding, yaw')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Mech(:,4), 's-', 'displayname', 'ugv mech yaw'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Aiding(:,4), 'x-', 'displayname', 'ugv aid yaw'); catch; end

        try plot(ugvRecorder.est.time, ugvRecorder.est.Yaw, 'd-', 'displayname', 'ugv est yaw'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorEst(:,4), 's-', 'displayname', 'ugv PosteriorEst yaw'); catch; end
        
        try plot(ugvRecorder.stereo.time, ugvRecorder.stereo.yaw_ugv-mean(ugvRecorder.stereo.yaw_ugv), 'o', 'displayname', 'stereo-yaw-ugv'); catch; end
        try plot(uavRecorder.est.time, uavRecorder.est.Yaw-mean(ugvRecorder.stereo.yaw_ugv), 's', 'displayname', 'uav est yaw'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
%     try axis([ugvRecorder.ckf.time(1)-1.5 ugvRecorder.ckf.time(end)+1.5 -3 4]); catch; end
%     try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(60); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(60); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('est yaw of auv')
    hold on
        try plot(ugvRecorder.stereo.time, ugvRecorder.stereo.yaw_ugv, 'o', 'displayname', 'stereo yaw ugv2uav'); catch; end
        try plot(vicon.uav.time, vicon.uav.yaw.global.degrees, 'kx', 'displayname', 'uav vicon global'); catch; end
        try plot(uavRecorder.est.time, uavRecorder.est.Yaw, 's', 'displayname', 'uav est yaw global'); catch ;end

    hold off
    grid on
    legend('toggle')    
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(61); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(61); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv ckf qw and rk')
    hold on
    yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Rk_re(:,1), 'o-', 'displayname', 'Rk (1,1)'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorCov_re(:,1), 'x-.', 'displayname', 'PosteriorCov(1,1)'); catch; end
    yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Qdk_re(:,1), 's-', 'displayname', 'Qdk(1,1)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(62); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(62); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv ckf qw and rk')
    hold on
    yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Rk_re(:,6), 'o-', 'displayname', 'Rk (2,2)'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorCov_re(:,6), 'x-.', 'displayname', 'PosteriorCov(2,2)'); catch; end
    yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Qdk_re(:,6), 's-', 'displayname', 'Qdk(2,2)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(63); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(63); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv ckf qw and rk')
    hold on
    yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Rk_re(:,11), 'o-', 'displayname', 'Rk (3,3)'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorCov_re(:,11), 'x-.', 'displayname', 'PosteriorCov(3,3)'); catch; end
    yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Qdk_re(:,11), 's-', 'displayname', 'Qdk(3,3)'); catch; end   
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(64); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(64); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv ckf qw and rk')
    hold on
    yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Rk_re(:,16), 'o-', 'displayname', 'Rk (4,4)'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.PosteriorCov_re(:,16), 'x-.', 'displayname', 'PosteriorCov(4,4)'); catch; end
    yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Qdk_re(:,16), 's-', 'displayname', 'Qdk(4,4)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv obs and est of uav in global')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.obs_uav_in_global(:,1), 's-', 'displayname', 'ugv obs of uav x in global'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.p_est_uav_in_global(:,1), 'o-', 'displayname', 'ugv est of uav x in global'); catch; end
        try plot(uavRecorder.est.time, uavRecorder.est.Position_gl(:,1), 'b.', 'displayname', 'uav ckf est x'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(67); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(67); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv obs and est of uav in global')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.obs_uav_in_global(:,2), 's-', 'displayname', 'ugv obs of uav y in global'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.p_est_uav_in_global(:,2), 'o-', 'displayname', 'ugv est of uav y in global'); catch; end
        try plot(uavRecorder.est.time, uavRecorder.est.Position_gl(:,2), 'b.', 'displayname', 'uav est y global'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(68); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(68); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('ugv obs and est of uav in global')
    hold on
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.obs_uav_in_global(:,3), 's-', 'displayname', 'ugv obs of uav z in global'); catch; end
        try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.p_est_uav_in_global(:,3), 'o-', 'displayname', 'ugv est of uav z in global'); catch; end
        try plot(uavRecorder.est.time, uavRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'uav est z global'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(70); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(70); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(4,1,1)
        hold on
            try plot(vicon.ugvk.time, vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'Global Vicon Position'); catch; end
            try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,1), '*-', 'displayname', 'Global Wheel Odometry'); catch; end    
            try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,1), 's', 'displayname', 'ugv x est'); catch; end
            try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Position_gl(:,1), 'x', 'displayname', 'ugv x ckf'); catch; end
        hold off
        ylabel('x [m]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(ugvRecorder.ckf.time(1)-1):floor(ugvRecorder.ckf.time(end)+1);
        current_axes.XLim = [ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1];
        current_axes.YTick = [-.25 0 .25 .5 .75];
        current_axes.YLim = [-.25 .25];
        clear current_axes current_limits
        catch; end

    subplot(4,1,2)
        hold on
            try plot(vicon.ugvk.time, vicon.ugvk.P.global(:,2), 'd-', 'displayname', 'Global Vicon Position y'); catch; end
            try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,2), '*-', 'displayname', 'Global Wheel Odometry y'); catch; end    
            try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,2), 's', 'displayname', 'ugv y est'); catch; end
            try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Position_gl(:,2), 'x', 'displayname', 'ugv y ckf'); catch; end
        hold off
        ylabel('y [m]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(ugvRecorder.ckf.time(1)-1):floor(ugvRecorder.ckf.time(end)+1);
        current_axes.XLim = [ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1];
        current_axes.YTick = [-.25 0 .25];
        current_axes.YLim = [-.25 .25];
        clear current_axes current_limits
        catch; end

    subplot(4,1,3)
        hold on
            try plot(vicon.ugvk.time, vicon.ugvk.P.global(:,3), 'd-', 'displayname', 'Global Vicon Position z'); catch; end
            try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,3), '*-', 'displayname', 'Global Wheel Odometry z'); catch; end    
            try plot(ugvRecorder.est.time, ugvRecorder.est.Position_gl(:,3), 's', 'displayname', 'ugv z est'); catch; end
            try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Position_gl(:,3), 'x', 'displayname', 'ugv z ckf'); catch; end
        hold off
        ylabel('z [m]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(ugvRecorder.ckf.time(1)-1):floor(ugvRecorder.ckf.time(end)+1);
        current_axes.XLim = [ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1];
        current_axes.YTick = [-.25 0 .25];
        current_axes.YLim = [-.25 .25];
        clear current_axes current_limits
        catch; end

    subplot(4,1,4)
        hold on
            try plot(vicon.ugvk.time, vicon.ugvk.yaw.global*180/pi, 'd-', 'displayname', 'Global Vicon Yaw'); catch; end
            try plot(ugvRecorder.wheel.time, ugvRecorder.wheel.yaw*180/pi, '*-', 'displayname', 'ugv wheel odometry yaw'); catch; end
            try plot(ugvRecorder.est.time, ugvRecorder.est.Yaw, 's', 'displayname', 'ugv yaw est'); catch; end
            try plot(ugvRecorder.ckf.time, ugvRecorder.ckf.Yaw, 'x', 'displayname', 'ugv yaw ckf'); catch; end
        hold off
        ylabel('heading [deg]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(ugvRecorder.ckf.time(1)-1):floor(ugvRecorder.ckf.time(end)+1);
        current_axes.XLim = [ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1];
        current_axes.YTick = -2:4:16;
        current_axes.YLim = [-2 16];
        clear current_axes current_limits
        catch; end
        xlabel('time [s]')

        
% meta.saveplots = true;
if meta.saveplots
    current_fig = gcf;
    saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']);  
    print('-depsc', [meta.figpath 'figure(' num2str(current_fig.Number) ').eps']); 
    clear current_fig;
end
% meta.saveplots = false;
%% figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
disp('RGB pixels and marker locations')
figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,2,1)
    hold on
    try  plot(ugvStereo.Red.left.xy(:,1), ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.left.xy(:,1), ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.left.xy(:,1), ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on

    subplot(2,2,2)
    hold on
    try  plot(ugvStereo.Red.right.xy(:,1), ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.right.xy(:,1), ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.right.xy(:,1), ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    subplot(2,2,3)
    hold on
    try  plot(ugvStereo.Red.left.xy(:,1), ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.left.xy(:,1), ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.left.xy(:,1), ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    axis([-320 320 -240 240])
    subplot(2,2,4)
    hold on
    try  plot(ugvStereo.Red.right.xy(:,1), ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.right.xy(:,1), ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.right.xy(:,1), ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    axis([-320 320 -240 240])
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(101); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(101); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,2,1)
    hold on
    try  plot(ugvStereo.Red.left.xy(:,1), ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.left.xy(:,1), ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.left.xy(:,1), ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    try addArrow([ugvStereo.Red.LEDs(bp1,1) ugvStereo.Red.LEDs(bp1,2)], [ugvStereo.Red.LEDs(bp2,1) ugvStereo.Red.LEDs(bp2,2)],'LineWidth',3, 'color', 'k', 'HeadWidth', 20); catch; end
    try addArrow([ugvStereo.Red.LEDs(bp1,1) ugvStereo.Red.LEDs(bp1,2)], [ugvStereo.Red.LEDs(bp2,1) ugvStereo.Red.LEDs(bp2,2)],'LineWidth',3, 'color', 'k', 'HeadWidth', 20);catch; end
    try addArrow([ugvStereo.Red.LEDs(bp1,1) ugvStereo.Red.LEDs(bp1,2)], [ugvStereo.Red.LEDs(bp2,1) ugvStereo.Red.LEDs(bp2,2)],'LineWidth',3, 'color', 'k', 'HeadWidth', 20);catch; end
    hold off
    grid on

    subplot(2,2,2)
    hold on
    try  plot(ugvStereo.Red.right.xy(:,1), ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.right.xy(:,1), ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.right.xy(:,1), ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    subplot(2,2,3)
    hold on
    try  plot(ugvStereo.Red.left.xy(:,1), ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.left.xy(:,1), ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.left.xy(:,1), ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    axis([-320 320 -240 240])
    subplot(2,2,4)
    hold on
    try  plot(ugvStereo.Red.right.xy(:,1), ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(ugvStereo.Blue.right.xy(:,1), ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(ugvStereo.Green.right.xy(:,1), ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    axis([-320 320 -240 240])
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['vicon marker positions (x) in ugv frame' [meta.date meta.run]])
    hold on
        try plot(vicon.red_target.time, vicon.red_target.P.ugv(:,1), 'r.', 'displayname', 'red-vicon-x'); catch; end
        try plot(vicon.blue_target.time, vicon.blue_target.P.ugv(:,1), 'b.', 'displayname', 'blue-vicon-x'); catch; end
        try plot(vicon.green_target.time, vicon.green_target.P.ugv(:,1), 'g.', 'displayname', 'green-vicon-x'); catch; end
        try plot(ugvStereo.time, ugvStereo.Red.P_ugv(:,1), 'r+', 'displayname', 'red-stereo-x'); catch; end
        try plot(ugvStereo.time, ugvStereo.Blue.P_ugv(:,1), 'b+', 'displayname', 'blue-stereo-x'); catch; end
        try plot(ugvStereo.time, ugvStereo.Green.P_ugv(:,1), 'g+', 'displayname', 'green-stereo-x'); catch; end
        try plot(ugvStereo.time, offlineStereo.Red.P_ugv(:,1), 'rd', 'displayname', 'red-offline-x'); catch; end
        try plot(ugvStereo.time, offlineStereo.Blue.P_ugv(:,1), 'bd', 'displayname', 'blue-offline-x'); catch; end
        try plot(ugvStereo.time, offlineStereo.Green.P_ugv(:,1), 'gd', 'displayname', 'green-offline-x'); catch; end
    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(107); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(107); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['vicon marker positions (y) in ugv frame' [meta.date meta.run]])
    hold on
        try plot(vicon.red_target.time, vicon.red_target.P.ugv(:,2), 'r.', 'displayname', 'red-vicon-y'); catch; end
        try plot(vicon.blue_target.time, vicon.blue_target.P.ugv(:,2), 'b.', 'displayname', 'red-vicon-y'); catch; end
        try plot(vicon.green_target.time, vicon.green_target.P.ugv(:,2), 'g.', 'displayname', 'red-vicon-y'); catch; end
        try plot(ugvStereo.time, ugvStereo.Red.P_ugv(:,2), 'r+', 'displayname', 'red-stereo-y'); catch; end
        try plot(ugvStereo.time, ugvStereo.Blue.P_ugv(:,2), 'b+', 'displayname', 'blue-stereo-y'); catch; end
        try plot(ugvStereo.time, ugvStereo.Green.P_ugv(:,2), 'g+', 'displayname', 'green-stereo-y'); catch; end
        try plot(ugvStereo.time, offlineStereo.Red.P_ugv(:,2), 'rd', 'displayname', 'red-offline-y'); catch; end
        try plot(ugvStereo.time, offlineStereo.Blue.P_ugv(:,2), 'bd', 'displayname', 'blue-offline-y'); catch; end
        try plot(ugvStereo.time, offlineStereo.Green.P_ugv(:,2), 'gd', 'displayname', 'green-offline-y'); catch; end

    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['vicon marker positions (z) in ugv frame' [meta.date meta.run]])
    hold on
        try plot(vicon.red_target.time, vicon.red_target.P.ugv(:,3), 'ro', 'displayname', 'red-vicon-z'); catch; end
        try plot(vicon.blue_target.time, vicon.blue_target.P.ugv(:,3), 'bo', 'displayname', 'red-vicon-z'); catch; end
        try plot(vicon.green_target.time, vicon.green_target.P.ugv(:,3), 'go', 'displayname', 'red-vicon-z'); catch; end
        try plot(ugvStereo.time, ugvStereo.Red.P_ugv(:,3), 'r+', 'displayname', 'red-stereo-z'); catch; end
        try plot(ugvStereo.time, ugvStereo.Blue.P_ugv(:,3), 'b+', 'displayname', 'blue-stereo-z'); catch; end
        try plot(ugvStereo.time, ugvStereo.Green.P_ugv(:,3), 'g+', 'displayname', 'green-stereo-z'); catch; end
        try plot(ugvStereo.time, offlineStereo.Red.P_ugv(:,3), 'rd', 'displayname', 'red-offline-z'); catch; end
        try plot(ugvStereo.time, offlineStereo.Blue.P_ugv(:,3), 'bd', 'displayname', 'blue-offline-z'); catch; end
        try plot(ugvStereo.time, offlineStereo.Green.P_ugv(:,3), 'gd', 'displayname', 'green-offline-z'); catch; end

    hold off
    grid on
    legend('toggle')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['vicon marker positions (z) in vicon frame' [meta.date meta.run]])
    hold on
        try plot(vicon.red_target.time, vicon.red_target.P.vicon(:,1), '.', 'displayname', 'uav x'); catch; end%UAV location, x
        try plot(vicon.blue_target.time, vicon.blue_target.P.vicon(:,1), '.', 'displayname', 'uav x'); catch; end%UAV location, x
        try plot(vicon.green_target.time, vicon.green_target.P.vicon(:,1), '.', 'displayname', 'uav x'); catch; end%UAV location, x
    hold off
    grid on
    legend('toggle')
%% figure(702); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(702); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,2,1)
        try  plot(ugvStereo.time, ugvStereo.Red.left.xy(:,1), 'ro', 'displayname', 'red-pixels-xl'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,2)
        try  plot(ugvStereo.time, ugvStereo.Red.right.xy(:,1), 'ro', 'displayname', 'red-pixels-xr'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,3)
        try  plot(ugvStereo.time, ugvStereo.Red.left.xy(:,2), 'rs', 'displayname', 'red-pixels-yl'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,4)
        try  plot(ugvStereo.time, ugvStereo.Red.right.xy(:,2), 'rs', 'displayname', 'red-pixels-yr'); catch; end
        legend('toggle')
        grid on    
    grid on
    xlabel('time')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(703); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(703); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,2,1)
        try  plot(ugvStereo.time, ugvStereo.Blue.left.xy(:,1), 'bo', 'displayname', 'blue-pixels-xl'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,2)
        try  plot(ugvStereo.time, ugvStereo.Blue.right.xy(:,1), 'bo', 'displayname', 'blue-pixels-xr'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,3)
        try  plot(ugvStereo.time, ugvStereo.Blue.left.xy(:,2), 'bs', 'displayname', 'blue-pixels-yl'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,4)
        try  plot(ugvStereo.time, ugvStereo.Blue.right.xy(:,2), 'bs', 'displayname', 'blue-pixels-yr'); catch; end
        legend('toggle')
        grid on
    xlabel('time')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% figure(704); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(704); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,2,1)
        try  plot(ugvStereo.time, ugvStereo.Green.left.xy(:,1), 'go', 'displayname', 'green-pixels-xl'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,2)
        try  plot(ugvStereo.time, ugvStereo.Green.right.xy(:,1), 'go', 'displayname', 'green-pixels-xr'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,3)
        try  plot(ugvStereo.time, ugvStereo.Green.left.xy(:,2), 'gs', 'displayname', 'green--pixels-yl'); catch; end
        legend('toggle')
        grid on
    subplot(2,2,4)
        try  plot(ugvStereo.time, ugvStereo.Green.right.xy(:,2), 'gs', 'displayname', 'green-pixels-yr'); catch; end
        legend('toggle')
        grid on
    xlabel('time')
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end
%% disp('sorting pixel values by yaw cov')
% disp('sorting pixel values by yaw cov')
% try ugvMatlab.sorted = sort_pixels_by_yaw_uncert(ugvStereo, ugvMatlab); catch; end
% % figure(7002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(7002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Red.left.xy(:,1), 'ro', 'displayname', 'red-pixels-xl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Red.right.xy(:,1), 'ro', 'displayname', 'red-pixels-xr sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Red.left.xy(:,2), 'rs', 'displayname', 'red-pixels-yl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Red.right.xy(:,2), 'rs', 'displayname', 'red-pixels-yr sorted'); catch; end
%         legend('toggle')
%         grid on    
%     grid on
%     xlabel('time')
% if meta.saveplots
%     %%
%     current_fig = gcf;
%     saveas(gcf, '~/benjamin/git/hast/data/pixelcomp/red_exp_sorted.png'); 
%     print('-depsc', '~/benjamin/git/hast/data/pixelcomp/red_exp_sorted.eps'); 
%     clear current_fig;
% end
% % figure(7003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(7003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Blue.left.xy(:,1), 'bo', 'displayname', 'blue-pixels-xl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Blue.right.xy(:,1), 'bo', 'displayname', 'blue-pixels-xr sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Blue.left.xy(:,2), 'bs', 'displayname', 'blue-pixels-yl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Blue.right.xy(:,2), 'bs', 'displayname', 'blue-pixels-yr sorted'); catch; end
%         legend('toggle')
%         grid on
%     xlabel('time')
%     if meta.saveplots
%         current_fig = gcf;
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
%         clear current_fig;
%     end
% if meta.saveplots
%     %%
%     current_fig = gcf;
%     saveas(gcf, '~/benjamin/git/hast/data/pixelcomp/blue_exp_sorted.png'); 
%     print('-depsc', '~/benjamin/git/hast/data/pixelcomp/blue_exp_sorted.eps'); 
%     clear current_fig;
% end
% % figure(7004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(7004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Green.left.xy(:,1), 'go', 'displayname', 'green-pixels-xl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Green.right.xy(:,1), 'go', 'displayname', 'green-pixels-xr sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Green.left.xy(:,2), 'gs', 'displayname', 'green--pixels-yl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(ugvStereo.time, ugvMatlab.sorted.Green.right.xy(:,2), 'gs', 'displayname', 'green-pixels-yr sorted'); catch; end
%         legend('toggle')
%         grid on
%     xlabel('time')
%     if meta.saveplots
%         current_fig = gcf;
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
%         clear current_fig;
%     end
% if meta.saveplots
%     %
%     current_fig = gcf;
%     saveas(gcf, '~/benjamin/git/hast/data/pixelcomp/green_exp_sorted.png'); 
%     print('-depsc', '~/benjamin/git/hast/data/pixelcomp/green_exp_sorted.eps'); 
%     clear current_fig;
% end


