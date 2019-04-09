function [data, meta] = plotclas_func(data, meta)
tic
disp('Plotting...')
try aprilnames = fieldnames(data.vicon); catch; end
try tagnames = fieldnames(data.april); catch; end
csvk = 1;
for i = 1:length(aprilnames)
    try
        if strcmp(aprilnames{i}(1:5), 'april')
          aprilOrd{csvk,1} = aprilnames{i}(6:7);
          csvk = csvk+1;
        end
    catch
    end 
end

linespec = {'d'; '<'; '>'; '^'; 'V'};

%% generate some data for metrics
% try
data = gen_metrics(data);

    meta.icra_root = '/Users/benjamin/hast/tex/icra/icra2019/data/';
    metrics = data.metrics;
    try save( [meta.icra_root 'metrics_' [meta.date(1:end-1) '_'  meta.run] '.mat'] , 'metrics'); catch; end
    clear metrics
% catch
% end
%% Turn plotting back on
set(0, 'DefaultFigureVisible', 'on');
figHandles = findall(0, 'Type', 'figure');
set(figHandles(:), 'visible', 'on');
clear figHandles

disp('agent xyz positions in various frames')

    
%     if 1
%         return
%     end
%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 3;
    
        title([meta.date meta.run])
        hold on; ii = 0; 
%             try h1 = plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,1), 'go',  ...
%                     'LineWidth', linewidth, 'displayname', 'uav x stereo global'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%             catch; end
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,1), 'd', 'Color', [1,1,1], ...
                     'MarkerFaceColor', [0,0,0], 'displayname', 'uav x goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'uav x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'uav x estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
%         try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1),'k.', 'displayname', 'uav x vicon'); catch; end
% %         try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,1), 'go', 'displayname', 'uav x stereo'); catch; end
%         
% %         try plot(data.ugvStereo.time, data.ugvStereo.uav.P_gl(:,1), 'go', 'displayname', 'uav x stereo'); catch; end
%         try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), 'b.', 'displayname', 'uav x ckf est'); catch; end
% %         try plot(data.slam.time, data.slam.uav.est.p.global(:,1), 'rs', 'displayname', 'uav x slam'); catch; end
% %         try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,1), 'go', 'displayname', 'uav stereo x global'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), 'r.', 'displayname', 'global desired x'); catch; end
% %         try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,1), 'm.', 'displayname', 'global cmd x'); catch; end
        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('x [m]')
        grid on

%         try %current axis
%             current_limits = axis; current_axes = gca; 
%             axes = [current_limits(1) current_limits(2) -0.5 3];
%             current_axes.XTick = [0:10:axes(2)];
%             current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%             current_axes.YTick = [axes(3):0.5:axes(4)];
%             current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%         end
%% figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2),'k.', 'displayname', 'uav y vicon'); catch; end
%         try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,2), 'go', 'displayname', 'uav y stereo'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,2), 'go', 'displayname', 'uav y stereo global'); catch; end
%         try plot(data.ugvStereo.time, data.ugvStereo.uav.P_gl(:,2), 'go', 'displayname', 'uav y stereo'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), 'b.', 'displayname', 'uav y ckf est'); catch; end
%         try plot(data.slam.time, data.slam.uav.est.p.global(:,2), 'rs', 'displayname', 'uav y slam'); catch; end
%         try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,2), 'go', 'displayname', 'uav stereo y global'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), 'r.', 'displayname', 'global desired y'); catch; end
%         try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,2), 'm.', 'displayname', 'global cmd y'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), 'r.', 'displayname', 'global desired x'); catch; end
    hold off; grid on
    title(['uav y position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('Y [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
        current_limits = axis; current_axes = gca; 
        axes = [current_limits(1) current_limits(2) -0.5 0.5];
%         limit_step = 10;
%         current_axes.XTick = [0:10:90];
%         current_axes.XTick = [0:10:limit_step*ceil(([data.uavRecorder.est.time(end) data.vicon.uav.time(end) data.ugvRecorder.stereo.time(end)]/limit_step))];
%         current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%         current_axes.YTick = [axes(3):0.25:axes(4)];
%         current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
        clear current_axes current_limits limit_step
    end
%% figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,3),'k.', 'displayname', 'uav z vicon'); catch; end
%         try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,3), 'go', 'displayname', 'uav z stereo'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,3), 'go', 'displayname', 'uav z stereo global'); catch; end
%         try plot(data.ugvStereo.time, data.ugvStereo.uav.P_gl(:,3), 'go', 'displayname', 'uav z stereo'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'uav z ckf est'); catch; end

%         try plot(data.slam.time, data.slam.uav.est.p.global(:,3), 'rs', 'displayname', 'uav z slam'); catch; end
%         try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,3), 'go', 'displayname', 'uav stereo z global'); catch; end
%         try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.Alt, 'g.', 'displayname', 'echoAlt'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,3), 'r.', 'displayname', 'global desired z'); catch; end
%         try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,3), 'm.', 'displayname', 'global cmd z'); catch; end
    hold off; grid on
    title(['uav z position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
    legend('toggle');legend('Location', 'SouthEast')
%     
%     
    try %current axis
        current_limits = axis; current_axes = gca; 
        axes = [current_limits(1) current_limits(2) -0.5 2.0];
%         limit_step = 10;
%         current_axes.XTick = [0:10:90];
% %         current_axes.XTick = [0:10:limit_step*ceil(([data.uavRecorder.est.time(end) data.vicon.uav.time(end) data.ugvRecorder.stereo.time(end)]/limit_step))];
%         current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
        current_axes.YTick = [axes(3):0.5:axes(4)];
        current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%         clear current_axes current_limits limit_step
    end
%% figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugvRecorder.stereo.time, ...
                data.ugvRecorder.stereo.yaw_ugv + (data.vicon.ugvk.atStereoTime.yaw.global * 180/pi), ...
                'go','displayname', 'uav yaw stereo global'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, 'b.', 'displayname', 'uav yaw ckf est'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees,'k.', 'displayname', 'uav yaw vicon'); catch; end
%         try plot(data.slam.time, data.slam.uav.est.yaw.global, 'rs', 'displayname', 'uav yaw slam'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Yaw, 'r.', 'displayname', 'global desired yaw'); catch; end
%         try plot(data.uavCon.time, 1000*data.uavCon.cmd.yawrate, 'm.', 'displayname', 'global cmd (400x) '); catch; end
    hold off; grid on
    title(['uav yaw position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('degrees')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
        current_limits = axis; current_axes = gca; 
        axes = [current_limits(1) current_limits(2) -75 75];
%         limit_step = 10;
%         current_axes.XTick = [0:10:90];
% %         current_axes.XTick = [0:10:limit_step*ceil(([data.uavRecorder.est.time(end) data.vicon.uav.time(end) data.ugvRecorder.stereo.time(end)]/limit_step))];
%         current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
        current_axes.YTick = [axes(3):25:axes(4)];
        current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%         clear current_axes current_limits limit_step
    end
%% figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'k.', 'displayname', 'ugv x vicon'); catch; end
        try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'bx', 'displayname', 'ugv x ckf est'); catch; end
        try plot(data.kobuki_logger.cmd_vel.time, data.kobuki_logger.cmd_vel_limited.linear_x, 'o', 'displayname', 'ugv cmd vel linear x'); catch;end
    hold off; grid on
    legend('toggle'); legend('Location', 'NorthWest')
    xlabel('time [s]'); ylabel('X [m]')
    title(['ugv x position, global frame ' [meta.date meta.run]])
    try %current axis
        current_limits = axis; current_axes = gca; 
        current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
%         current_axes.YLim = [0 5];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
        clear current_axes current_limits
    end
    if meta.saveplots
        current_fig = gcf;
        meta.savefilename = ['UGVx_' meta.run];
        try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
%         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
        clear current_fig;
    end
%% figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,2), 'k.', 'displayname', 'ugv y vicon'); catch; end
        try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'bx', 'displayname', 'ugv y ckf est'); catch; end
    hold off; grid on
    legend('toggle'); legend('Location', 'NorthWest')
    xlabel('time [s]'); ylabel('Y [m]')
    title(['ugv y position, global frame ' [meta.date meta.run]])
%     try %current axis
%         current_limits = axis; current_axes = gca; 
%         current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
% %         current_axes.YLim = [0 5];
% %         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%     end
%     if meta.saveplots
%         current_fig = gcf;
%         meta.savefilename = ['UGVy_' meta.run];
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
% %         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
% %         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
%         clear current_fig;
%     end
%% figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.ugvk.time, data.vicon.ugvk.yaw.global*180/pi, 'k.', 'displayname', 'ugv yaw vicon'); catch; end
        try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'bx', 'displayname', 'ugv yaw ckf est'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Yaw, 'r.', 'displayname', 'global desired yaw'); catch; end
    hold off; grid on
    legend('toggle'); legend('Location', 'NorthWest')
    xlabel('time [s]'); ylabel('Yaw [degrees]')
    title(['ugv yaw orientation, global frame ' [meta.date meta.run]])
%     try %current axis
%         current_limits = axis; current_axes = gca; 
%         current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
% %         current_axes.YLim = [0 5];
% %         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%     end
%     if meta.saveplots
%         current_fig = gcf;
%         meta.savefilename = ['UGVyaw_' meta.run];
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
% %         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
% %         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
%         clear current_fig;
%     end
%% figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,1),'k.', 'displayname', 'uav x vicon'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_ugv(:,1), 'go', 'displayname', 'uav x stereo global'); catch; end
    hold off; grid on
    ylabel('X [m]'), xlabel('time [s]')
    title(['uav x position, ugv frame ' [meta.date meta.run]])
    legend('toggle');legend('Location', 'SouthEast')
%% figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,2),'k.', 'displayname', 'uav x vicon'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_ugv(:,2), 'go', 'displayname', 'uav x stereo global'); catch; end
    hold off; grid on
    ylabel('X [m]'), xlabel('time [s]')
    title(['uav x position, ugv frame ' [meta.date meta.run]])
    legend('toggle');legend('Location', 'SouthEast')
%% figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot( data.vicon.uav.P.global(:,1), data.vicon.uav.P.global(:,3),'k.', 'displayname', 'uav x vicon'); catch; end
        try plot(data.ugvStereo.uav.P_gl(:,1), data.ugvStereo.uav.P_gl(:,3),'go', 'displayname', 'uav x stereo'); catch; end
        try plot(data.ugvStereo.uav.P_ugv(:,1), data.ugvStereo.uav.P_ugv(:,3), 'go', 'displayname', 'uav x stereo'); catch; end
%         try plot(data.uavRecorder.est.Position_gl(:,1),data.uavRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'uav x ckf est'); catch; end
%         try plot(data.slam.time, data.slam.uav.est.p.global(:,1), 'rs', 'displayname', 'uav x slam'); catch; end
%         try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,1), 'go', 'displayname', 'uav stereo x global'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), 'r.', 'displayname', 'global desired x'); catch; end
%         try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,1), 'm.', 'displayname', 'global cmd x'); catch; end
    hold off; grid on
    ylabel('z [m]'), xlabel('x [m]')
    title(['uav x position, global frame ' [meta.date meta.run]])
    legend('toggle');legend('Location', 'SouthEast')%% figure(14); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%% figure(11); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(11); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav position in ugv frame')
    try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv, 'x', 'displayname', 'x'); catch; end
    try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv, 'o', 'displayname', 'y'); catch; end
    try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv, '.', 'displayname', 'z'); catch; end
%% figure(14); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(14); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav yaw ugv frame ' [meta.date meta.run]])    
    ylabel('degrees')
    hold on
        try plot(data.ugvStereo.time, data.ugvStereo.uav.yaw_ugv, 'o', 'displayname', 'uav yaw stereo'); catch; end
        try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'b.', 'displayname', 'ugv yaw estimated'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(15); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(15); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav yaw global frame ' [meta.date meta.run]])    
    ylabel('degrees')
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'kx', 'displayname', 'uav vicon global'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, 's', 'displayname', 'uav est yaw'); catch ;end
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.CompassYaw, 'gd', 'displayname', 'navdata-compass-yaw' ); catch; end
        try plot(data.uavRecorder.est.time, data.vicon.est_yaw.spline_error, '.', 'displayname', 'uav yaw est error' ); catch; end

    hold off
    grid on
    legend('toggle')
%% figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav yaw command analysis, global frame ' [meta.date meta.run]])    
    ylabel('degrees')
    hold on
%         try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'kx', 'displayname', 'uav yaw vicon'); catch; end
        try plot(data.uavCon.time, data.uavCon.Current.Yaw, 'bx', 'displayname', 'uav yaw est'); catch; end
        try plot(data.uavCon.time, 100*data.uavCon.cmd.angular_g(:,3), 'ks', 'displayname', 'uav yaw cmd'); catch ;end
        try plot(data.uavCon.time, data.uavCon.Desired.Yaw, 'r.', 'displayname', 'uav yaw desired'); catch; end
%         try plot(data.uavCon.time, 100*data.uavCon.cmd.yawrate, 'mo', 'displayname', 'uav cmd yawrate'); catch; end

    hold off
    grid on
    legend('toggle')
    try %current axis
        current_limits = axis; current_axes = gca; 
        current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
%         current_axes.YLim = [0 5];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
        clear current_axes current_limits

    end
%% figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav pos-x-PID commands (uav frame)')
    hold on
        try plot(data.uavCon.time, data.uavCon.Current.Position_dr(:,1), '.', 'displayname', 'uav current P x'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_dr(:,1), '.', 'displayname', 'uav desired P x'); catch; end
        try plot(data.uavCon.time, data.uavCon.pos_PID.u_cmd(:,1), 'x', 'displayname', 'uav u-cmd'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,1), 'd', 'displayname', 'uav cmd-vel'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav pos-y-PID commands (uav frame)')
    hold on
        try plot(data.uavCon.time, data.uavCon.Current.Position_dr(:,2), '.', 'displayname', 'uav current P x'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_dr(:,2), '.', 'displayname', 'uav desired P x'); catch; end
        try plot(data.uavCon.time, data.uavCon.pos_PID.u_cmd(:,2), 'x', 'displayname', 'uav u-cmd'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,2), 'd', 'displayname', 'uav cmd-vel'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(19); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(19); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav pos-x-PID commands (uav frame)')
    hold on
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.V(:,1)/1000, '.', 'displayname', 'uav V x'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,1), 'd', 'displayname', 'uav X cmd-vel'); catch; end

    hold off
    grid on
    legend('toggle')
%% figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    title('uav pos-y-PID commands (uav frame)')
    hold on
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.V(:,2)/1000, '.', 'displayname', 'uav V x'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,2), 'd', 'displayname', 'uav cmd-vel'); catch; end

    hold off
    grid on
    legend('toggle')
%% figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    title('uav commands x (uav frame)')
    hold on
        try plot(data.uavCon.cmd.time, data.uavCon.cmd.linear(:,1), 'd', 'displayname', 'cmd-x'); catch; end
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.V(:,1)/1000, '.', 'displayname', 'uav V x'); catch; end
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.RPY(:,1), '.', 'displayname', 'uav RPY1'); catch; end
%         try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.RPY(:,2), '.', 'displayname', 'uav RPY2'); catch; end


    hold off
    grid on
    legend('toggle')
%% figure(22); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(22); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    title('uav commands y (uav frame)')
    hold on
        try plot(data.uavCon.cmd.time, data.uavCon.cmd.linear(:,2), 'd', 'displayname', 'cmd-y'); catch; end
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.V(:,2)/1000, '.', 'displayname', 'uav V y'); catch; end
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.RPY(:,2), '.', 'displayname', 'uav RPY2'); catch; end


    hold off
    grid on
    legend('toggle')
%% figure(23); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(23); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    title('uav commands x (uav frame)')
    hold on
        try plot(data.uavCon.cmd.time, data.uavCon.cmd.linear(:,3), 'd', 'displayname', 'cmd-z'); catch; end
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.V(:,3)/1000, '.', 'displayname', 'uav V x'); catch; end
%         try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.RPY(:,1), '.', 'displayname', 'uav RPY1'); catch; end
%         try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.RPY(:,2), '.', 'displayname', 'uav RPY2'); catch; end


    hold off
    grid on
    legend('toggle')
%% figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on
            try plot(data.uavRecorder.est.time, data.vicon.uav.splines.P.global.x_diff, 'o', 'displayname', 'uav x estimate error'); catch; end    
            try plot(data.uavRecorder.est.time, data.vicon.uav.splines.P.global.y_diff, 'o', 'displayname', 'uav y estimate error'); catch; end    
            try plot(data.uavRecorder.est.time, data.vicon.uav.splines.P.global.z_diff, 'o', 'displayname', 'uav z estimate error'); catch; end   
        hold off
        grid on
        legend('toggle')
        legend('location', 'southeast')
        title(['stereo errors of UAV (ugv frame) ' meta.date meta.run])
        ylabel('linear difference [m]')

%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XLim = [0 data.ugvStereo.time(end)];
%             current_axes.YLim = [-1 1];
%             current_axes.XTick = [20:10:10*ceil(data.ugvStereo.time(end)/10)];
%             
%             current_axes.YTick = [-0.5:0.5:0.5];
%             
%             clear current_axes current_limits
%         end
    subplot(2,1,2)
        hold on
            try plot(data.ugvRecorder.stereo.time, data.vicon.diff_yaw.spline_error, 'o-', 'displayname', 'uav yaw error'); catch; end    
            
            
            
        hold off
        grid on
        xlabel('time [s]')
        ylabel('angular difference [deg]')
        legend('toggle')
        legend('location', 'southeast')
        try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XTick = [0 10 20 30 40 50 60];
%             current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
%             current_axes.YTick = [-10:2.5:10];
%             current_axes.YLim = [-10 10];
%             clear current_axes current_limits
        end
%% figure(28); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(28); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on
            try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.x_diff_stereotime, 'ro', 'displayname', 'x'); catch; end    
            try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.y_diff_stereotime, 'bo', 'displayname', 'y'); catch; end    
            try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.z_diff_stereotime, 'ko', 'displayname', 'z'); catch; end   
        hold off
        grid on
        legend('toggle')
        legend('location', 'southeast')
        title(['stereo errors of UAV (ugv frame) ' meta.date meta.run])
        ylabel('linear difference [m]')
try     disp(['ugvstereo.uav mean differences' ...
        ', x: ' num2str(mean(data.vicon.uav.splines.P.global.x_diff_stereotime)) ...
        ', y :' num2str(mean(data.vicon.uav.splines.P.global.y_diff_stereotime)) ...
        ', z :' num2str(mean(data.vicon.uav.splines.P.global.z_diff_stereotime))]); catch; end

%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
% %             current_axes.XTick = [0 10 20 30 40 50 60];
%             current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
%             current_axes.YLim = [-0.5 0.5];
%             current_axes.YTick = [current_axes.YLim(1):0.25:current_axes.YLim(2)];
%             clear current_axes current_limits
%         end
    subplot(2,1,2)
        hold on
            try plot(data.ugvStereo.time, data.vicon.uav.splines.yaw.global.error_stereotime, 'ko', 'displayname', 'yaw'); catch; end   
        hold off
        grid on
        xlabel('time [s]')
        ylabel('angular difference [deg]')
        legend('toggle')
        legend('location', 'southeast')
        try
            current_limits = axis;
            current_axes = gca; % current axes
%             current_axes.XTick = [0 10 20 30 40 50 60];
            current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
            current_axes.YLim = [-50 50];
            current_axes.YTick = [current_axes.YLim(1):25:current_axes.YLim(2)];
            clear current_axes current_limits
        end    
    if meta.saveplots
        current_fig = gcf;
        meta.savefilename = ['UAVerrors_gl_' meta.run];
        try     saveas(gcf, [meta.dataroot meta.date meta.savefilename '.png']); catch; end
%         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
        clear current_fig;
    end
%% figure(29); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(29); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on
            try plot(data.vicon.uav.splines.P.cam.z_uavstereotime, abs(data.vicon.uav.splines.P.global.x_diff_stereotime), 'r.', 'displayname', 'uav x diff btw stereo and vicon'); catch; end    
            try plot(data.vicon.uav.splines.P.cam.z_uavstereotime, abs(data.vicon.uav.splines.P.global.y_diff_stereotime), 'b.', 'displayname', 'uav y diff btw stereo and vicon'); catch; end    
            try plot(data.vicon.uav.splines.P.cam.z_uavstereotime, abs(data.vicon.uav.splines.P.global.z_diff_stereotime), 'k.', 'displayname', 'uav z diff btw stereo and vicon'); catch; end   
            
            
        hold off
        grid on
        legend('toggle')
        legend('location', 'southeast')
        title(['stereo errors of UAV (ugv frame) vs range ' meta.date meta.run])
        ylabel('linear difference [m]')

%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
% %             current_axes.XTick = [0 10 20 30 40 50 60];
% %             current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
%             current_axes.YLim = [-0.5 0.5];
%             current_axes.YTick = [current_axes.YLim(1):0.2:current_axes.YLim(2)];
%             clear current_axes current_limits
%         end
    subplot(2,1,2)
        hold on
            try plot(data.vicon.uav.splines.P.cam.z_uavstereotime, abs(data.vicon.uav.splines.yaw.global.error_stereotime), 'k.', 'displayname', 'uav yaw diff btw stereo and vicon'); catch; end    
        hold off
        grid on
        xlabel('range [m]')
        ylabel('angular difference [deg]')
        legend('toggle')
        legend('location', 'southeast')
%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
% %             current_axes.XTick = [0 10 20 30 40 50 60];
% %             current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
%             current_axes.YLim = [-50 50];
%             current_axes.YTick = [current_axes.YLim(1):20:current_axes.YLim(2)];
%             clear current_axes current_limits
%         end
%% figure(30); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%% disp('uav ckf debug plots')
disp('uav ckf debug plots')
%% figure(31); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(31); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Yaw')
    hold on
        try plot(data.uavRecorder.oneckf.time, data.uavRecorder.oneckf.Yaw, 'displayname', 'oneCKF Yaw'); catch; end
        try plot(data.uavRecorder.oneckf.time, data.uavRecorder.oneckf.PosteriorEst(:,4), 'displayname', 'PosteriorEst4'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(32); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(32); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Yaw Bias')
    hold on
        try plot(data.uavRecorder.oneckf.time, data.uavRecorder.oneckf.YawBias, 'displayname', 'EstYawBias'); catch; end
        try plot(data.uavRecorder.oneckf.time, data.uavRecorder.oneckf.PosteriorEst(:,5), 'displayname', 'PosteriorEst5'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(33); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(33); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Rk_re(:,1), 'o-', 'displayname', 'Rk (1,1)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Qdk_re(:,1), 's-', 'displayname', 'Qdk(1,1)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.PosteriorCov_re(:,1), 'd-.', 'displayname', 'PosteriorCov(1,1)'); catch; end
        try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.ckf.Qdk(:,1), 's-', 'displayname', 'Qdk_i(1,1)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([data.uavRecorder.ckf.time(1)-3 data.uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
%% figure(34); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(34); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Rk_re(:,6), 'o-', 'displayname', 'Rk (2,2)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Qdk_re(:,7), 's-', 'displayname', 'Qdk(2,2)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.PosteriorCov_re(:,7), 'd-.', 'displayname', 'PosteriorCov(2,2)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([data.uavRecorder.ckf.time(1)-3 data.uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end
%% figure(36); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(36); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Rk_re(:,11), 'o-', 'displayname', 'Rk (3,3)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Qdk_re(:,13), 's-', 'displayname', 'Qdk(3,3)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.PosteriorCov_re(:,13), 'd-.', 'displayname', 'PosteriorCov(3,3)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([data.uavRecorder.ckf.time(1)-3 data.uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
%% figure(37); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(37); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Rk_re(:,16), 'o-', 'displayname', 'Rk (4,4)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Qdk_re(:,19), 's-', 'displayname', 'Qdk(4,4)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.PosteriorCov_re(:,19), 'd-.', 'displayname', 'PosteriorCov(4,4)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.PosteriorCov_re(:,25), 'x-.', 'displayname', 'PosteriorCov(5,5)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([data.uavRecorder.ckf.time(1)-3 data.uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end
%% figure(38); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(38); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav ckf qw and rk')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Qdk_re(:,25), 's-', 'displayname', 'Qdk(4,4)'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.PosteriorCov_re(:,25), 'd-.', 'displayname', 'PosteriorCov(5,5)'); catch; end
    hold off
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([data.uavRecorder.ckf.time(1)-3 data.uavRecorder.ckf.time(end)+3 current_axes(3) current_axes(4)]); catch; end
    clear current_axes
%% disp('SLAM')
disp('SLAM')
%% figure(48); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(48); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['Estimated tag yaw' [meta.date meta.run]])
    hold on

    
        try plot(data.slam.time, data.april.tag_02.EstYaw_gl, 's-', 'displayname', 'tag02 yaw est'); catch; end
        try plot(data.slam.time, data.april.tag_03.EstYaw_gl, 's-', 'displayname', 'tag03 yaw est'); catch; end
        try plot(data.slam.time, data.april.tag_06.EstYaw_gl, 's-', 'displayname', 'tag06 yaw est'); catch; end
        try plot(data.slam.time, data.april.tag_07.EstYaw_gl, 's-', 'displayname', 'tag07 yaw est'); catch; end
        
    hold off 
    grid on
    xlabel('x position (m)'); 
    ylabel('y position (m)'); 
    legend('toggle')
    legend('location', 'Northwest')
%% figure(49); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(49); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['Estimated UGV Position XY' [meta.date meta.run]])
    hold on
        try plot(data.ugvRecorder.est.Position_gl(:,1), data.ugvRecorder.est.Position_gl(:,2),'s', 'displayname', 'ugv est global xy'); catch; end


%         try plot(data.april.tag_02.MeasPosition_gl(:,1), data.april.tag_02.MeasPosition_gl(:,2),'o', 'displayname', 'april02 meas global xy'); catch; end
%         try plot(data.april.tag_03.MeasPosition_gl(:,1), data.april.tag_03.MeasPosition_gl(:,2),'o', 'displayname', 'april02 meas global xy'); catch; end
    
        try plot(data.april.tag_02.EstPosition_gl(:,1), data.april.tag_02.EstPosition_gl(:,2),'s', 'displayname', 'april02 est global xy'); catch; end
        try plot(data.april.tag_03.EstPosition_gl(:,1), data.april.tag_03.EstPosition_gl(:,2),'s', 'displayname', 'april03 est global xy'); catch; end
        try plot(data.april.tag_06.EstPosition_gl(:,1), data.april.tag_06.EstPosition_gl(:,2),'s', 'displayname', 'april06 est global xy'); catch; end
        try plot(data.april.tag_07.EstPosition_gl(:,1), data.april.tag_07.EstPosition_gl(:,2),'s', 'displayname', 'april07 est global xy'); catch; end
        
        try plot(data.vicon.april02_.P.global(:,1), data.vicon.april02_.P.global(:,2),'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 15, ...
                'displayname', 'april02 vicon global xy'); catch; end
        try plot(data.vicon.april03_.P.global(:,1), data.vicon.april03_.P.global(:,2),'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 15, ...
                'displayname', 'april03 vicon global xy'); catch; end
        try plot(data.vicon.april06_.P.global(:,1), data.vicon.april06_.P.global(:,2),'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 15, ...
                'displayname', 'april06 vicon global xy'); catch; end
        try plot(data.vicon.april07_.P.global(:,1), data.vicon.april07_.P.global(:,2),'ks', 'MarkerFaceColor', 'k', 'MarkerSize', 15, ...
                'displayname', 'april07 vicon global xy'); catch; end

        try plot(data.vicon.ugvk.P.global(:,1), data.vicon.ugvk.P.global(:,2), 'k.', 'displayname', 'ugv vicon global') ; catch; end                
        
    hold off 
    grid on
    xlabel('x position (m)'); 
    ylabel('y position (m)'); 
    legend('toggle')
    legend('location', 'Northwest')
%     axis([-0.5 3.5 -0.5 3.5])
%% figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['Estimated UGV Position XY' [meta.date meta.run]])
    hold on
        try plot(data.ugvRecorder.wheel.P_odom(:,1), data.ugvRecorder.wheel.P_odom(:,2), 'LineWidth',4, 'displayname', 'ugv wheel odom xy'); catch; end
        try plot(data.ugvRecorder.est.Position_gl(:,1), data.ugvRecorder.est.Position_gl(:,2),'s', 'displayname', 'ugv est global xy'); catch; end
        try plot(data.vicon.ugvk.P.global(:,1), data.vicon.ugvk.P.global(:,2), '.', 'displayname', 'ugv vicon global') ; catch; end
    hold off 
    grid on
    xlabel('x position (m)'); 
    ylabel('y position (m)'); 
    legend('toggle')
    legend('location', 'Northwest')
%     current_axes = axis;
%     try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%     clear current_axes
% 	axis([-3 .5 -1 1])
%     axis([-0.5 1.5 -1 1])
%     axis([-0.25 1 -0.1 0.1])
        
%     if 0
%         %%
%         current_fig = gcf;
%         meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
%         saveas(gcf, [meta.figpath meta.savefilename '.png']); 
%         clear current_fig;
%     end
%% figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
        title(['Estimated Position (x vs time)' [meta.date meta.run]])
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'Global Vicon Position'); catch; end
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.P_odom(:,1), '*-', 'displayname', 'Global Wheel Odometry'); catch; end    
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 's', 'displayname', 'ugv x est'); catch; end
%             try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,1), 'x', 'displayname', 'ugv x ckf'); catch; end
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
%% figure(52); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(52); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
        title(['Estimated Position (y vs time)' [meta.date meta.run]])
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,2), 'd-', 'displayname', 'Global Vicon Position y'); catch; end
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.P_odom(:,2), '*-', 'displayname', 'Global Wheel Odometry y'); catch; end    
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 's', 'displayname', 'ugv y est'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,2), 'x', 'displayname', 'ugv y ckf'); catch; end

        hold off 
        grid on
        xlabel('time (sec)'); 
        ylabel('y position (m)'); 
        legend('toggle')
        legend('location', 'northwest')
        current_axes = axis;
        try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
        clear current_axes

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(53); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(53); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
        title(['Estimated Position (z vs time)' [meta.date meta.run]])
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,3), 'd-', 'displayname', 'Global Vicon Position z'); catch; end
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.P_odom(:,3), '*-', 'displayname', 'Global Wheel Odometry z'); catch; end    
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,3), 's', 'displayname', 'ugv z est'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,3), 'x', 'displayname', 'ugv z ckf'); catch; end

        hold off 
        grid on
        xlabel('time (sec)'); 
        ylabel('z position (m)'); 
        legend('toggle')
        legend('location', 'northwest')
        current_axes = axis;
        try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
        clear current_axes

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(54); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(54); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
        title(['Estimated Position (yaw vs time)' [meta.date meta.run]])
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.yaw.global*180/pi, 'd-', 'displayname', 'Global Vicon Yaw'); catch; end
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.yaw*180/pi, '*-', 'displayname', 'ugv wheel odometry yaw'); catch; end
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.EstYaw_gl, '*-', 'displayname', 'ugv EstYaw-gl'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 's', 'displayname', 'ugv yaw est'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Yaw, 'x', 'displayname', 'ugv yaw ckf'); catch; end
%             try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.yaw_q, 'x', 'displayname', 'ugv yaw q'); catch; end
            
        hold off 
        grid on
        legend('toggle')
        legend('location', 'northwest')
        xlabel('time (sec)'); 
        ylabel('yaw position (deg)'); 
        current_axes = axis;
        try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
        clear current_axes

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(55); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(55); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('Mech and Aiding, ugvframe x')
        hold on
            try plot(data.ckfRecorder.time, data.ckfRecorder.Mech(:,1), 's-', 'displayname', 'ckf mech x'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.Aiding(:,1), 'x-', 'displayname', 'ckf aid x'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.zk(:,1), 'h-', 'displayname', 'ugv zk x'); catch; end
            
%             try plot(data.ckfRecorder.time, data.ckfRecorder.ugv.correction_gl(:,1), 'ks-', 'displayname', 'ugv correction-gl x'); catch; end
%             
%             try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Mech(:,1), 's-', 'displayname', 'ugv mech x'); catch; end
%             try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Aiding(:,1), 'o-', 'displayname', 'ugv aid x'); catch; end
%             try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'd-', 'displayname', 'ugv est x'); catch; end
%             try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorEst(:,1), 's-', 'displayname', 'ugv PosteriorEst x'); catch; end
        hold off
        grid on
        legend('toggle')
        current_axes = axis;
        try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%         try axis([1 35 -10 10]); catch; end
        clear current_axes
        

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv Mech and Aiding, y')
        hold on
            try plot(data.ckfRecorder.time, data.ckfRecorder.Mech(:,2), 's-', 'displayname', 'ugv mech y'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.Aiding(:,2), 'x-', 'displayname', 'ugv aid y'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.PosteriorEst(:,2), 'h-', 'displayname', 'ugv PosteriorEst y'); catch; end

            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Mech(:,2), 's-', 'displayname', 'ugv mech y'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Aiding(:,2), 'o-', 'displayname', 'ugv aid y'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'd-', 'displayname', 'ugv est y'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorEst(:,2), 'h-', 'displayname', 'ugv PosteriorEst y'); catch; end
        hold off
        grid on
        legend('toggle')
        current_axes = axis;
        try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
        clear current_axes

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(57); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(57); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv Mech and Aiding, z')
        hold on
            try plot(data.ckfRecorder.time, data.ckfRecorder.Mech(:,3), 's-', 'displayname', 'ugv mech z'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.Aiding(:,3), 'x-', 'displayname', 'ugv aid z'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.PosteriorEst(:,3), 'h-', 'displayname', 'ugv PosteriorEst z'); catch; end

            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Mech(:,3), 's-', 'displayname', 'ugv mech z'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Aiding(:,3), 'o-', 'displayname', 'ugv aid z'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,3), 'd-', 'displayname', 'ugv est z'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorEst(:,3), 's-', 'displayname', 'ugv PosteriorEst z'); catch; end
        hold off
        grid on
        legend('toggle')
        current_axes = axis;
        try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
        clear current_axes

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(58); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(58); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv Mech and Aiding, yaw')
        hold on
            try plot(data.ckfRecorder.time, data.ckfRecorder.Mech(:,4), 's-', 'displayname', '1ckf mech yaw'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.Aiding(:,4), 'x-', 'displayname', '1ckf aid yaw'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.PosteriorEst(:,4), 'h-', 'displayname', '1ckf PosteriorEst yaw'); catch; end

            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Mech(:,4), 's-', 'displayname', 'ugv mech yaw'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Aiding(:,4), 'o-', 'displayname', 'ugv aid yaw'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'd-', 'displayname', 'ugv est yaw'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorEst(:,4), 'h-', 'displayname', 'ugv PosteriorEst yaw'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.zk(:,4), 'x-', 'displayname', 'ugv zk yaw'); catch; end

        hold off
        grid on
        legend('toggle')
        current_axes = axis;
        try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
        clear current_axes

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(61); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(61); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on
        yyaxis left; axleft = gca; ylabel(axleft, 'Qdk'); %ylim(axleft, [-0.5 0.5]);        
            try plot(data.ckfRecorder.time, data.ckfRecorder.ugv.Qdk_re(1,:), 'o-', 'displayname', 'ugv Qdk(1,1)'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.uav.Qdk_re(1,:), 'x-', 'displayname', 'uav Qdk(1,1)'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.Qdk_re(1,:), 'h-', 'displayname', 'cQdk(1,1)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Rk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(1,:), 's-', 'displayname', 'Rk(1,1)'); catch; end
        hold off
        grid on
        legend('toggle')
%         current_axes = axis;
%         try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%         clear current_axes
%% figure(62); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(62); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on
        yyaxis left; axleft = gca; ylabel(axleft, 'Qdk'); %ylim(axleft, [-0.5 0.5]);        
            try plot(data.ckfRecorder.time, data.ckfRecorder.ugv.Qdk_re(6,:), 'o-', 'displayname', 'ugv Qdk(2,2)'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.uav.Qdk_re(6,:), 'x-', 'displayname', 'uav Qdk(2,2)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Rk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(6,:), 's-', 'displayname', 'Rk(2,2)'); catch; end
        hold off
        grid on
        legend('toggle')
%         current_axes = axis;
%         try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%         clear current_axes
%% figure(63); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(63); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on
        yyaxis left; axleft = gca; ylabel(axleft, 'Qdk'); %ylim(axleft, [-0.5 0.5]);        
            try plot(data.ckfRecorder.time, data.ckfRecorder.ugv.Qdk_re(11,:), 'o-', 'displayname', 'ugv Qdk(3,3)'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.uav.Qdk_re(11,:), 'x-', 'displayname', 'uav Qdk(3,3)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Rk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(11,:), 's-', 'displayname', 'Rk(3,3)'); catch; end
        hold off
        grid on
        legend('toggle')
%         current_axes = axis;
%         try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%         clear current_axes
%% figure(64); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(64); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on
        yyaxis left; axleft = gca; ylabel(axleft, 'Qdk'); %ylim(axleft, [-0.5 0.5]);        
            try plot(data.ckfRecorder.time, data.ckfRecorder.ugv.Qdk_re(16,:), 'o-', 'displayname', 'ugv Qdk(4,4)'); catch; end
            try plot(data.ckfRecorder.time, data.ckfRecorder.uav.Qdk_re(16,:), 'x-', 'displayname', 'uav Qdk(4,4)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Rk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(16,:), 's-', 'displayname', 'Rk(4,4)'); catch; end
        hold off
        grid on
        legend('toggle')
%         current_axes = axis;
%         try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%         clear current_axes
%% figure(65); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(65); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on 
            try plot(data.ckfRecorder.time, data.ckfRecorder.Qdk_re(1,:), 'h-', 'displayname', 'cQdk(1,1)'); catch; end
%             try plot(data.ckfRecorder.time, data.ckfRecorder.ugv.Qdk_re(1,:), 'o-', 'displayname', 'ugv Qdk(1,1)'); catch; end
%             try plot(data.ckfRecorder.time, data.ckfRecorder.uav.Qdk_re(1,:), 'x-', 'displayname', 'uav Qdk(1,1)'); catch; end
%             try plot(data.ckfRecorder.time, data.ckfRecorder.ugv.correction_gl(:,1), 'ks-', 'displayname', 'ugv correction-gl x'); catch; end
            
        hold off
        grid on
        legend('toggle')
%         current_axes = axis;
%         try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
%         clear current_axes
%% figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv autopilot, x')
        hold on
            try plot(data.ugvAutopilot.guide.time, data.ugvAutopilot.guide.estP_gl(:,1), 's', 'displayname', 'ugv-x-est'); catch; end
            try plot(data.ugvAutopilot.guide.time, data.ugvAutopilot.guide.goalP_gl(:,1), 'o', 'displayname', 'ugv-x-goal'); catch; end
            try plot(data.ugvAutopilot.guide.time, data.ugvAutopilot.guide.errorP_gl(:,1), 'x', 'displayname', 'ugv-x-erroe'); catch; end
        hold off
        grid on
        legend('toggle')
        legend('Location', 'northwest')
%% figure(67); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(67); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv wheel velocities')
        hold on
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.twist.linear_xyz(:,1), 'x', 'displayname', 'ugv-x-vel'); catch; end
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.twist.angular_xyz(:,3), 'x', 'displayname', 'ugv-w-vel'); catch; end
        hold off
        grid on
        legend('toggle')
        legend('Location', 'northwest')
%% figure(68); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(68); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv autopilot, yaw')
        hold on

            try plot(data.ugvAutopilot.guide.time, data.ugvAutopilot.guide.yawEst, 's', 'displayname', 'ugv-yaw-est'); catch; end
            try plot(data.ugvAutopilot.guide.time, data.ugvAutopilot.guide.yawFacing, 'o', 'displayname', 'ugv-yaw-facing'); catch; end
            try plot(data.ugvAutopilot.guide.time, data.ugvAutopilot.guide.yawError, 'x', 'displayname', 'ugv-yaw-error'); catch; end
            try plot(data.ugvAutopilot.guide.time, data.ugvAutopilot.guide.phiGain, 'x', 'displayname', 'ugv-yaw-gain'); catch; end
        hold off
        grid on
        legend('toggle')
        legend('Location', 'northwest') 
%% figure(70); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(70); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        subplot(4,1,1)
        title(['ugv motion estimates ' meta.date meta.run])
            hold on
                try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
                try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'rs', ...
                        'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
                try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,1), 'ks', ...
                        'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
            hold off
            legend('toggle')
            legend('Location', 'northeast')
            ylabel('x [m]')
            try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
            current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
            current_axes.YLim = [-.1 .1];
            current_axes.YTick = [current_axes.YLim(1): 0.05 : current_axes.YLim(2)];            
            clear current_axes current_limits
            catch; end

        subplot(4,1,2)
            hold on
                try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
                try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'rs', ...
                        'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
                try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,2), 'ks', ...
                        'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
            hold off
            ylabel('y [m]')
            try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
            current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
            current_axes.YLim = [-.1 .1];
            current_axes.YTick = [current_axes.YLim(1): 0.05 : current_axes.YLim(2)];
            clear current_axes current_limits
            catch; end

        subplot(4,1,3)
            hold on
                try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
                try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,3), 'rs', ...
                        'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
                try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,3), 'ks', ...
                        'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
            hold off
            ylabel('z [m]')
            try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
            current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
            current_axes.YLim = [-.1 .1];
            current_axes.YTick = [current_axes.YLim(1): 0.05 : current_axes.YLim(2)];
            clear current_axes current_limits
            catch; end

        subplot(4,1,4)
            hold on
                try plot(data.vicon.ugvk.time, data.vicon.ugvk.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
                try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'rs', ...
                        'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
                try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Yaw, 'ks', ...
                        'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
            hold off
            ylabel('heading [deg]')
            try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
            current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
            current_axes.YTick = -2:6:34;
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits
            catch; end
            xlabel('time [s]')

    if meta.saveirosplots
            %%
            current_fig = gcf;
            meta.saveplotroot = ['/media/benjamin/devsdb/hast/tex/iros2017'];
            meta.savefilename = ['figure_14'];

            try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
            try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
            try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
            clear current_fig;
    end
%% figure(71); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(71); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(4,1,1)
%     title(['ugv motion estimates ' meta.date meta.run])
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,1), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('x [m]')
        try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
            current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
            current_axes.YLim = [-.25 1.0];
            current_axes.YTick = current_axes.YLim(1):0.25:current_axes.YLim(2);
            clear current_axes current_limits
        catch; end

        legend('toggle')
        legend('Location', 'northeast')
    subplot(4,1,2)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,2), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('y [m]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
        current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
        current_axes.YTick = [-.25 0 .25];
        current_axes.YLim = [-.25 .25];
        clear current_axes current_limits
        catch; end

    subplot(4,1,3)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,3), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,3), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('z [m]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
        current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
        current_axes.YTick = [-.25 0 .25];
        current_axes.YLim = [-.25 .25];
        clear current_axes current_limits
        catch; end

    subplot(4,1,4)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Yaw, 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('heading [deg]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
        current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
        current_axes.YLim = [-2 2];
        current_axes.YTick = current_axes.YLim(1):1:current_axes.YLim(2);

        clear current_axes current_limits
        catch; end
        xlabel('time [s]')

        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end    
if meta.saveirosplots
        %%
        current_fig = gcf;
        meta.saveplotroot = ['/media/benjamin/devsdb/hast/tex/iros2017'];
        meta.savefilename = ['figure_15'];

        try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
        try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
        try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
        clear current_fig;
end
% meta.saveplots = false;
% end
%% figure(72); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(72); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(4,1,1)
%     title(['ugv motion estimates ' meta.date meta.run])
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,1), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('x [m]')
        try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
            current_axes.XTick = floor(current_axes.XLim(1)-1):floor(current_axes.XLim(2)+2);
            current_axes.YLim = [-.25 .5];
            current_axes.YTick = current_axes.YLim(1):0.25:current_axes.YLim(2);
            clear current_axes current_limits
        catch; end
        legend('toggle')
        legend('Location', 'northeast')
    subplot(4,1,2)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,2), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('y [m]')
        try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
            current_axes.XTick = floor(current_axes.XLim(1)-1):floor(current_axes.XLim(2)+2);
            current_axes.YLim = [-.25 .25];
            current_axes.YTick = current_axes.YLim(1):0.25:current_axes.YLim(2);
            clear current_axes current_limits
        catch; end

    subplot(4,1,3)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,3), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,3), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('z [m]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
        current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
        current_axes.YTick = [-.25 0 .25];
        current_axes.YLim = [-.25 .25];
        clear current_axes current_limits
        catch; end

    subplot(4,1,4)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Yaw, 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('heading [deg]')
        try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
        current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
        current_axes.YLim = [-2 38];
        current_axes.YTick = current_axes.YLim(1):10:current_axes.YLim(2);

        clear current_axes current_limits
        catch; end
        xlabel('time [s]')
%% figure(73); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(73); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,2,1)
%     title(['ugv motion estimates ' meta.date meta.run])
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,1), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('x [m]')
%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
%             current_axes.XTick = floor(current_axes.XLim(1)-1):floor(current_axes.XLim(2)+2);
%             current_axes.YLim = [-.25 .5];
%             current_axes.YTick = current_axes.YLim(1):0.25:current_axes.YLim(2);
%             clear current_axes current_limits
%         catch; end
        legend('toggle')
        legend('Location', 'northeast')
    subplot(2,2,2)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,2), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('y [m]')
%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
%             current_axes.XTick = floor(current_axes.XLim(1)-1):floor(current_axes.XLim(2)+2);
%             current_axes.YLim = [-.25 .25];
%             current_axes.YTick = current_axes.YLim(1):0.25:current_axes.YLim(2);
%             clear current_axes current_limits
%         catch; end

    subplot(2,2,3)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,3), 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,3), 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('z [m]')
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
%         current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
%         current_axes.YTick = [-.25 0 .25];
%         current_axes.YLim = [-.25 .25];
%         clear current_axes current_limits
%         catch; end

    subplot(2,2,4)
        hold on
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'rs', ...
                    'MarkerFaceColor', 'r', 'displayname', 'estimated ugv trajectory'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Yaw, 'ks', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('heading [deg]')
%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XTick = floor(data.ugvRecorder.ckf.time(1)-1):floor(data.ugvRecorder.ckf.time(end)+2);
%             current_axes.XLim = [data.ugvRecorder.ckf.time(1)-1 data.ugvRecorder.ckf.time(end)+2];
%             current_axes.YLim = [-2 38];
%             current_axes.YTick = current_axes.YLim(1):10:current_axes.YLim(2);
% 
%             clear current_axes current_limits
%         catch; end
        xlabel('time [s]')
%% figure(74); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(74); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
%     try plot(data.ugvController.cmd.time, data.ugvController.cmd.linearRate, '.', 'displayname', 'ugv cmd linear rate'); catch; end
    try plot(data.ugvController.cmd.time, data.ugvController.cmd.angularRate, '.', 'displayname', 'ugv cmd angular rate'); catch; end
    try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.twist.angular_xyz(:,3), '.', 'displayname', 'ugv cmd angular rate'); catch; end
hold off
%% disp('RGB pixels and marker locations')
disp('RGB pixels and marker locations')
%% figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['top down agent locations '  [meta.date meta.run]])    
    hold on
    try h1 = plot(0,0, 'k*', 'displayname', 'vicon-origin'); catch; end
    try h2 = plot(data.vicon.ugvk.P.vicon(:,1), data.vicon.ugvk.P.vicon(:,2), 'ms', 'displayname', 'ugv-vicon'); catch; end
    try h3 = plot(data.vicon.red_.P.vicon(:,1), data.vicon.red_.P.vicon(:,2), 'ro', 'displayname', 'red-vicon-xy'); catch; end
    try h4 = plot(data.vicon.blue_.P.vicon(:,1), data.vicon.blue_.P.vicon(:,2), 'bo', 'displayname', 'red-vicon-xy'); catch; end
    try h5 = plot(data.vicon.green_.P.vicon(:,1), data.vicon.green_.P.vicon(:,2), 'go', 'displayname', 'red-vicon-xy'); catch; end
    
    try % ugv initial pointing vector
        for i = 1:10
%         for i = 1: size(data.vicon.ugvk.P.x,1)
            x1 = data.vicon.ugvk.P.vicon(i,1);
            x2 = x1 + 0.1* data.vicon.ugvk.R_vic2lo(1,1,i);
            y1 = vicon.ugvk.P.vicon(i,2);
            y2 = y1 + 0.1* data.vicon.ugvk.R_vic2lo(1,2,i);
            line([x1 x2], [y1 y2]);
        end
        clear x1 x2 y1 y2 i
    catch
    end
    try % ugv final pointing vector
        for i = 1:10
%         for i = 1: size(data.vicon.ugvk.P.x,1)
            x1 = data.vicon.ugvk.P.vicon(end-i,1);
            x2 = x1 + 0.1* data.vicon.ugvk.R_vic2lo(1,1,end-i);
            y1 = data.vicon.ugvk.P.vicon(end-i,2);
            y2 = y1 + 0.1* data.vicon.ugvk.R_vic2lo(1,2,end-i);
            line([x1 x2], [y1 y2]);
        end
        clear x1 x2 y1 y2 i
    catch
    end
    % April tags
try
    for i = 1:length(aprilOrd)
        try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.vicon(:,1), ...
                 data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.vicon(:,2), ...
            ['k' linespec{i}], 'displayname', ['april' aprilOrd{i}]); catch; end
    end
catch
end

    hold off
    grid on
    try
        legend('toggle')
        legend([h1, h2, h3, h4, h5], 'Location', 'northwest')
        clear h1 h2 h3 h4 h5
    catch
    end
    
%     axis([-.5 4.5 -2.5 2.5])
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end
%% figure(101); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(101); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,2,1)
    hold on
    try  plot(data.ugvStereo.Red.left.xy(:,1), data.ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(data.ugvStereo.Blue.left.xy(:,1), data.ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(data.ugvStereo.Green.left.xy(:,1), data.ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    try addArrow([data.ugvStereo.Red.LEDs(bp1,1) ugvStereo.Red.LEDs(bp1,2)], [data.ugvStereo.Red.LEDs(bp2,1) ugvStereo.Red.LEDs(bp2,2)],'LineWidth',3, 'color', 'k', 'HeadWidth', 20); catch; end
    try addArrow([data.ugvStereo.Red.LEDs(bp1,1) ugvStereo.Red.LEDs(bp1,2)], [data.ugvStereo.Red.LEDs(bp2,1) ugvStereo.Red.LEDs(bp2,2)],'LineWidth',3, 'color', 'k', 'HeadWidth', 20);catch; end
    try addArrow([data.ugvStereo.Red.LEDs(bp1,1) ugvStereo.Red.LEDs(bp1,2)], [data.ugvStereo.Red.LEDs(bp2,1) ugvStereo.Red.LEDs(bp2,2)],'LineWidth',3, 'color', 'k', 'HeadWidth', 20);catch; end
    hold off
    grid on

    subplot(2,2,2)
    hold on
    try  plot(data.ugvStereo.Red.right.xy(:,1), data.ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(data.ugvStereo.Blue.right.xy(:,1), data.ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(data.ugvStereo.Green.right.xy(:,1), data.ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    subplot(2,2,3)
    hold on
    try  plot(data.ugvStereo.Red.left.xy(:,1), data.ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(data.ugvStereo.Blue.left.xy(:,1), data.ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(data.ugvStereo.Green.left.xy(:,1), data.ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    axis([-320 320 -240 240])
    subplot(2,2,4)
    hold on
    try  plot(data.ugvStereo.Red.right.xy(:,1), data.ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-left'); catch; end
    try  plot(data.ugvStereo.Blue.right.xy(:,1), data.ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-left'); catch; end
    try  plot(data.ugvStereo.Green.right.xy(:,1), data.ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-left'); catch; end
    hold off
    grid on
    axis([-320 320 -240 240])
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end
%% figure(102); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(102); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,1,1)
    hold on
    try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,1), 'ro', 'displayname', 'red-pixels-leftx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,1), 'bo', 'displayname', 'blue-pixels-leftx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,1), 'go', 'displayname', 'green-pixels-leftx'); catch; end
    
    try plot(data.ugvStereo.time, data.vicon.red_.splines.left.xy(:,1), 'rx', 'displayname', 'red-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.blue_.splines.left.xy(:,1), 'bx', 'displayname', 'blue-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.green_.splines.left.xy(:,1), 'gx', 'displayname', 'green-pixels-vicon'); catch; end
    legend('toggle')
    
%     disp(['mean error between x pixels [left rbg] :'])
%     disp([...
%         mean(data.vicon.red_.splines.left.xy(:,1) - data.ugvStereo.Red.left.xy(:,1)) ...
%         mean(data.vicon.blue_.splines.left.xy(:,1) - data.ugvStereo.Blue.left.xy(:,1)) ...
%         mean(data.vicon.green_.splines.left.xy(:,1) - data.ugvStereo.Green.left.xy(:,1)) ])
%     disp(['mean error between y pixels [left rbg] :'])
    
    hold off
    grid on
    ylabel('left x pixels [px]')
    subplot(2,1,2)
    hold on
    try  plot(data.ugvStereo.time, data.ugvStereo.Red.right.xy(:,1), 'ro', 'displayname', 'red-pixels-rightx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Blue.right.xy(:,1), 'bo', 'displayname', 'blue-pixels-rightx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Green.right.xy(:,1), 'go', 'displayname', 'green-pixels-rightx'); catch; end
    try plot(data.ugvStereo.time, data.vicon.red_.splines.right.xy(:,1), 'rx', 'displayname', 'red-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.blue_.splines.right.xy(:,1), 'bx', 'displayname', 'blue-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.green_.splines.right.xy(:,1), 'gx', 'displayname', 'green-pixels-vicon'); catch; end

    hold off
    grid on
    ylabel('right x pixels [px]')

%     disp(['mean error between x pixels [right rbg] :'])
%     disp([...
%         mean(data.vicon.red_.splines.right.xy(:,1) - data.ugvStereo.Red.right.xy(:,1)) ...
%         mean(data.vicon.blue_.splines.right.xy(:,1) - data.ugvStereo.Blue.right.xy(:,1)) ...
%         mean(data.vicon.green_.splines.right.xy(:,1) - data.ugvStereo.Green.right.xy(:,1)) ])
% 
%     disp([...
%         mean(data.vicon.red_.splines.left.xy(:,2) - data.ugvStereo.Red.left.xy(:,2)) ...
%         mean(data.vicon.blue_.splines.left.xy(:,2) - data.ugvStereo.Blue.left.xy(:,2)) ...
%         mean(data.vicon.green_.splines.left.xy(:,2) - data.ugvStereo.Green.left.xy(:,2)) ])
%     disp([...
%         mean(data.vicon.red_.splines.right.xy(:,2) - data.ugvStereo.Red.right.xy(:,2)) ...
%         mean(data.vicon.blue_.splines.right.xy(:,2) - data.ugvStereo.Blue.right.xy(:,2)) ...
%         mean(data.vicon.green_.splines.right.xy(:,2) - data.ugvStereo.Green.right.xy(:,2)) ])
%% figure(103); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(103); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,1,1)
    hold on
    try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-leftx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-leftx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-leftx'); catch; end
    
    try plot(data.ugvStereo.time, data.vicon.red_.splines.left.xy(:,2), 'rx', 'displayname', 'red-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.blue_.splines.left.xy(:,2), 'bx', 'displayname', 'blue-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.green_.splines.left.xy(:,2), 'gx', 'displayname', 'green-pixels-vicon'); catch; end
    legend('toggle')
    
%     disp(['mean error between x pixels [left rbg] :'])
%     disp([...
%         mean(data.vicon.red_.splines.left.xy(:,1) - data.ugvStereo.Red.left.xy(:,1)) ...
%         mean(data.vicon.blue_.splines.left.xy(:,1) - data.ugvStereo.Blue.left.xy(:,1)) ...
%         mean(data.vicon.green_.splines.left.xy(:,1) - data.ugvStereo.Green.left.xy(:,1)) ])
%     disp(['mean error between y pixels [left rbg] :'])
    
    hold off
    grid on
    ylabel('left y pixels [px]')
    subplot(2,1,2)
    hold on
    try  plot(data.ugvStereo.time, data.ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-rightx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-rightx'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-rightx'); catch; end
    try plot(data.ugvStereo.time, data.vicon.red_.splines.right.xy(:,2), 'rx', 'displayname', 'red-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.blue_.splines.right.xy(:,2), 'bx', 'displayname', 'blue-pixels-vicon'); catch; end
    try plot(data.ugvStereo.time, data.vicon.green_.splines.right.xy(:,2), 'gx', 'displayname', 'green-pixels-vicon'); catch; end

    hold off
    grid on
    ylabel('right y pixels [px]')

%     disp(['mean error between x pixels [right rbg] :'])
%     disp([...
%         mean(data.vicon.red_.splines.right.xy(:,1) - data.ugvStereo.Red.right.xy(:,1)) ...
%         mean(data.vicon.blue_.splines.right.xy(:,1) - data.ugvStereo.Blue.right.xy(:,1)) ...
%         mean(data.vicon.green_.splines.right.xy(:,1) - data.ugvStereo.Green.right.xy(:,1)) ])
% 
%     disp([...
%         mean(data.vicon.red_.splines.left.xy(:,2) - data.ugvStereo.Red.left.xy(:,2)) ...
%         mean(data.vicon.blue_.splines.left.xy(:,2) - data.ugvStereo.Blue.left.xy(:,2)) ...
%         mean(data.vicon.green_.splines.left.xy(:,2) - data.ugvStereo.Green.left.xy(:,2)) ])
%     disp([...
%         mean(data.vicon.red_.splines.right.xy(:,2) - data.ugvStereo.Red.right.xy(:,2)) ...
%         mean(data.vicon.blue_.splines.right.xy(:,2) - data.ugvStereo.Blue.right.xy(:,2)) ...
%         mean(data.vicon.green_.splines.right.xy(:,2) - data.ugvStereo.Green.right.xy(:,2)) ])
%% figure(104); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(104); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on
            try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,1)-data.vicon.red_.splines.left.xy(:,1), 'ro', 'displayname', 'red-pixels-leftx'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,1)-data.vicon.blue_.splines.left.xy(:,1), 'bo', 'displayname', 'blue-pixels-leftx'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,1)-data.vicon.green_.splines.left.xy(:,1), 'go', 'displayname', 'green-pixels-leftx'); catch; end    
        legend('toggle')
        hold off; grid on
        ylabel('left x pixels [px]')
        title('difference between actual pixels and vicon mapped pixels')
    subplot(2,1,2)
        hold on
            try  plot(data.ugvStereo.time, data.ugvStereo.Red.right.xy(:,1)-data.vicon.red_.splines.right.xy(:,1), 'ro', 'displayname', 'red-pixels-rightx'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Blue.right.xy(:,1)-data.vicon.blue_.splines.right.xy(:,1), 'bo', 'displayname', 'blue-pixels-rightx'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Green.right.xy(:,1)-data.vicon.green_.splines.right.xy(:,1), 'go', 'displayname', 'green-pixels-rightx'); catch; end
        hold off
        grid on
        ylabel('right x pixels [px]')
%% figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on
            try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,2)-data.vicon.red_.splines.left.xy(:,2), 'ro', 'displayname', 'red-pixels-lefty'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,2)-data.vicon.blue_.splines.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-lefty'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,2)-data.vicon.green_.splines.left.xy(:,2), 'go', 'displayname', 'green-pixels-lefty'); catch; end    
        legend('toggle')
        hold off; grid on
        ylabel('left y pixels [px]')
        title('difference between actual pixels and vicon mapped pixels')
    subplot(2,1,2)
        hold on
            try  plot(data.ugvStereo.time, data.ugvStereo.Red.right.xy(:,2)-data.vicon.red_.splines.right.xy(:,2), 'ro', 'displayname', 'red-pixels-righty'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Blue.right.xy(:,2)-data.vicon.blue_.splines.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-righty'); catch; end
            try  plot(data.ugvStereo.time, data.ugvStereo.Green.right.xy(:,2)-data.vicon.green_.splines.right.xy(:,2), 'go', 'displayname', 'green-pixels-righty'); catch; end
        hold off
        grid on
        ylabel('right y pixels [px]')
%% figure(106); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(106); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,2,1)
        hold on
            try  plot(data.ugvStereo.Red.left.rawxy(:,1), data.ugvStereo.Red.left.rawxy(:,2), 'ro', 'displayname', 'raw'); catch; end
            try  plot(data.ugvStereo.Blue.left.rawxy(:,1), data.ugvStereo.Blue.left.rawxy(:,2), 'bo', 'displayname', 'raw'); catch; end
            try  plot(data.ugvStereo.Green.left.rawxy(:,1), data.ugvStereo.Green.left.rawxy(:,2), 'go', 'displayname', 'raw'); catch; end
            axis([0 data.ugvStereo.pgryaml.image_width 0 data.ugvStereo.pgryaml.image_height])
            set(gca, 'ydir', 'reverse')
        hold off
        grid on
        legend('toggle');legend('Location', 'SouthEast')
        try 
            current_limits = axis; current_axes = gca; 
            current_axes.XTick = [current_limits(1):current_limits(2)/4:current_limits(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [current_limits(3):current_limits(4)/4:current_limits(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits limit_step
        end

    subplot(2,2,2)
        hold on
            try  plot(data.ugvStereo.Red.right.rawxy(:,1), data.ugvStereo.Red.right.rawxy(:,2), 'ro', 'displayname', 'raw'); catch; end
            try  plot(data.ugvStereo.Blue.right.rawxy(:,1), data.ugvStereo.Blue.right.rawxy(:,2), 'bo', 'displayname', 'raw'); catch; end
            try  plot(data.ugvStereo.Green.right.rawxy(:,1), data.ugvStereo.Green.right.rawxy(:,2), 'go', 'displayname', 'raw'); catch; end
            axis([0 data.ugvStereo.pgryaml.image_width 0 data.ugvStereo.pgryaml.image_height])
            set(gca, 'ydir', 'reverse')
        hold off
        grid on
        legend('toggle');legend('Location', 'SouthEast')
        try 
            current_limits = axis; current_axes = gca; 
            current_axes.XTick = [current_limits(1):current_limits(2)/4:current_limits(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [current_limits(3):current_limits(4)/4:current_limits(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits limit_step
        end

    subplot(2,2,3)
        hold on
            try  plot(data.ugvStereo.Red.left.xy(:,1), data.ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'cx-x'); catch; end
            try  plot(data.ugvStereo.Blue.left.xy(:,1), data.ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'cx-x'); catch; end
            try  plot(data.ugvStereo.Green.left.xy(:,1), data.ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'cx-x'); catch; end
            axis([-data.ugvStereo.pgryaml.image_width/2 data.ugvStereo.pgryaml.image_width/2 -data.ugvStereo.pgryaml.image_height/2 data.ugvStereo.pgryaml.image_height/2])
            set(gca, 'xdir', 'reverse')
        hold off
        grid on
        legend('toggle');legend('Location', 'SouthEast')
        try 
            current_limits = axis; current_axes = gca; 
            current_axes.XTick = [current_limits(1):current_limits(2)/2:current_limits(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [current_limits(3):current_limits(4)/2:current_limits(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits limit_step
        end

    subplot(2,2,4)
        hold on
            try  plot(data.ugvStereo.Red.right.xy(:,1), data.ugvStereo.Red.right.xy(:,2), 'ro', 'displayname', 'red-pixels-right'); catch; end
            try  plot(data.ugvStereo.Blue.right.xy(:,1), data.ugvStereo.Blue.right.xy(:,2), 'bo', 'displayname', 'blue-pixels-right'); catch; end
            try  plot(data.ugvStereo.Green.right.xy(:,1), data.ugvStereo.Green.right.xy(:,2), 'go', 'displayname', 'green-pixels-right'); catch; end
            axis([-data.ugvStereo.pgryaml.image_width/2 data.ugvStereo.pgryaml.image_width/2 -data.ugvStereo.pgryaml.image_height/2 data.ugvStereo.pgryaml.image_height/2])
            set(gca, 'xdir', 'reverse')
        hold off
        grid on
        legend('toggle');legend('Location', 'SouthEast')
        try 
            current_limits = axis; current_axes = gca; 
            current_axes.XTick = [current_limits(1):current_limits(2)/2:current_limits(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [current_limits(3):current_limits(4)/2:current_limits(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits limit_step
        end
%% figure(107); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(107); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['top down agent locations, cam frame, '  [meta.date meta.run]])    
    hold on
        try h1 = plot(data.vicon.red_.P.cam(:,1), data.vicon.red_.P.cam(:,3), 'r.', 'displayname', 'red-vicon-xy'); catch; end
        try h2 = plot(data.vicon.blue_.P.cam(:,1), data.vicon.blue_.P.cam(:,3), 'b.', 'displayname', 'blue-vicon-xy'); catch; end
        try h3 = plot(data.vicon.green_.P.cam(:,1), data.vicon.green_.P.cam(:,3), 'g.', 'displayname', 'green-vicon-xy'); catch; end
 
        try h4 = plot(data.ugvStereo.Red.P_cam(:,1), data.ugvStereo.Red.P_cam(:,3), 'ro', 'displayname', 'red-stereo-xy'); catch; end
        try h5 = plot(data.ugvStereo.Blue.P_cam(:,1), data.ugvStereo.Blue.P_cam(:,3), 'bo', 'displayname', 'blue-stereo-xy'); catch; end
        try h6 = plot(data.ugvStereo.Green.P_cam(:,1), data.ugvStereo.Green.P_cam(:,3), 'go', 'displayname', 'green-stereo-xy'); catch; end
        
        
    hold off
%% figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['top down agent locations, ugv frame, '  [meta.date meta.run]])    
    hold on
        try h1 = plot(data.vicon.red_.P.ugv(:,1), data.vicon.red_.P.ugv(:,2), 'r.', 'displayname', 'red-vicon-xy'); catch; end
        try h2 = plot(data.vicon.blue_.P.ugv(:,1), data.vicon.blue_.P.ugv(:,2), 'b.', 'displayname', 'blue-vicon-xy'); catch; end
        try h3 = plot(data.vicon.green_.P.ugv(:,1), data.vicon.green_.P.ugv(:,2), 'g.', 'displayname', 'green-vicon-xy'); catch; end
 
        try h4 = plot(data.ugvStereo.Red.P_ugv(:,1), data.ugvStereo.Red.P_ugv(:,2), 'ro', 'displayname', 'red-stereo-xy'); catch; end
        try h5 = plot(data.ugvStereo.Blue.P_ugv(:,1), data.ugvStereo.Blue.P_ugv(:,2), 'bo', 'displayname', 'blue-stereo-xy'); catch; end
        try h6 = plot(data.ugvStereo.Green.P_ugv(:,1), data.ugvStereo.Green.P_ugv(:,2), 'go', 'displayname', 'green-stereo-xy'); catch; end
        
        
    hold off
%% figure(109); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(109); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,1,1)
    hold on
        try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,1) - data.ugvStereo.Red.right.xy(:,1), 'ro', 'displayname', 'red-disp-x'); catch; end
        try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,1) - data.ugvStereo.Blue.right.xy(:,1), 'bo', 'displayname', 'blue-disp-x'); catch; end
        try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,1) - data.ugvStereo.Green.right.xy(:,1), 'go', 'displayname', 'green-disp-x'); catch; end
    hold off
    legend('toggle')
    grid on
    title('disparity [px]')

    subplot(2,1,2)
    hold on
        try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-y'); catch; end
        try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-y'); catch; end
        try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-y'); catch; end
    hold off
    legend('toggle')
    grid on
%% figure(110); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(110); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['normed lengths of marker difference vectors' [meta.date meta.run]])
    subplot(2,1,1)
        hold on
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_BmR, 'o', 'displayname', 'norm-BmR'); catch; end%UAV location, x
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_RmG, 'o', 'displayname', 'norm-RmG'); catch; end%UAV location, x
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_GmB, 'o', 'displayname', 'norm-GmB'); catch; end%UAV location, x
        hold off
        grid on
        legend('toggle')
    subplot(2,1,2)
        hold on
            try plot(data.ugvStereo.time, data.ugvStereo.uav.normdiff_BmR, 'o', 'displayname', 'norm-BmR'); catch; end%UAV location, x
            try plot(data.ugvStereo.time, data.ugvStereo.uav.normdiff_RmG, 'o', 'displayname', 'norm-RmG'); catch; end%UAV location, x
            try plot(data.ugvStereo.time, data.ugvStereo.uav.normdiff_GmB, 'o', 'displayname', 'norm-GmB'); catch; end%UAV location, x
        hold off
        grid on
        legend('toggle')
%% figure(111); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(111); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker position in camera frame X ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.red_.time, data.vicon.red_.P.cam(:,1), 'r.', 'displayname', 'red marker vicon'); catch; end
        try plot(data.vicon.blue_.time, data.vicon.blue_.P.cam(:,1), 'b.', 'displayname', 'blue marker vicon'); catch; end
        try plot(data.vicon.green_.time, data.vicon.green_.P.cam(:,1), 'g.', 'displayname', 'green marker vicon'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Red.P_cam(:,1), 'ro', 'displayname', 'red marker pgr'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_cam(:,1), 'bo', 'displayname', 'red marker pgr'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Green.P_cam(:,1), 'go', 'displayname', 'red marker pgr'); catch; end

    hold off
    grid on
    legend('toggle')
%% figure(112); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(112); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker position in camera frame Y ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.red_.time, data.vicon.red_.P.cam(:,2), 'r.', 'displayname', 'red marker vicon'); catch; end
        try plot(data.vicon.blue_.time, data.vicon.blue_.P.cam(:,2), 'b.', 'displayname', 'blue marker vicon'); catch; end
        try plot(data.vicon.green_.time, data.vicon.green_.P.cam(:,2), 'g.', 'displayname', 'green marker vicon'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Red.P_cam(:,2), 'ro', 'displayname', 'red marker pgr'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_cam(:,2), 'bo', 'displayname', 'red marker pgr'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Green.P_cam(:,2), 'go', 'displayname', 'red marker pgr'); catch; end

    hold off
    grid on
    legend('toggle')
%% figure(113); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(113); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker position in camera frame Z ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.red_.time, data.vicon.red_.P.cam(:,3), 'r.', 'displayname', 'red marker vicon'); catch; end
        try plot(data.vicon.blue_.time, data.vicon.blue_.P.cam(:,3), 'b.', 'displayname', 'blue marker vicon'); catch; end
        try plot(data.vicon.green_.time, data.vicon.green_.P.cam(:,3), 'g.', 'displayname', 'green marker vicon'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Red.P_cam(:,3), 'ro', 'displayname', 'red marker pgr'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_cam(:,3), 'bo', 'displayname', 'red marker pgr'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Green.P_cam(:,3), 'go', 'displayname', 'red marker pgr'); catch; end

    hold off
    grid on
    legend('toggle')
%% figure(114); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(114); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try 
            [redspline, redvector_newtime, data.vicon.red_.splines.err_ugv(:,1)] = spliner(data.vicon.red_.time, data.vicon.red_.P.ugv(:,1), data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,1));
            plot(data.ugvStereo.time, data.vicon.red_.splines.err_ugv(:,1), 'r.', 'displayname', 'red x error ugv'); 
        catch; end
        try 
            [redspline, redvector_newtime, data.vicon.blue_.splines.err_ugv(:,1)] = spliner(data.vicon.blue_.time, data.vicon.blue_.P.ugv(:,1), data.ugvStereo.time, data.ugvStereo.Blue.P_ugv(:,1));
            plot(data.ugvStereo.time, data.vicon.blue_.splines.err_ugv(:,1), 'b.', 'displayname', 'blue x error ugv'); 
        catch; end
        try 
            [redspline, redvector_newtime, data.vicon.green_.splines.err_ugv(:,1)] = spliner(data.vicon.green_.time, data.vicon.green_.P.ugv(:,1), data.ugvStereo.time, data.ugvStereo.Green.P_ugv(:,1));
            plot(data.ugvStereo.time, data.vicon.green_.splines.err_ugv(:,1), 'g.', 'displayname', 'green x error ugv'); 
        catch; end
    hold off
    grid on; legend('toggle'), xlabel('time [s]'); ylabel('x [m]')
    title('x error of UAV marker locations')
    try %current axis
        current_limits = axis; current_axes = gca; 
        current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
        current_axes.YLim = [-0.1 0.1];
        current_axes.YTick = [current_axes.YLim(1):0.025:current_axes.YLim(2)];
        clear current_axes current_limits
    end
%% figure(115); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(115); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try 
            [redspline, redvector_newtime, data.vicon.red_.splines.err_ugv(:,2)] = spliner(data.vicon.red_.time, data.vicon.red_.P.ugv(:,2), data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,2));
            plot(data.ugvStereo.time, data.vicon.red_.splines.err_ugv(:,2), 'r.', 'displayname', 'red x error ugv'); 
        catch; end
        try 
            [redspline, redvector_newtime, data.vicon.blue_.splines.err_ugv(:,2)] = spliner(data.vicon.blue_.time, data.vicon.blue_.P.ugv(:,2), data.ugvStereo.time, data.ugvStereo.Blue.P_ugv(:,2));
            plot(data.ugvStereo.time, data.vicon.blue_.splines.err_ugv(:,2), 'b.', 'displayname', 'blue x error ugv'); 
        catch; end
        try 
            [redspline, redvector_newtime, data.vicon.green_.splines.err_ugv(:,2)] = spliner(data.vicon.green_.time, data.vicon.green_.P.ugv(:,2), data.ugvStereo.time, data.ugvStereo.Green.P_ugv(:,2));
            plot(data.ugvStereo.time, data.vicon.green_.splines.err_ugv(:,2), 'g.', 'displayname', 'green x error ugv'); 
        catch; end
    hold off
    grid on; legend('toggle'), xlabel('time [s]'); ylabel('y [m]')
    title('y error of UAV marker locations')
    try %current axis
        current_limits = axis; current_axes = gca; 
        current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
        current_axes.YLim = [-0.1 0.1];
        current_axes.YTick = [current_axes.YLim(1):0.025:current_axes.YLim(2)];
        clear current_axes current_limits
    end
%% figure(116); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(116); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try 
            [redspline, redvector_newtime, data.vicon.red_.splines.err_ugv(:,3)] = spliner(data.vicon.red_.time, data.vicon.red_.P.ugv(:,3), data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,3));
            plot(data.ugvStereo.time, data.vicon.red_.splines.err_ugv(:,3), 'r.', 'displayname', 'red x error ugv'); 
        catch; end
        try 
            [redspline, redvector_newtime, data.vicon.blue_.splines.err_ugv(:,3)] = spliner(data.vicon.blue_.time, data.vicon.blue_.P.ugv(:,3), data.ugvStereo.time, data.ugvStereo.Blue.P_ugv(:,3));
            plot(data.ugvStereo.time, data.vicon.blue_.splines.err_ugv(:,3), 'b.', 'displayname', 'blue x error ugv'); 
        catch; end
        try 
            [redspline, redvector_newtime, data.vicon.green_.splines.err_ugv(:,3)] = spliner(data.vicon.green_.time, data.vicon.green_.P.ugv(:,3), data.ugvStereo.time, data.ugvStereo.Green.P_ugv(:,3));
            plot(data.ugvStereo.time, data.vicon.green_.splines.err_ugv(:,3), 'g.', 'displayname', 'green x error ugv'); 
        catch; end
    hold off
    title('z error of UAV marker locations')
    grid on; legend('toggle'), xlabel('time [s]'); ylabel('z [m]')
    try %current axis
        current_limits = axis; current_axes = gca; 
        current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
        current_axes.YLim = [-0.1 0.1];
        current_axes.YTick = [current_axes.YLim(1):0.025:current_axes.YLim(2)];
        clear current_axes current_limits
    end
%% figure(117); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(117); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.ugvStereo.time, data.vicon.red_.splines.err_ugv_rms, 'r.', 'displayname', 'red abs error'); catch; end
        try plot(data.ugvStereo.time, data.vicon.blue_.splines.err_ugv_rms, 'b.', 'displayname', 'blue abs error'); catch; end
        try plot(data.ugvStereo.time, data.vicon.green_.splines.err_ugv_rms, 'g.', 'displayname', 'green abs error'); catch; end
    hold off
    grid on; legend('toggle'), xlabel('time [s]'); ylabel('abs error [m]')
    title('abs error of UAV marker locations')
    try %current axis
        current_limits = axis; current_axes = gca; 
        current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
        current_axes.YLim = [-0.5 0.5];
        current_axes.YTick = [current_axes.YLim(1):0.1:current_axes.YLim(2)];
        clear current_axes current_limits
    end
%% figure(118); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(118); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.red_.splines.P_cam(:,3), data.vicon.red_.splines.err_ugv_rms, 'r.', 'displayname', 'red abs error'); catch; end
        try plot(data.vicon.red_.splines.P_cam(:,3), data.vicon.blue_.splines.err_ugv_rms, 'b.', 'displayname', 'blue abs error'); catch; end
        try plot(data.vicon.red_.splines.P_cam(:,3), data.vicon.green_.splines.err_ugv_rms, 'g.', 'displayname', 'green abs error'); catch; end
    hold off
    grid on; legend('toggle'), xlabel('range along optical axis'); ylabel('abs error [m]')
    title(['abs error of UAV marker locations as a function of range, ' [meta.date meta.run]])
    try %current axis
        current_limits = axis; current_axes = gca; 
%         current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
        current_axes.YLim = [-0.5 0.5];
        current_axes.YTick = [current_axes.YLim(1):0.1:current_axes.YLim(2)];
        clear current_axes current_limits
    end
%% figure(119); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(119); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.red_.splines.P_cam(:,3), data.vicon.red_.splines.err_ugv_rms./(data.vicon.red_.splines.P_cam(:,3).^2), 'r.', 'displayname', 'red abs error'); catch; end
        try plot(data.vicon.red_.splines.P_cam(:,3), data.vicon.blue_.splines.err_ugv_rms./(data.vicon.red_.splines.P_cam(:,3).^2), 'b.', 'displayname', 'blue abs error'); catch; end
        try plot(data.vicon.red_.splines.P_cam(:,3), data.vicon.green_.splines.err_ugv_rms./(data.vicon.red_.splines.P_cam(:,3).^2), 'g.', 'displayname', 'green abs error'); catch; end
    hold off
    grid on; legend('toggle'), xlabel('range along optical axis'); ylabel('abs error [m]')
    title(['abs error of UAV marker locations as a function of range, scaled by the square of the range' [meta.date meta.run]])
    try %current axis
        current_limits = axis; current_axes = gca; 
%         current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
        current_axes.YLim = [-0.5 0.5];
        current_axes.YTick = [current_axes.YLim(1):0.1:current_axes.YLim(2)];
        clear current_axes current_limits
    end
%% figure(120); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(120); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.red_.time, data.vicon.red_.P.ugv(:,1), 'r.', 'displayname', 'vicon px ugv'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,1), 'ro', 'displayname', 'stereo px ugv'); catch; end
        try plot(data.ugvStereo.time, test.Red.P_ugv(:,1), 'rs', 'displayname', 'stereo px ugv'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')
%% figure(121); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(121); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.red_.time, data.vicon.red_.P.ugv(:,2), 'r.', 'displayname', 'vicon py ugv'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,2), 'ro', 'displayname', 'stereo py ugv'); catch; end
        try plot(data.ugvStereo.time, test.Red.P_ugv(:,2), 'rs', 'displayname', 'stereo py ugv'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')
%% figure(122); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(122); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.vicon.red_.time, data.vicon.red_.P.ugv(:,3), 'r.', 'displayname', 'vicon pz ugv'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,3), 'ro', 'displayname', 'stereo pz ugv'); catch; end
        try plot(data.ugvStereo.time, test.Red.P_ugv(:,3), 'rs', 'displayname', 'stereo pz ugv'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')
%% figure(123); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(123); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    try plot(abs(data.vicon.diff_yaw.spline_error),sqrt(data.ugvRecorder.stereo.yaw_cov)*180/pi, 'o'); catch; end
    hold off
%% figure(124); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(124); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
try 
    for tag_i = 1:length(tagnames)
            try plot(data.april.(matlab.lang.makeValidName([tagnames{tag_i}])).MeasPosition_gl(:,1), ...
                     data.april.(matlab.lang.makeValidName([tagnames{tag_i}])).MeasPosition_gl(:,2), ...
                ['ks'], 'displayname', ['april' tagnames{tag_i}]); catch; end
    end
end
    
    try plot(data.vicon.uav.P.global(:,1), data.vicon.uav.P.global(:,2), '.', 'displayname', 'uav path'); catch; end
    try plot(data.vicon.ugvk.P.global(:,1), data.vicon.ugvk.P.global(:,2), '.', 'displayname', 'ugvk path'); catch; end
    try plot(data.experiment.uav.DesiredState_gl(:,1), data.experiment.uav.DesiredState_gl(:,2), '-x'); catch; end
    
    grid on
    hold off
%% figure(125); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(125); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title('UAV markers z-axis position')
    hold on
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,1), 'd', 'Color', [1,1,1], ...
                     'MarkerFaceColor', [0,0,0], 'displayname', 'uav x goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end

        try plot(data.vicon.red_.time, data.vicon.red_.P.cam(:,3), 'r-'); catch; end    
        try plot(data.vicon.blue_.time, data.vicon.blue_.P.cam(:,3), 'b-'); catch; end    
        try plot(data.vicon.green_.time, data.vicon.green_.P.cam(:,3), 'g-'); catch; end    
        try plot(data.ugvStereo.Red.solo.Time,   data.ugvStereo.Red.solo.P_cam(:,3), 'ro'); catch; end
        try plot(data.ugvStereo.Blue.solo.Time,  data.ugvStereo.Blue.solo.P_cam(:,3), 'bo'); catch; end
        try plot(data.ugvStereo.Green.solo.Time, data.ugvStereo.Green.solo.P_cam(:,3), 'go'); catch; end
    hold off
    grid on
    try
        axis([min([...
            data.vicon.red_.time(1)...
            data.vicon.blue_.time(1)...
            data.vicon.green_.time(1)...
            data.ugvStereo.Red.solo.Time(1)...
            data.ugvStereo.Blue.solo.Time(1)...
            data.ugvStereo.Green.solo.Time(1)...
                ]) max([...
            data.vicon.red_.time(end)...
            data.vicon.blue_.time(end)...
            data.vicon.green_.time(end)...
            data.ugvStereo.Red.solo.Time(end)...
            data.ugvStereo.Blue.solo.Time(end)...
            data.ugvStereo.Green.solo.Time(end)...
                ]) 0 8])
            xlabel('time [s]')
            ylabel('range [m]')
    catch
    end
        
        
try
    [~, data.ugvStereo.Red.solo.vicon_range, ~]  = spliner(...
        data.vicon.red_.time, data.vicon.red_.P.cam(:,3), ...
        data.ugvStereo.Red.solo.Time, data.ugvStereo.Red.solo.P_cam(:,3));
    
    [~, data.ugvStereo.Blue.solo.vicon_range, ~]  = spliner(...
        data.vicon.blue_.time, data.vicon.blue_.P.cam(:,3), ...
        data.ugvStereo.Blue.solo.Time, data.ugvStereo.Blue.solo.P_cam(:,3));
    
    [~, data.ugvStereo.Green.solo.vicon_range, ~]  = spliner(...
        data.vicon.green_.time, data.vicon.green_.P.cam(:,3), ...
        data.ugvStereo.Green.solo.Time, data.ugvStereo.Green.solo.P_cam(:,3));
catch
end
    hold on
%         try plot(data.ugvStereo.Red.solo.Time,   data.ugvStereo.Red.solo.vicon_range, 'ro', 'MarkerFaceColor', 'r'); catch; end
%         try plot(data.ugvStereo.Blue.solo.Time,   data.ugvStereo.Blue.solo.vicon_range, 'ro', 'MarkerFaceColor', 'r'); catch; end
        try plot(data.ugvStereo.Green.solo.Time,   data.ugvStereo.Green.solo.vicon_range, 'go', 'MarkerFaceColor', 'g'); catch; end
    hold off
    
        
        try fprintf('Max detected range of (R B G) markers: ( % -3.2f & % -3.2f & % -3.2f )\n', ...
            max(data.ugvStereo.Red.solo.vicon_range),...
            max(data.ugvStereo.Blue.solo.vicon_range),...
            max(data.ugvStereo.Green.solo.vicon_range));
        catch
        end
        
%% figure(150); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(150); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title(['uav yaw global frame ' [meta.date meta.run]])    
    ylabel('degrees')
    xlabel('range along optical axis, [m]')
    hold on
try 
    ordinate = mean(data.vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(data.vicon.uav.splines.P.cam.z_uavstereotime));
    ordinate2 = mean(data.vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(data.ugvRecorder.stereo.time));
    ordinate3 = mean(data.vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(data.ugvStereo.PostProc.uav.yaw_var));
 
        try 
            h1 = plot(mean(data.vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(data.ugvRecorder.stereo.time)), ...
                data.vicon.diff_yaw.spline_error, 'bo', 'displayname', 'stereo yaw error'); 
        catch; end
        try 
            h1 = plot(mean(data.vicon.uav.splines.P.cam.z_uavstereotime)*ones(size(data.ugvStereo.time)), ...
                data.vicon.diff_yaw.spline_error, 'bo', 'displayname', 'stereo yaw error'); 
        catch; end
%         try h2 = plot(0.025+ordinate, ...
%                  180*sqrt(data.ugvStereo.uav.UAV_yaw_var)/pi, 'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
%         try h3 = plot(0.025+ordinate, ...
%                 -180*sqrt(data.ugvStereo.uav.UAV_yaw_var)/pi, 'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
%         try 
        try h4 = plot(0.025+ordinate2, ...
                 180*sqrt(data.ugvRecorder.stereo.yaw_cov)/pi, 'rx', 'displayname', 'pixel propagated uncertainty2');  catch; end
%         catch; end
        try h5 = plot(0.025+ordinate2, ...
                -180*sqrt(data.ugvRecorder.stereo.yaw_cov)/pi, 'rx', 'displayname', 'pixel propagated uncertainty2'); catch; end
        try h6 = plot(-0.025+ordinate3,180*sqrt(data.ugvStereo.PostProc.uav.yaw_var)/pi, ...
               'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
        try h6 = plot(-0.025+ordinate3,-180*sqrt(data.ugvStereo.PostProc.uav.yaw_var)/pi, ...
               'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end

    hold off
    grid on
    clear ordinate ordinate2 ordinate3
%     axis([0 5 -135 135])
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'northwest')
        clear h1 h2 h3
    catch
    end
        
catch
end
%% figure(209); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(209); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try 
    for i = 1:length(aprilOrd)
        subplot(2,1,1)
            title(['Global measurent errors of fiducial tags, aggregated ' [meta.date meta.run]])
            hold on
                try h1 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.x_uav_diff, ...
                        'ro', 'displayname', ['x']); catch; end
                try h2 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.y_uav_diff, ...
                        'bo', 'displayname', ['y']); catch; end
                try h3 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.z_uav_diff, ...
                        'ko', 'displayname', ['z']); catch; end
            hold off
            grid on
            try
                legend('toggle')
                legend([h1, h2, h3], 'Location', 'southeast')
                clear h1 h2 h3 h4 h5
            catch
            end

            title(['UAV frame measurent errors of fiducial tags, aggregated ' [meta.date meta.run]])
            ylabel('linear difference [m]')
            try
                current_limits = axis;
                current_axes = gca; % current axes
    %             current_axes.XTick = [0 10 20 30 40 50 60];
                current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
                current_axes.YLim = [-0.5 0.5];
                current_axes.YTick = [current_axes.YLim(1):0.25:current_axes.YLim(2)];
                clear current_axes current_limits
            end

        subplot(2,1,2)
            hold on
                try h1 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).yaw.spline.yaw_uav_diff, ...
                        'ko',  'displayname', ['yaw']); catch; end            
            hold off
            grid on
            xlabel('time [s]')
            ylabel('angular difference [deg]')
            try
                legend('toggle')
                legend([h1], 'Location', 'southeast')
                clear h1 h2 h3 h4 h5
            catch
            end
            try
                current_limits = axis;
                current_axes = gca; % current axes
    %             current_axes.XTick = [0 10 20 30 40 50 60];
                current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
                current_axes.YLim = [-50 50];
                current_axes.YTick = [current_axes.YLim(1):25:current_axes.YLim(2)];

                clear current_axes current_limits
            end
    end
catch
end

    if meta.saveplots
        current_fig = gcf;
        meta.savefilename = ['FiducialErrors_uav_' meta.run];
        try     saveas(gcf, [meta.dataroot meta.date meta.savefilename '.png']); catch; end
%         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
        clear current_fig;
    end
%% figure(210); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(210); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    for i = 1:length(aprilOrd)
        subplot(2,1,1)
            hold on
                try h1 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.x_gl_diff, ...
                        'ro', 'displayname', ['x']); catch; end
                try h2 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.y_gl_diff, ...
                        'bo', 'displayname', ['y']); catch; end
                try h3 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.z_gl_diff, ...
                        'ko', 'displayname', ['z']); catch; end
            hold off
            grid on
            try
                legend('toggle')
                legend([h1, h2, h3], 'Location', 'southeast')
                clear h1 h2 h3 h4 h5
            catch
            end
            title(['Global frame measurent errors of fiducial tags, aggregated ' [meta.date meta.run]])
            ylabel('linear difference [m]')
            try
                current_limits = axis;
                current_axes = gca; % current axes
    %             current_axes.XTick = [0 10 20 30 40 50 60];
                current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
                current_axes.YLim = [-0.5 0.5];
                current_axes.YTick = [current_axes.YLim(1):0.25:current_axes.YLim(2)];
                clear current_axes current_limits
            end

        subplot(2,1,2)
            hold on
                try h1 = plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                        data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).yaw.spline.yaw_gl_diff, ...
                        'ko',  'displayname', ['yaw']); catch; end            
            hold off
            grid on
            xlabel('time [s]')
            ylabel('angular difference [deg]')
            try
                legend('toggle')
                legend([h1], 'Location', 'southeast')
                clear h1 h2 h3 h4 h5
            catch
            end
            try
                current_limits = axis;
                current_axes = gca; % current axes
    %             current_axes.XTick = [0 10 20 30 40 50 60];
                current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
                current_axes.YLim = [-50 50];
                current_axes.YTick = [current_axes.YLim(1):25:current_axes.YLim(2)];

                clear current_axes current_limits
            end
    end
catch
end
    if meta.saveplots
        current_fig = gcf;
        meta.savefilename = ['FiducialErrors_gl_' meta.run];
        try     saveas(gcf, [meta.dataroot meta.date meta.savefilename '.png']); catch; end
%         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
        clear current_fig;
    end
%% figure(22x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try 
    for i = 1:length(aprilOrd)
    figure(220+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['landmark measured and estimated position, global,  ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.global(:,1), ...
                 data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.global(:,2), ...
                 '.', 'MarkerSize', 13, 'displayname', ['vicon april' aprilOrd{i}]); catch; end
        try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_gl(:,1), ...
                 data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_gl(:,2), ...
                 'k.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i}]); catch; end
        try plot(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1), ...
                 data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,2), ...
                 's', 'displayname', ['slam april' aprilOrd{i}]); catch; end
        hold off
    grid on
    legend('toggle')
    xlabel('position [m]') 
    ylabel('position [m]') 
    end
catch
end
%% figure(23x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try 
    for i = 1:length(aprilOrd)
    figure(230+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        subplot(3,1,1); 
        title(['landmark measured and estimated position, global,  ' [meta.date meta.run]])
        hold on
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.global(:,1), ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' x']); catch; end
            try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1), ...
                    'rs', 'displayname', ['slam april' aprilOrd{i} ' x']); catch; end
            try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_gl(:,1), ...
                    'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' x']); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('position [m]') 
        subplot(3,1,2); 
        hold on
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.global(:,2), ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' y']); catch; end
            try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,2), ...
                    'rs', 'displayname', ['slam april' aprilOrd{i} ' y']); catch; end
            try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_gl(:,2), ...
                    'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' y']); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('position [m]') 
        subplot(3,1,3); 
        hold on
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.global(:,3), ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' z']); catch; end
            try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,3), ...
                    'rs', 'displayname', ['slam april' aprilOrd{i} ' z']); catch; end
            try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_gl(:,3), ...
                    'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' z']); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('position [m]') 
   
    xlabel('time [s]')
    end
catch
end
%% figure(24x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    for i = 1:length(aprilOrd)
    figure(240+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        subplot(3,1,1); 
        hold on
        title(['landmark measured and true position, uav frame,  ' [meta.date meta.run]])
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, ...
                    data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.uav(:,1), ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' x']); catch; end
            try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, ...
                    data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_uav(:,1), ...
                    'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' x']); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('position [m]') 
        subplot(3,1,2); 
        hold on
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.uav(:,2), ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' y']); catch; end
            try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_uav(:,2), ...
                    'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' y']); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('position [m]') 
        subplot(3,1,3); 
        hold on
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.uav(:,3), ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' z']); catch; end
            try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasPosition_uav(:,3), ...
                    'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' z']); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('position [m]') 
        xlabel('time [s]')    
%     disp([ 'april' aprilOrd{i} ' mean differences' ...
%         ', x: ' num2str(mean(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.x_uav_diff)) ...
%         ', y :' num2str(mean(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.y_uav_diff)) ...
%         ', z :' num2str(mean(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.z_uav_diff))])
    end
catch
end
%% figure(25x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    for i = 1:length(aprilOrd)
    figure(250+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        title(['landmark measured and estimated yaw, global frame,  ' [meta.date meta.run]])
        try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).yaw.global_deg, ...
                'k.', 'displayname', ['vicon april' aprilOrd{i} ' yaw']); catch; end
        try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.yaw.global(:,1), ...
                'rs', 'displayname', ['slam april' aprilOrd{i} ' yaw']); catch; end
        try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasYaw_gl, ...
                'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' yaw']); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('yaw [deg]') 
    xlabel('time [s]')
    end
catch
end 
%% figure(26x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    for i = 1:length(aprilOrd)
        figure(260+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
        title(['landmark measured and estimated position, uav frame,  ' [meta.date meta.run]])
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).time, data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).yaw.uav, ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' yaw']); catch; end
            try plot(data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).time, data.april.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).MeasYaw_uav, ...
                    'b.', 'MarkerSize', 13, 'displayname', ['meas april' aprilOrd{i} ' yaw']); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('yaw angle [deg]') 
        xlabel('time [s]')
    end
catch
end
%% figure(27x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    for i = 1:length(aprilOrd)
        figure(270+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title(['Global measurent errors of april tags,  ' [meta.date meta.run]])
        hold on
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                    data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.x_gl_diff, ...
                    'o', 'displayname', ['april' aprilOrd{i} ' meas x']); catch; end
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                    data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.y_gl_diff, ...
                    'o', 'displayname', ['april' aprilOrd{i} ' meas y']); catch; end
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.downtime, ...
                    data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).P.spline.z_gl_diff, ...
                    'o', 'displayname', ['april' aprilOrd{i} ' meas z']); catch; end
        hold off
        grid on
        legend('toggle')
        legend('Location', 'Northwest')
    end
catch
end
%% figure(300); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(300); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    hold on
    for i = 1:length(aprilOrd)
        try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.time(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
            data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.spline.x_diff(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
            ['k' linespec{i}], 'displayname', ['april' aprilOrd{i} ' x diff btw slam and vicon']); catch; end
    end
    try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.x_diff_stereotime, 'o', 'displayname', 'uav x diff btw stereo and vicon'); catch; end    
    try plot(data.vicon.uav.slam.time, data.vicon.uav.slam.spline.x_diff, 'rx', 'displayname', 'uav x diff btw slam and vicon'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')    
    try %current axis
        current_limits = axis; current_axes = gca; 
%         current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
        current_axes.YLim = [-0.75 0.75];
        current_axes.YTick = [current_axes.YLim(1):0.1:current_axes.YLim(2)];
        clear current_axes current_limits
    end
catch
end
%% figure(301); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(301); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    hold on
    for i = 1:length(aprilOrd)
        try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.time(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
            data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.spline.y_diff(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
            ['k' linespec{i}], 'displayname', ['april' aprilOrd{i} ' y diff btw slam and vicon']); catch; end
    end
    try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.y_diff_stereotime, 'bo', 'displayname', 'uav y diff btw stereo and vicon'); catch; end    
    try plot(data.vicon.uav.slam.time, data.vicon.uav.slam.spline.y_diff, 'rx', 'displayname', 'uav y diff btw slam and vicon'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')
catch
end
%% figure(302); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(302); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    try
        for i = 1:length(aprilOrd)
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.time(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
                data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.spline.z_diff(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
                ['k' linespec{i}], 'displayname', ['april' aprilOrd{i} ' z diff btw slam and vicon']); catch; end
        end
    catch
    end
    try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.z_diff_stereotime, 'bo', 'displayname', 'uav z diff btw stereo and vicon'); catch; end    
    try plot(data.vicon.uav.slam.time, data.vicon.uav.slam.spline.z_diff, 'rx', 'displayname', 'uav z diff btw slam and vicon'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')
%% figure(31x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    for i = 1:length(aprilOrd)
        figure(310+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title(['slam Rk and Ck,  ' [meta.date meta.run]])
        hold on
            try plot(data.slam.time, data.slam.uav.Ck_re(1,:), 's', 'displayname', 'uav Ck(1,1)'); catch; end
            try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).Ck_re(1,:), 'x', 'displayname', ['april' aprilOrd{i} ' Ck(1,1)']); catch; end
            try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).Rk_re(1,:), 'o', 'displayname', ['april' aprilOrd{i} ' Rk(1,1)']); catch; end
        hold off
        grid on
        legend('toggle')
        legend('Location', 'Northwest')
    end
catch
end
%% figure(37x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try 
    for i = 1:length(aprilOrd)
    figure(370+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    title(['global frame, tag error vs vicon location,  ' [meta.date meta.run]])
        try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.time(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
                data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.spline.x_diff(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,1)~=0), ...
                'k.', 'displayname', ['vicon april' aprilOrd{i} ' x diff']); catch; end
%         try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.x_diff_stereotime, 'bo', 'displayname', 'uav x diff btw stereo and vicon'); catch; end
%         try plot(data.vicon.uav.slam.time, data.vicon.uav.slam.spline.x_diff, 'rx', 'displayname', 'uav x diff btw slam and vicon'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')
    end
catch
end
%% figure(38x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try 
    for i = 1:length(aprilOrd)
    figure(380+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    title(['global frame, tag error vs vicon location,  ' [meta.date meta.run]])
        try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.time(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,2)~=0), ...
                data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.spline.y_diff(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,2)~=0), ...
                'k.', 'displayname', ['vicon april' aprilOrd{i} ' y diff']); catch; end
%         try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.y_diff_stereotime, 'o', 'displayname', 'uav y diff btw stereo and vicon'); catch; end    
%         try plot(data.vicon.uav.slam.time, data.vicon.uav.slam.spline.y_diff, 'rx', 'displayname', 'uav y diff btw slam and vicon'); catch; end
    hold off
    legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
    xlabel('time [s]')
    end
catch
end
%% figure(39x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
try
    for i = 1:length(aprilOrd)
        figure(390+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
        title(['global frame, tag error vs vicon location,  ' [meta.date meta.run]])
            try plot(data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.time(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,3)~=0), ...
                    data.vicon.(matlab.lang.makeValidName(['april' aprilOrd{i} '_'])).slam.spline.z_diff(data.slam.(matlab.lang.makeValidName(['tag_' aprilOrd{i}])).est.p.global(:,3)~=0), ...
                    'k.', 'displayname', ['vicon april' aprilOrd{i} ' z diff']); catch; end
    %         try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.z_diff_stereotime, 'o', 'displayname', 'uav z diff btw stereo and vicon'); catch; end    
    %         try plot(data.vicon.uav.slam.time, data.vicon.uav.slam.spline.z_diff, 'rx', 'displayname', 'uav z diff btw slam and vicon'); catch; end
        hold off
        legend('toggle'); legend('Location', 'NorthWest'); grid on; ylabel('error [m]') 
        xlabel('time [s]')
    end
catch
end
%% figure(401); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(401); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(data.slam.time, data.slam.uav.Ck_re(1,:), 's', 'displayname', 'uav Ck(1,1)'); catch; end
    for i = 0:7
        try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_0' num2str(i)])).Ck_re(1,:), ...
                'x', 'displayname', ['tag0' num2str(i) ' Ck(1,1)']); catch; end 
    end
    try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(1,:), 'o', 'displayname', 'uav CKF(1,1)'); catch; end
    
hold off
grid on
legend('toggle')
legend('Location', 'Northwest')
%% figure(402); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(402); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(data.slam.time, data.slam.uav.Ck_re(6,:), 's', 'displayname', 'uav Ck(2,2)'); catch; end
    for i = 0:7
        try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_0' num2str(i)])).Ck_re(6,:), ...
                'x', 'displayname', ['tag0' num2str(i) ' Ck(2,2)']); catch; end 
    end
    try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(6,:), 'o', 'displayname', 'uav CKF(2,2)'); catch; end
hold off
grid on
legend('toggle')
legend('Location', 'Northwest')
%% figure(403); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(403); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(data.slam.time, data.slam.uav.Ck_re(11,:), 's', 'displayname', 'uav Ck(3,3)'); catch; end
    for i = 0:7
        try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_0' num2str(i)])).Ck_re(11,:), ...
                'x', 'displayname', ['tag0' num2str(i) ' Ck(3,3)']); catch; end
    end
    try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(11,:), 'o', 'displayname', 'uav CKF(3,3)'); catch; end
hold off
grid on
legend('toggle')
legend('Location', 'Northwest')
%% figure(404); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(404); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(data.slam.time, data.slam.uav.Ck_re(16,:), 's', 'displayname', 'uav Ck(4,4)'); catch; end
    for i = 0:7
        try plot(data.slam.time, data.slam.(matlab.lang.makeValidName(['tag_0' num2str(i)])).Ck_re(16,:), ...
                'x', 'displayname', ['tag0' num2str(i) ' Ck(4,4)']); catch; end
    end
    try plot(data.ckfRecorder.time, data.ckfRecorder.Rk_re(16,:), 'o', 'displayname', 'uav CKF(4,4)'); catch; end
hold off
grid on
legend('toggle')
legend('Location', 'Northwest')
%% figure(501); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(501); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 3;
    
    subplot(2,1,1)
        title([meta.date meta.run])
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,1), 'd', 'Color', [1,1,1], ...
                     'MarkerFaceColor', [0,0,0], 'displayname', 'uav x goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'uav x actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'uav x estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
%             try h1 = plot(data.ugvStereo.time, data.caldata.uav.P_ugv(:,1), '--', 'Color', [0.25,0.25,0.25], ...
%                     'LineWidth', linewidth, 'displayname', 'uav x recal-stereo'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%             catch; end
        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('x [m]')
        grid on

        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 0 3];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
        end
       
    subplot(2,1,2)
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,2), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV y goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'UAV y actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
%             try h1 = plot(data.ugvStereo.time, data.caldata.uav.P_ugv(:,2), '--', 'Color', [0.25,0.25,0.25], ...
%                     'LineWidth', linewidth, 'displayname', 'uav y recal-stereo'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%             catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'UAV y estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('y [m]')
        grid on
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -1.5 1.5];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits
        end
        xlabel('time [s]')
%% figure(502); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(502); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
    title([meta.date meta.run])
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.DesiredState_gl(:,3), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV z goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
%             try plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,3), 'rd', 'displayname', 'UAV z goal'); catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,3), 'Color', [0.75,0.75,0.75], ...
                'LineWidth', linewidth, 'displayname', 'UAV z actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), '--', 'Color', [0.25,0.25,0.25], ...
                'LineWidth', linewidth, 'displayname','UAV z estimated');  ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
%             try h1 = plot(data.ugvStereo.time, data.caldata.uav.P_ugv(:,3), '--', 'Color', [0.25,0.25,0.25], ...
%                     'LineWidth', linewidth, 'displayname', 'uav z recal-stereo'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%             catch; end

        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('z [m]')
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 0.0 3.0];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits
        end
        grid on

    subplot(2,1,2)
        hold on; ii = 0; 
            if strcmp(meta.date, '20170211iros/')
                A = data.experiment.uav.Waypoint(:,4);
                A(A==0.8) = 0;
                data.experiment.uav.Waypoint(:,4) = A;
                clear A
                data.experiment.uav.Waypoint(:,3) = 0.8;
            end
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,4), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV yaw goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname','UAV yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname','UAV yaw estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
%             try h1 = plot(data.ugvStereo.time, data.caldata.yaw_ugv, '--', 'Color', [0.25,0.25,0.25], ...
%                     'LineWidth', linewidth, 'displayname', 'uav yaw recal-stereo'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%             catch; end
        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('heading [deg]')
        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -75 75];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):25:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits
        end
        grid on
        
        xlabel('time [s]')
%% figure(503); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure503 = figure(503); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig legend_str
    linewidth = 2;
    t_offset = 0; % -8 for 20170914_040
    subplot(2,1,1)
        hold on; ii = 0; % ]
        legend('AutoUpdate','off')
            try 
                h1 = plot(data.metrics.uav.time+10, data.metrics.uav.z.err, 'Color', 'k', ...
                    'LineWidth', linewidth,  'displayname', 'uav z error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try 
                h1 = plot(data.metrics.uav.time+10, data.metrics.uav.y.err, 'Color', 'r', ...
                    'LineWidth', linewidth,  'displayname', 'uav y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try 
                h1 = plot(data.metrics.uav.time+10, data.metrics.uav.x.err, 'Color', 'b', ...
                    'LineWidth', linewidth, 'displayname', 'uav x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
        grid on
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('Position error [m]')
        
        try 
            current_limits = axis;
            current_axes = gca; % current axes
%             current_axes.XLim = [floor(data.metrics.uav.time(1)) ceil(data.metrics.uav.time(end))];
            current_axes.XLim = [0 90];            
            current_axes.YLim = [-0.2 0.2];
            current_axes.XTick = [current_axes.XLim(1):10:current_axes.XLim(2)];
            current_axes.YTick = [current_axes.YLim(1):.1:current_axes.YLim(2)];
        
            try 
                hold on
                line([data.ugvRecorder.ckf.time(1) data.ugvRecorder.ckf.time(1)],[current_axes.YLim(1) current_axes.YLim(2)-0.5]);
                line([data.ugvRecorder.ckf.time(end) data.ugvRecorder.ckf.time(end)],[current_axes.YLim(1) current_axes.YLim(2)-0.5]);
                hold off
            catch; end
        catch; end
        

    subplot(2,1,2)
        hold on; ii = 0;
            try 
                h1 = plot(data.metrics.uav.yaw.time+10, data.metrics.uav.yaw.err, 'Color', [0,0,0], ...
                    'LineWidth', linewidth, 'displayname', 'uav yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end

        hold off; clear h1 ii
        grid on
        try columnlegend(1,legend_str); catch; end
        ylabel('Yaw error [deg]')
        xlabel('time [s]')
        
        if 0
            %%
            A = data.vicon.est_yaw.spline_error; A(data.uavRecorder.est.time<1.1) = [];
            disp('mean(data.vicon.est_yaw.spline_error)'); disp(mean(A));
            clear A
        end
        
        try
            current_limits = axis;
            current_axes = gca; % current axes
%             current_axes.XLim = [floor(data.ugvRecorder.stereo.time(1)) ceil(data.ugvRecorder.stereo.time(end))];
            current_axes.XLim = [0 90];
            current_axes.XTick = [current_axes.XLim(1):10:current_axes.XLim(2)];
%             current_axes.YLim = [-60 60];
            current_axes.YLim = [-10 10];
            current_axes.YTick = [current_axes.YLim(1):5:current_axes.YLim(2)];
        catch; end
        
        try 
            hold on
            line([data.ugvRecorder.ckf.time(1) data.ugvRecorder.ckf.time(1)],[current_axes.YLim(1) current_axes.YLim(2)-20]);
            line([data.ugvRecorder.ckf.time(end) data.ugvRecorder.ckf.time(end)],[current_axes.YLim(1) current_axes.YLim(2)-20]);
            hold off
        catch; end
        
        
        legend boxoff   


%         hold on
%             for m = 1: length(ugv_move_index)
%                 try line([data.ugvRecorder.ckf.time(ugv_move_index(m)) data.ugvRecorder.ckf.time(ugv_move_index(m))], ...
%                         [current_axes.YLim(1) current_axes.YLim(2)-2]);
%                 catch; end
%             end; clear m
%         hold off
%         clear linewidth ugv_move_index t_offset
%     title(['uav yaw and command ' [meta.date meta.run]])
%     hold on
%         try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees,'k.', 'displayname', 'uav yaw vicon'); catch; end        
%         try plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, 'bo', ...
%                 'MarkerSize', 5, 'displayname', 'uav yaw ref trajectory'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Yaw, 'r.', 'displayname', 'global desired yaw'); catch; end
%         try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.yaw_ugv, 'go', 'displayname', 'uav stereo yaw global'); catch; end
%     hold off
%% figure(504); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(504); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    set(0, 'DefaultAxesFontSize', 32)
    set(0, 'DefaultTextFontSize', 32)

        hold on; ii = 0; 
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'Color', [0,0,0], ...
                    'LineWidth', 8, 'displayname','uav yaw vicon'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.Yaw, '--', 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', 12, 'displayname', 'uav yaw ckf est');  ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, ':', 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', 8, 'displayname','uav yaw ref trajectory'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end

        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('UAV yaw angle [deg]')
        try %current axis
        current_limits = axis;
        current_axes = gca; % current axes

        current_axes.XLim = [7 17];
        current_axes.XTick = [current_axes.XLim(1):2:current_axes.XLim(2)];
%         current_axes.YTick = [-90:90:90];
%         current_axes.YLim = [-90 90];
        current_axes.YTick = [-150:50:50];
        current_axes.YLim = [-150 50];
        clear current_axes current_limits
        xlabel('time [s]')
        end
    grid on
    
    

%     axis([data.vicon.uav.time(1) data.vicon.uav.time(end) 0 12])
% axis([0 10 -150 150])
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
% meta.saveplotroot = [meta.dataroot 'notable/' meta.date];
%         saveas(gcf, [meta.saveplotroot 'yaw_vs_time_022_exp.fig']); 
%         print('-depsc', [meta.saveplotroot 'yaw_vs_time_022_exp.eps']); 
%% figure(505); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(505); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,1,1)
    hold on
        try plot(data.metrics.uav_only.time, data.metrics.uav_only.x.err, 'x'); catch; end
        try plot(data.metrics.uav_only.time, data.metrics.uav_only.y.err, '.'); catch; end
        try plot(data.metrics.uav_only.time, data.metrics.uav_only.z.err, 'o'); catch; end
    hold off
    subplot(2,1,2)
    hold on
        try plot(data.metrics.uav_only.time, data.metrics.uav_only.yaw.err, 's'); catch; end
        try plot(data.uavRecorder.est.time, data.vicon.uav.splines.yaw.global_deg_atEstTime, 's'); catch; end
        
    hold off
%% figure(506); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(506); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 4;
    try
        obstacles_x = [data.vicon.april02_.P.global(end,1) data.vicon.april03_.P.global(end,1) ];
        obstacles_y = [data.vicon.april02_.P.global(end,2) data.vicon.april03_.P.global(end,2) ];
    catch
        try
            obstacles_x = [data.vicon.april06_.P.global(end,1) data.vicon.april03_.P.global(end,1) ];
            obstacles_y = [data.vicon.april06_.P.global(end,2) data.vicon.april03_.P.global(end,2) ];
        catch; end
    end
    
    subplot(2,1,1)        
        hold on; ii = 0; 
            try h4 = plot(data.vicon.april07_.P.global(end,1), data.vicon.april07_.P.global(end,2), '^', 'Color', [0.25,0.25,0.25], ...
                    'displayname', 'goal', 'MarkerFaceColor', [0.75,0.75,0.75], 'MarkerSize', 20); ii = ii + 1; legend_str{ii} = h4.DisplayName; catch; end
            try h3 = plot(obstacles_x,obstacles_y, 's', 'Color', [0,0,0], ...
                    'displayname', 'obstacle', 'MarkerFaceColor', 'k', 'MarkerSize', 20); ii = ii + 1; legend_str{ii} = h3.DisplayName; catch; end
%             try h1 = plot(data.vicon.ugvk.P.global(10:end,1), data.vicon.ugvk.P.global(10:end,2), 'k-', ...'Color', [0.5,0.5,0.5], ...
%                     'LineWidth', 2*linewidth,  'displayname', 'actual path');  ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
            try h1 = plot(data.metrics.kobuki.vPx, data.metrics.kobuki.vPy, 'k-', ...'Color', [0.5,0.5,0.5], ...
                    'LineWidth', 2*linewidth,  'displayname', 'actual path');  ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
            try h2 = plot(data.ugvRecorder.est.Position_gl(:,1), data.ugvRecorder.est.Position_gl(:,2), 'r:', ... 'Color', [0,0,0], ...
                    'LineWidth', linewidth,  'displayname', 'estimated path'); ii = ii + 1; legend_str{ii} = h2.DisplayName; catch; end
        hold off; grid on; 
        
        try columnlegend(4,legend_str); catch; end
        
        xlabel('UGV x position [m]'); 
        ylabel('UGV y position [m]'); 
        clear h1 h2 h3 h4 ii
        
        clear legend_str
        hold on
        try 
        % Vicon ending circle
            x = data.metrics.kobuki.vPx(end); y = data.metrics.kobuki.vPy(end); r = 0.1757; 
            th = 0:pi/50:2*pi; xunit = r * cos(th) + x; yunit = r * sin(th) + y;
            h = plot(xunit, yunit, 'k--');
        % goal location circle
            x = data.vicon.april07_.P.global(end,1); y = data.vicon.april07_.P.global(end,2); r = 0.1437; 
            th = 0:pi/50:2*pi; xunit = r * cos(th) + x; yunit = r * sin(th) + y;
            h = plot(xunit, yunit, 'k--');
        hold off
        catch
        end
        axis([-0.5 4 -0.5 1.75])

    subplot(2,1,2)   
%         hold on
%             try h1 = plot(data.metrics.kobuki.errtime, data.metrics.kobuki.errx, 'Color', [0.75,0.75,0.75], ...
%                 'LineWidth', linewidth, 'displayname', 'ugv x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%             catch; end
%             try h1 = plot(data.metrics.kobuki.errtime, data.metrics.kobuki.erry, '--', 'Color', [0.25,0.25,0.25], ...
%                     'LineWidth', linewidth, 'displayname', 'ugv y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
%             catch; end
%         hold off
            
        hold on; ii = 0; % ]
try         RMS_error = sqrt(...
            data.metrics.kobuki.errx.^2 + data.metrics.kobuki.erry.^2); catch; end
            try h1 = plot(data.metrics.kobuki.odom, RMS_error, '-', 'Color', [0.0,0.0,0.0], ...
                    'LineWidth', linewidth,  'displayname', 'ugv estimate error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
%             try h1 = plot(data.metrics.kobuki.odom, data.metrics.kobuki.errx, '-', 'Color', [0.0,0.0,0.0], ...
%                     'LineWidth', linewidth,  'displayname', 'ugv x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
%             try h1 = plot(data.metrics.kobuki.odom, data.metrics.kobuki.erry, '-', 'Color', [0.5,0.5,0.5], ...
%                     'LineWidth', linewidth,  'displayname', 'ugv y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
%             try h1 = plot(data.ugvRecorder.est.time, data.metrics.ugv_plan.yaw_error, ':', 'Color', [0.75,0.75,0.75], ...
%                     'LineWidth', linewidth,  'displayname', 'ugv yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        hold off; 
        clear h1 ii
        axis([-0.5 4 -0.02 0.02])
        grid on
%         try columnlegend(2,legend_str, 'location', 'southeast'); catch; end; clear legend_str
        legend('toggle')
        legend boxoff   
        legend('location', 'southwest')
        ylabel('Position error [m]')
        xlabel('Distance along trajectory [m]')

    clear linewidth
%% figure(5006); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
figure(5006); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
linewidth = 4;
        hold on; ii = 0; % ]
        try
            RMS_error = sqrt(...
            data.metrics.kobuki.errx.^2 + data.metrics.kobuki.erry.^2); 
        catch
        end
        try h1 = plot(data.metrics.kobuki.odom, RMS_error, '-', 'Color', [0.0,0.0,0.0], ...
            'LineWidth', linewidth,  'displayname', 'ugv estimate error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        hold off; 
        clear h1 ii
        grid on
        legend('toggle')
        legend boxoff   
        legend('location', 'northwest')
        ylabel('Position error [m]')
        xlabel('Distance along trajectory [m]')

    clear linewidth

%%     subplot(2,1,2)  
figure(5007); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on; ii = 0; % ]
            try h1 = plot(kobuki.errtime, kobuki.errx, '-', 'Color', [0.0,0.0,0.0], ...
                    'LineWidth', linewidth,  'displayname', 'ugv x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; 
            catch; end
            try h1 = plot(kobuki.errtime, kobuki.erry, '--', 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth,  'displayname', 'ugv y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
%             try h1 = plot(data.ugvRecorder.est.time, data.metrics.ugv_plan.error(:,3), ':', 'Color', [0.75,0.75,0.75], ...
%                     'LineWidth', linewidth,  'displayname', 'ugv z error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        hold off; clear h1 ii
        grid on
        try columnlegend(3,legend_str, 'location', 'northwest'); catch; end; clear legend_str
        ylabel('Position error [m]')
        xlabel('time [s]')

figure(5008); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on; ii = 0; % ]
            try h1 = plot(kobuki.errtime, kobuki.erryaw, '-', 'Color', [0.0,0.0,0.0], ...
                    'LineWidth', linewidth,  'displayname', 'ugv yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        hold off; clear h1 ii
        grid on
        try columnlegend(1,legend_str, 'location', 'northwest'); catch; end; clear legend_str
        ylabel('Orientation error [deg]')
        xlabel('time [s]')
% clear kobuki
%% figure(507); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(507); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 4;    
        hold on; ii = 0; % ]
            try h1 = plot(data.kobuki_logger.navfn_path.path.xyz{6}(:,1), data.kobuki_logger.navfn_path.path.xyz{6}(:,2), ':', 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth,  'displayname', 'UGV planned path'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
            try h1 = plot(data.vicon.ugvk.P.global(:,1), data.vicon.ugvk.P.global(:,2), '-', 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth,  'displayname', 'UGV actual path'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
            try h1 = plot(data.ugvRecorder.est.Position_gl(:,1), data.ugvRecorder.est.Position_gl(:,2), '--', 'Color', [0,0,0], ...
                    'LineWidth', linewidth,  'displayname', 'UGV estimated path'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
            try h1 = plot(data.uavCon.Desired.Position_g(:,1), data.uavCon.Desired.Position_g(:,2), '--', 'Color', 'b', ...
                    'LineWidth', linewidth,  'displayname', 'UAV desired locations'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
            try h1 = plot(data.vicon.uav.P.global(:,1), data.vicon.uav.P.global(:,2), ':', 'Color', 'r', ...
                    'LineWidth', linewidth,  'displayname', 'UAV actual locations'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
            try h1 = plot(data.vicon.april07_.P.global(1,1), data.vicon.april07_.P.global(1,2), '^', 'Color', [0.25,0.25,0.25], ...
                    'displayname', 'goal', 'MarkerFaceColor', [0.75,0.75,0.75], 'MarkerSize', 20); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end            
            try h1 = plot(data.vicon.april07_.P.global(end,1), data.vicon.april07_.P.global(end,2), '^', 'Color', [0.25,0.25,0.25], ...
                    'displayname', 'goal', 'MarkerFaceColor', [0.75,0.75,0.75], 'MarkerSize', 20); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end            
        hold off; grid on; clear h1 h4 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        xlabel('UGV x position [m]'); 
        ylabel('UGV y position [m]'); 
%% figure(508); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig        
figure(508); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on; ii = 0; % ]
        legend('AutoUpdate','off')
            try 
                h1 = plot(data.uavRecorder.navdata.time, abs(data.vicon.uav.splines.P.global.x_diff./data.vicon.uav.splines.P.global.x_uavesttime), 'Color', [0,0,0], ...
                    'LineWidth', linewidth, 'displayname', 'uav x error %'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try 
                h1 = plot(data.uavRecorder.navdata.time, abs(data.vicon.uav.splines.P.global.y_diff./data.vicon.uav.splines.P.global.y_uavesttime), 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth, 'displayname', 'uav y error %'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try 
                h1 = plot(data.uavRecorder.navdata.time, abs(data.vicon.uav.splines.P.global.z_diff./data.vicon.uav.splines.P.global.z_uavesttime), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'uav z error %'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end

        hold off; clear h1 ii
        grid on
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('Position error [m]')
        
        try 
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XLim = [floor(data.ugvRecorder.stereo.time(1)) ceil(data.ugvRecorder.stereo.time(end))];
            current_axes.YLim = [-0.09 0.09];
            current_axes.XTick = [current_axes.XLim(1):10:current_axes.XLim(2)];
            current_axes.YTick = [current_axes.YLim(1):0.03:current_axes.YLim(2)];
        
            try 
                hold on
                line([data.ugvRecorder.ckf.time(1) data.ugvRecorder.ckf.time(1)],[current_axes.YLim(1) current_axes.YLim(2)-0.5]);
                line([data.ugvRecorder.ckf.time(end) data.ugvRecorder.ckf.time(end)],[current_axes.YLim(1) current_axes.YLim(2)-0.5]);
                hold off
            catch; end
        catch; end
%% figure(510); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(510); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 4;
    t_offset = 0; % -8 for 20170914_040
    subplot(2,1,1)
        hold on; ii = 0; % ]
        legend('AutoUpdate','off')

            try 
                h1 = plot(data.ugvStereo.time, data.caldata.uav.error_ugv(:,1), 'Color', [0,0,0], ...
                    'LineWidth', linewidth, 'displayname', 'uav x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end            
            try 
                h1 = plot(data.ugvStereo.time, data.caldata.uav.error_ugv(:,2), 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth,  'displayname', 'uav y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try 
                h1 = plot(data.ugvStereo.time, data.caldata.uav.error_ugv(:,3), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth,  'displayname', 'uav z error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end

        hold off; clear h1 ii
        grid on
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('Position error [m]')
        
        try 
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XLim = [floor(data.ugvRecorder.stereo.time(1)) ceil(data.ugvRecorder.stereo.time(end))];
            current_axes.YLim = [-1.00 1.00];
            current_axes.XTick = [current_axes.XLim(1):10:current_axes.XLim(2)];
            current_axes.YTick = [current_axes.YLim(1):0.25:current_axes.YLim(2)];
        
            try 
                hold on
                line([data.ugvRecorder.ckf.time(1) data.ugvRecorder.ckf.time(1)],[current_axes.YLim(1) current_axes.YLim(2)-0.5]);
                line([data.ugvRecorder.ckf.time(end) data.ugvRecorder.ckf.time(end)],[current_axes.YLim(1) current_axes.YLim(2)-0.5]);
                hold off
            catch; end
        catch; end
        

    subplot(2,1,2)
        hold on; ii = 0;
            try 
                h1 = plot(data.ugvStereo.time, data.caldata.uav.error_yaw, 'Color', [0,0,0], ...
                    'LineWidth', linewidth, 'displayname', 'uav yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end



        hold off; clear h1 ii
        grid on
        try columnlegend(2,legend_str); catch; end
        ylabel('Yaw error [deg]')
        xlabel('time [s]')
        
        try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XLim = [floor(data.ugvRecorder.stereo.time(1)) ceil(data.ugvRecorder.stereo.time(end))];
            current_axes.XTick = [current_axes.XLim(1):10:current_axes.XLim(2)];
            current_axes.YLim = [-60 60];
            current_axes.YTick = [current_axes.YLim(1):20:current_axes.YLim(2)];
        catch; end
        
        try 
            hold on
            line([data.ugvRecorder.ckf.time(1) data.ugvRecorder.ckf.time(1)],[current_axes.YLim(1) current_axes.YLim(2)-20]);
            line([data.ugvRecorder.ckf.time(end) data.ugvRecorder.ckf.time(end)],[current_axes.YLim(1) current_axes.YLim(2)-20]);
            hold off
        catch; end
%% figure(700); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(700); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Local Plan')
    hold on
    try 
        for i = 1:length(data.kobuki_logger.local_plan.path.xyz)
            try plot(data.kobuki_logger.local_plan.path.xyz{i}(:,1), data.kobuki_logger.local_plan.path.xyz{i}(:,2)); catch; end
        end
    catch
    end
    
    try plot(data.vicon.april02_.P.global(end,1), data.vicon.april02_.P.global(end,2), 'ks', ...
            'MarkerFaceColor', 'k', 'MarkerSize', 20); catch; end
    try plot(data.vicon.april03_.P.global(end,1), data.vicon.april03_.P.global(end,2), 'ks', ...
            'MarkerFaceColor', 'k', 'MarkerSize', 20); catch; end
    try plot(data.vicon.april06_.P.global(end,1), data.vicon.april06_.P.global(end,2), 'ks', ...
            'MarkerFaceColor', 'k', 'MarkerSize', 20); catch; end
    try plot(data.vicon.april07_.P.global(end,1), data.vicon.april07_.P.global(end,2), 'b^', ...
            'MarkerFaceColor', 'b', 'MarkerSize', 20); catch; end
%     try %current axis
%         current_limits = axis; current_axes = gca; 
%         axes = [current_limits(1) current_limits(2) -3 1];
%         current_axes.XTick = [0:10:axes(2)];
%         current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%         current_axes.YTick = [axes(3):0.5:axes(4)];
%         current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%         set(gca,'XTickLabel',{' '})
%     end
    axis([-0.5 4 -1 1])
    xlabel('UGV x position [m]'); 
    ylabel('UGV y position [m]'); 
    grid on
    hold off
%% figure(701); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(701); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Global Plan')
    hold on
    try
    for i = 1:length(data.kobuki_logger.global_plan.path.xyz)
        try plot(data.kobuki_logger.global_plan.path.xyz{i}(:,1), data.kobuki_logger.global_plan.path.xyz{i}(:,2)); catch; end
    end
    catch
    end
    hold off
%% figure(702); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(702); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Local Plan')
    hold on
        try plot(data.kobuki_logger.local_plan.path.xyz{1}(:,1), data.kobuki_logger.local_plan.path.xyz{1}(:,2)); catch; end
    hold off
%% figure(703); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(703); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Global Plan')
    hold on
        try plot(data.kobuki_logger.global_plan.path.xyz{1}(:,1), data.kobuki_logger.global_plan.path.xyz{1}(:,2)); catch; end
    hold off
%% figure(710); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(710); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Local Plan')
    hold on
    try
    for i = 1:length(data.kobuki_logger.local_plan.path.xyz)
        try plot(...
                [data.experiment.plan.path_end(i,1) data.experiment.plan.ugv_waypoint(i,1)], ...
                [data.experiment.plan.path_end(i,2) data.experiment.plan.ugv_waypoint(i,2)]); catch; end
        try plot(data.experiment.plan.path_end(i,1:2), data.experiment.plan.uav_waypoint(i,1:2)); catch; end
        try plot(data.kobuki_logger.local_plan.path.xyz{i}(:,1), data.kobuki_logger.local_plan.path.xyz{i}(:,2), ...
            'LineWidth', 2); catch; end
    end
    catch
    end
    
    try plot(data.uavCon.Desired.Position_g(:,1), data.uavCon.Desired.Position_g(:,2), 'r.'); catch; end
    hold off
%% figure(711); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(711); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Local Plan, x')
    hold on
    try
    for i = 1:length(data.kobuki_logger.local_plan.path.xyz)
        try h1 = plot(data.kobuki_logger.local_plan.time(i), data.kobuki_logger.local_plan.path.xyz{i}(end,1), 'x', 'displayname', 'ugv path local'); catch; end
    end
    catch
    end
    try h2 = plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), 'r.', 'displayname', 'uav desired position'); catch; end
    hold off
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'northwest')
        clear h1 h2 h3 h4 h5
    catch
    end
    
figure(712); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Local Plan, y')
    hold on
    try
    for i = 1:length(data.kobuki_logger.local_plan.path.xyz)
        try h1 = plot(data.kobuki_logger.local_plan.time(i), data.kobuki_logger.local_plan.path.xyz{i}(end,2), 'x', 'displayname', 'ugv path local'); catch; end
    end
    catch
    end
    
    try h2 = plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), 'r.', 'displayname', 'uav desired position'); catch; end
    hold off
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'northwest')
        clear h1 h2 h3 h4 h5
    catch
    end

figure(713); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Local Plan, z')
    hold on
    try
        for i = 1:length(data.kobuki_logger.local_plan.path.xyz)
            try h1 = plot(data.kobuki_logger.local_plan.time(i), data.kobuki_logger.local_plan.path.xyz{i}(end,3), 'x', 'displayname', 'ugv path local'); catch; end
        end
    catch
    end
        
    try h2 = plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,3), 'r.', 'displayname', 'uav desired position'); catch; end
    hold off
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'northwest')
        clear h1 h2 h3 h4 h5
    catch
    end
%% figure(801); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(801); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% Set the figure properties 'PaperPosition' and 'PaperUnits' accordingly.
    linewidth = 3;
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,1), 'd', 'Color', [1,1,1], ...
                     'MarkerFaceColor', [0,0,0], 'displayname', 'uav goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'uav actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'uav estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
        ylabel('X [m]')
        grid on

        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 1 3];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
        end
        try columnlegend(3,legend_str); catch; end; clear legend_str
% print('-depsc', [meta.saveplotroot 'figs/abcde_x_' meta.date(1:end-1) '_' meta.run '.eps'])
% on macOS-matlab:
% gcf:
%     Position: [1 1 1326 268]
%        Units: 'pixels'
% figure(802); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(802); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 3;
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,2), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV y goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'UAV y actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'UAV y estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii

        ylabel('Y [m]')
        grid on
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -1.0 1.0];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
            clear current_axes current_limits
        end
%         xlabel('time [s]')
%         try columnlegend(3,legend_str); catch; end; clear legend_str
% print('-depsc', [meta.saveplotroot 'figs/abcde_y_' meta.date(1:end-1) '_' meta.run '.eps'])
%% figure(803); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(803); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 3;
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.DesiredState_gl(:,3), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV z goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,3), 'Color', [0.75,0.75,0.75], ...
                'LineWidth', linewidth, 'displayname', 'UAV z actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), '--', 'Color', [0.25,0.25,0.25], ...
                'LineWidth', linewidth, 'displayname','UAV z estimated');  ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii

        ylabel('Z [m]')
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 0.0 2.0];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
            clear current_axes current_limits
        end
        grid on
%         try columnlegend(3,legend_str); catch; end; clear legend_str
% print('-depsc', [meta.saveplotroot 'figs/abcde_z_' meta.date(1:end-1) '_' meta.run '.eps'])
%% figure(804); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(804); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 3;
        hold on; ii = 0; 
            if strcmp(meta.date, '20170211iros/')
                A = data.experiment.uav.Waypoint(:,4);
                A(A==0.8) = 0;
                data.experiment.uav.Waypoint(:,4) = A;
                clear A
                data.experiment.uav.Waypoint(:,3) = 0.8;
            end
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,4), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV yaw goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname','UAV yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname','UAV yaw estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
%         try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('Heading [deg]')
        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -75 75];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):30:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits
        end
        grid on
        xlabel('time [s]')
% print('-depsc', [meta.saveplotroot 'figs/abcde_yaw_' meta.date(1:end-1) '_' meta.run '.eps'])
%% figure(805); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(805); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 3;
    subplot(2,1,1)
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,1), 'd', 'Color', [1,1,1], ...
                     'MarkerFaceColor', [0,0,0], 'displayname', 'uav goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'uav actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'uav estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
        ylabel('X [m]')
        grid on

        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 1 3];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
        end
        try columnlegend(3,legend_str); catch; end; clear legend_str

        subplot(2,1,2)
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,2), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV y goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2), 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname', 'UAV y actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'UAV y estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii

        ylabel('Y [m]')
        grid on
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -1.0 1.0];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
            clear current_axes current_limits
        end
%         xlabel('time [s]')
%         try columnlegend(3,legend_str); catch; end; clear legend_str
% print('-depsc', [meta.saveplotroot 'figs/abcde_xy_' meta.date(1:end-1) '_' meta.run '.eps'])
%     Position: [1 1 1326 525]
%        Units: 'pixels'
%% figure(806); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(806); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 3;
        subplot(2,1,1)
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.DesiredState_gl(:,3), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV z goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,3), 'Color', [0.75,0.75,0.75], ...
                'LineWidth', linewidth, 'displayname', 'UAV z actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), '--', 'Color', [0.25,0.25,0.25], ...
                'LineWidth', linewidth, 'displayname','UAV z estimated');  ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii

        ylabel('Z [m]')
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 0.0 2.0];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
            clear current_axes current_limits
        end
        grid on
        subplot(2,1,2)
        hold on; ii = 0; 
            if strcmp(meta.date, '20170211iros/')
                A = data.experiment.uav.Waypoint(:,4);
                A(A==0.8) = 0;
                data.experiment.uav.Waypoint(:,4) = A;
                clear A
                data.experiment.uav.Waypoint(:,3) = 0.8;
            end
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,4), 'd', 'Color', [1,1,1], ...
                    'MarkerFaceColor', [0,0,0], 'displayname', 'UAV yaw goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth, 'displayname','UAV yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname','UAV yaw estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
%         try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('Heading [deg]')
        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -75 75];
            current_axes.XTick = [0:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):30:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits
        end
        grid on
        xlabel('time [s]')
% print('-depsc', [meta.saveplotroot 'figs/abcde_zyaw_' meta.date(1:end-1) '_' meta.run '.eps'])
%% figure(807); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(807); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%         try columnlegend(3,legend_str); catch; end; clear legend_str
%     Position: [1 1 1326 820]
%        Units: 'pixels'

time_start = 20;

    linewidth = 4;
    dashwidth = 2;
    subplot(4,1,1)
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,1), 'd', 'Color', 'r', ...
                     'MarkerFaceColor', 'r', 'displayname', 'uav goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1), 'Color', [0.8,0.8,0.8], ...
                    'LineWidth', linewidth, 'displayname', 'uav actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), '--', 'Color', 'k', ...
                    'LineWidth', dashwidth, 'displayname', 'uav estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
        ylabel('X [m]')
        grid on
        try columnlegend(3,legend_str); catch; end; clear legend_str
        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 1 3];
            current_axes.XTick = [time_start:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
        end
%         legend_h = legend('toggle');
%         set(legend_h, 'Color', 'None', 'Box', 'off');
        

        subplot(4,1,2)
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,2),  'd', 'Color', 'r', ...
                     'MarkerFaceColor', 'r', 'displayname', 'uav goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2), 'Color', [0.8,0.8,0.8], ...
                    'LineWidth', linewidth, 'displayname', 'uav actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), '--', 'Color', 'k', ...
                    'LineWidth', dashwidth, 'displayname', 'uav estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii

        ylabel('Y [m]')
        grid on
        
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -1.0 1.0];
            current_axes.XTick = [time_start:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
            clear current_axes current_limits
        end
        
        subplot(4,1,3)
        hold on; ii = 0; 
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.DesiredState_gl(:,3), 'd', 'Color', 'r', ...
                     'MarkerFaceColor', 'r', 'displayname', 'UAV z goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,3), 'Color', [0.8,0.8,0.8], ...
                'LineWidth', linewidth, 'displayname', 'UAV z actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), '--', 'Color', 'k', ...
                'LineWidth', dashwidth, 'displayname','UAV z estimated');  ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii

        ylabel('Z [m]')
        try %current axis 
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) 0.0 2.0];
            current_axes.XTick = [time_start:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):0.5:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            set(gca,'XTickLabel',{' '})
            clear current_axes current_limits
        end
        grid on
        subplot(4,1,4)
        hold on; ii = 0; 
            if strcmp(meta.date, '20170211iros/')
                A = data.experiment.uav.Waypoint(:,4);
                A(A==0.8) = 0;
                data.experiment.uav.Waypoint(:,4) = A;
                clear A
                data.experiment.uav.Waypoint(:,3) = 0.8;
            end
            try h1 = plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,4),  'd', 'Color', 'r', ...
                     'MarkerFaceColor', 'r', 'displayname', 'UAV yaw goal'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'Color', [0.8,0.8,0.8], ...
                    'LineWidth', linewidth, 'displayname','UAV yaw actual'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw,  '--', 'Color', 'k', ...
                    'LineWidth', dashwidth, 'displayname','UAV yaw estimated'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off; clear h1 ii
        ylabel('Heading [deg]')
        try %current axis
            current_limits = axis; current_axes = gca; 
            axes = [current_limits(1) current_limits(2) -75 75];
            current_axes.XTick = [time_start:10:axes(2)];
            current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
            current_axes.YTick = [axes(3):30:axes(4)];
            current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
            clear current_axes current_limits
        end
        grid on
        xlabel('time [s]')
        
        clear time_start
% set(gcf, 'WindowStyle','normal')
% set(gcf, 'visible', 'off')
% set(gcf, 'Position', [1 1 1200 1600]);
% print('-depsc', [meta.saveplotroot 'figs/abcde_xyzyaw_' meta.date(1:end-1) '_' meta.run '.eps'])
%% figure(851); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(851); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    
    linewidth = 3;
    subplot(2,1,1)
    title('ugv errors while in motion')
        hold on
            try h1 = plot(data.metrics.kobuki.errtime, data.metrics.kobuki.errx, 'Color', [0.75,0.75,0.75], ...
                'LineWidth', linewidth, 'displayname', 'ugv x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try plot(data.metrics.kobuki.errtime, data.metrics.kobuki.erry, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'ugv y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off
    subplot(2,1,2)
        hold on
            try plot(data.metrics.kobuki.errtime, data.metrics.kobuki.erryaw, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'ugv yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off
%% figure(852); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(852); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    
    linewidth = 3;
    subplot(2,1,1)
    title('uav errors while in motion')
        hold on
            try h1 = plot(data.metrics.uav_during_path.time, data.metrics.uav_during_path.errx, 'Color', [0.75,0.75,0.75], ...
                'LineWidth', linewidth, 'displayname', 'ugv x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try plot(data.metrics.uav_during_path.time, data.metrics.uav_during_path.erry, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'ugv y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try plot(data.metrics.uav_during_path.time, data.metrics.uav_during_path.erry, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'ugv z error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off
    subplot(2,1,2)
        hold on
            try plot(data.metrics.uav_during_path.time, data.metrics.uav_during_path.erryaw, '--', 'Color', [0.25,0.25,0.25], ...
                    'LineWidth', linewidth, 'displayname', 'ugv yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
        hold off
end

% imread() can import whole bunch of graphical data types, including .jpg and .png. Conversion to .png, as you ask, is not necessary.
% 
% img = imread('filename.png');
% image(img);
% To alter the size/positioning of the image within your figure, you can touch the underlying axes object:
% 
% x = 0;
% y = 0;
% width = 0.5;% measured relative to the figure width
% height = 0.5;% measured relative to the figure height
% set(gca,'units','normalized','position',[x y width height])


function data = splineYawFig502(data)
    spline = csapi(data.vicon.uav.time,data.vicon.uav.yaw.global.degrees);
    vector_newtime = fnval(spline, data.uavRecorder.est.time);
    data.vicon.uav.splines.yaw.global_deg_atEstTime = vector_newtime;
    data.vicon.uav.yaw.global.degrees_error = data.uavRecorder.est.Yaw - data.vicon.uav.yaw.global.degrees_atEstTime;

end

function savefigbynum(meta, fignum)
    figure(fignum)
    current_fig = gcf;
    meta.savefilename = ['figure(' num2str(fignum) ')'];
    saveas(gcf, [meta.figpath meta.savefilename '.png']); 
    clear current_fig;    
end

function sump = getDisplacement(A)
% getDisplacement(data.vicon.ugvk.atEstTime.P.global')
    p = vecnorm(A)';
    dp(1) = 0;
    sump(1) = 0;
    for i = 1:(length(p)-1)
        dp(i+1,1) = abs(p(i+1)-p(i));
        sump(i+1,1) = sump(i,1) + dp(i+1,1);
    end
end

function saveplotdata()
%%
% save([meta.dataroot meta.date meta.run '/timers_' meta.run '.mat'], 'timers')



end

%% Cruft
% %% figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(21); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% 
% 
% boolstart = 12.2;
% boolend = 30;
% k = 1;
% 
% for i = 1:0.1:(boolend - boolstart)
% % for i = 10:0.1:30
% %     boolselector = data.uavRecorder.navdata.time<i;
%     boolselector = ~(data.uavRecorder.navdata.time<boolstart).* ~(data.uavRecorder.navdata.time>(boolstart+i)) ;
%     time_short = data.uavRecorder.navdata.time; time_short(~boolselector) = [];
%     data_short = data.uavRecorder.navdata.compassError; data_short(~boolselector) = [];
%     fitvars = polyfit(time_short, data_short, 1);
% %     disp(['y = mx+b, [m b] = ' num2str(fitvars(1)) ' ' num2str(fitvars(2)) ']' ])
%     m(k,1) = fitvars(1);
%     b(k,2) = fitvars(2);
%     k = k+1;
% end; clear i
% 
% % size(m)
% % 
% % m = fitvars(1);
% % c = fitvars(2);
% 
% disp(['y = mx+b, [m b] = ' num2str(fitvars(1)) ' ' num2str(fitvars(2)) ']' ])
% 
% hold on
%     try plot(time_short, data_short, 'o', 'displayname', 'compass error'); catch; end
%     try line([time_short(1) time_short(end)],...
%         [fitvars(1)*time_short(1)+ fitvars(2)...
%          fitvars(1)*time_short(end)+ fitvars(2)], 'Color', 'black', 'LineWidth', 5, 'displayname', 'yaw diff fitline'); catch; end
%     try plot(data.uavRecorder.navdata.time, data.uavRecorder.navdata.RPY(:,3), '.', 'displayname', 'navdata uav yaw'); catch; end
%     try plot(data.vicon.uav.time, data.vicon.uav.yaw.degrees, 'x', 'displayname', 'vicon uav yaw'); catch; end
% hold off
% legend('toggle')
% %% figure(22); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(22); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% hold on
%     plot(m, 'displayname', 'slope of navdata compass')
%     
%     
% hold off
% legend('toggle')
% 
% 
% 
% clear boolselector time_short data_short fitvars m b 
% %% figure(24); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(24); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
%     title('uav command gains x global')
%     hold on
%         try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,1), '.', 'displayname', 'uav current p global'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), '.', 'displayname', 'uav desired p global'); catch; end
%         try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,1), '.', 'displayname', 'uav cmd global'); catch; end
%     hold off
%     grid on
%     legend('toggle')
%     xlabel('time')
%     ylabel('x meters')
% %% figure(25); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(25); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('uav command gains x local')
%     hold on
%         try plot(data.uavCon.time, data.uavCon.Current.Position_dr(:,1), '.', 'displayname', 'uav current p local'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_dr(:,1), '.', 'displayname', 'uav desired p local'); catch; end
%         try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,1), '.', 'displayname', 'uav cmd local'); catch; end
%     hold off
%     grid on
%     legend('toggle')
% %% figure(26); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(26); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
%     title('uav command gains y global')
%     hold on
%         try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,2), '.', 'displayname', 'uav current p global'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), '.', 'displayname', 'uav desired p global'); catch; end
%         try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,2), '.', 'displayname', 'uav cmd global'); catch; end
%     hold off
%     grid on
%     legend('toggle')
% %% figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('uav command gains y local')
%     hold on
%         try plot(data.uavCon.time, data.uavCon.Current.Position_dr(:,2), '.', 'displayname', 'uav current p local'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Position_dr(:,2), '.', 'displayname', 'uav desired p local'); catch; end
%         try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,2), '.', 'displayname', 'uav cmd local'); catch; end
%     hold off
%     grid on
%     legend('toggle')

%% figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(3,1,1)
%         hold on
% %             try plot(data.vicon.red_target.time, data.vicon.red_target.P.cam(:,1), 'r.', 'displayname', 'red-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,1), 'r+', 'displayname', 'red-stereo-x-ugv'); catch; end
% %             try plot(data.ugvStereo.time, data.ugvStereo.uav.yaw_ugv, 'go', 'displayname', 'uav stereo yaw ugv frame'); catch; end
% 
%         hold off
% %         try
% %             current_limits = axis;
% %             current_axes = gca; % current axes
% %             current_axes.XTick = [0:5:data.ugvStereo.time(end)];
% %             current_axes.XLim = [0 data.ugvStereo.time(end)];
% %             current_axes.YLim = [-1.25 1.25];
% %             current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
% %             clear current_axes current_limits
% %         end
%         grid on
%         title(['vicon marker positions (x) in ugv frame' [meta.date meta.run]])
%         ylabel('x [m]')
%         legend('toggle')
%     subplot(3,1,2)
%         hold on
%             try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.cam(:,1), 'b.', 'displayname', 'blue-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_ugv(:,1), 'b+', 'displayname', 'blue-stereo-x-ugv'); catch; end
%         hold off
%         try
% %         current_limits = axis;
% %         current_axes = gca; % current axes
% %         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
% %         current_axes.XLim = [0 data.ugvStereo.time(end)];
% %         current_axes.YLim = [-1.25 1.25];
% %         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
% %         clear current_axes current_limits
%         end
%         ylabel('x [m]')
%         grid on
%         legend('toggle')
%     subplot(3,1,3)
%         hold on
%             try plot(data.vicon.green_target.time, data.vicon.green_target.P.cam(:,1), 'g.', 'displayname', 'green-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Green.P_ugv(:,1), 'g+', 'displayname', 'green-stereo-x-ugv'); catch; end
%         hold off
%         try
% %         current_limits = axis;
% %         current_axes = gca; % current axes
% %         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
% %         current_axes.XLim = [0 data.ugvStereo.time(end)];
% %         current_axes.YLim = [-1.25 1.25];
% %         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
% %         clear current_axes current_limits
%         end
%         ylabel('x [m]')
%         grid on
%         legend('toggle')
%% figure(106); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(106); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(3,1,1)
%         hold on
% %             try plot(data.vicon.red_target.time, data.vicon.red_target.P.cam(:,1), 'r.', 'displayname', 'red-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,2), 'r+', 'displayname', 'red-stereo-y-ugv'); catch; end
% %             try plot(data.ugvStereo.time, data.ugvStereo.uav.yaw_ugv, 'go', 'displayname', 'uav stereo yaw ugv frame'); catch; end
% 
%         hold off
% %         try
% %             current_limits = axis;
% %             current_axes = gca; % current axes
% %             current_axes.XTick = [0:5:data.ugvStereo.time(end)];
% %             current_axes.XLim = [0 data.ugvStereo.time(end)];
% %             current_axes.YLim = [-1.25 1.25];
% %             current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
% %             clear current_axes current_limits
% %         end
%         grid on
%         title(['vicon marker positions (x) in ugv frame' [meta.date meta.run]])
%         ylabel('y [m]')
%         legend('toggle')
%     subplot(3,1,2)
%         hold on
% %             try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.cam(:,1), 'b.', 'displayname', 'blue-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_ugv(:,2), 'b+', 'displayname', 'blue-stereo-y-ugv'); catch; end
%         hold off
%         try
% %         current_limits = axis;
% %         current_axes = gca; % current axes
% %         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
% %         current_axes.XLim = [0 data.ugvStereo.time(end)];
% %         current_axes.YLim = [-1.25 1.25];
% %         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
% %         clear current_axes current_limits
%         end
%         ylabel('y [m]')
%         grid on
%         legend('toggle')
%     subplot(3,1,3)
%         hold on
% %             try plot(data.vicon.green_target.time, data.vicon.green_target.P.cam(:,1), 'g.', 'displayname', 'green-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Green.P_ugv(:,2), 'g+', 'displayname', 'green-stereo-y-ugv'); catch; end
%         hold off
%         try
% %         current_limits = axis;
% %         current_axes = gca; % current axes
% %         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
% %         current_axes.XLim = [0 data.ugvStereo.time(end)];
% %         current_axes.YLim = [-1.25 1.25];
% %         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
% %         clear current_axes current_limits
%         end
%         ylabel('y [m]')
%         grid on
%         legend('toggle')

%% figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(3,1,1)
%         hold on
%             try plot(data.vicon.red_target.time, data.vicon.red_target.P.cam(:,1), 'r.', 'displayname', 'red-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Red.P_cam(:,1), 'r+', 'displayname', 'red-stereo-x'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         grid on
%         title(['vicon marker positions (x) in camera frame' [meta.date meta.run]])
%         ylabel('x [m]')
%         legend('toggle')
%     subplot(3,1,2)
%         hold on
%             try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.cam(:,1), 'b.', 'displayname', 'blue-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_cam(:,1), 'b+', 'displayname', 'blue-stereo-x'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('x [m]')
%         grid on
%         legend('toggle')
%     subplot(3,1,3)
%         hold on
%             try plot(data.vicon.green_target.time, data.vicon.green_target.P.cam(:,1), 'g.', 'displayname', 'green-vicon-x'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Green.P_cam(:,1), 'g+', 'displayname', 'green-stereo-x'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('x [m]')
%         grid on
%         legend('toggle')
%% figure(106); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(106); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(3,1,1)
%         hold on
%             try plot(data.vicon.red_target.time, data.vicon.red_target.P.cam(:,2), 'r.', 'displayname', 'red-vicon-y'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Red.P_cam(:,2), 'r+', 'displayname', 'red-stereo-y'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('y [m]')
%         grid on
%         title(['vicon marker positions (y) in camera frame' [meta.date meta.run]])
%         legend('toggle')
%     subplot(3,1,2)
%         hold on
%             try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.cam(:,2), 'b.', 'displayname', 'blue-vicon-y'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_cam(:,2), 'b+', 'displayname', 'blue-stereo-y'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('y [m]')
%         grid on
%         legend('toggle')
%     subplot(3,1,3)
%         hold on
%             try plot(data.vicon.green_target.time, data.vicon.green_target.P.cam(:,2), 'g.', 'displayname', 'green-vicon-y'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Green.P_cam(:,2), 'g+', 'displayname', 'green-stereo-y'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('y [m]')
%         grid on
%         legend('toggle')
%% figure(107); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(107); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(3,1,1)
%         hold on
%             try plot(data.vicon.red_target.time, data.vicon.red_target.P.cam(:,3), 'ro', 'displayname', 'red-vicon-z'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Red.P_cam(:,3), 'r+', 'displayname', 'red-stereo-z'); catch; end
%         hold off
%         title(['vicon marker positions (z) in camera frame' [meta.date meta.run]])
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [0 5];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('z [m]')
%         grid on
%         grid on
%         legend('toggle')
%     subplot(3,1,2)
%         hold on
%             try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.cam(:,3), 'bo', 'displayname', 'blue-vicon-z'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_cam(:,3), 'b+', 'displayname', 'blue-stereo-z'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [0 5];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('z [m]')
%         grid on
%         grid on
%         legend('toggle')
%     subplot(3,1,3)
%         hold on
%             try plot(data.vicon.green_target.time, data.vicon.green_target.P.cam(:,3), 'go', 'displayname', 'green-vicon-z'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.Green.P_cam(:,3), 'g+', 'displayname', 'green-stereo-z'); catch; end
%         hold off
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [0 5];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
%         end
%         ylabel('z [m]')
%         grid on
%         grid on
%         legend('toggle')
%% figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(108); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(['vicon marker positions (z) in vicon frame' [meta.date meta.run]])
%     hold on
%         try plot(data.vicon.red_target.time, data.vicon.red_target.P.vicon(:,1), '.', 'displayname', 'uav x'); catch; end%UAV location, x
%         try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.vicon(:,1), '.', 'displayname', 'uav x'); catch; end%UAV location, x
%         try plot(data.vicon.green_target.time, data.vicon.green_target.P.vicon(:,1), '.', 'displayname', 'uav x'); catch; end%UAV location, x
%     hold off
%     grid on
%     legend('toggle')
%% figure(702); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(702); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,1), 'ro', 'displayname', 'red-pixels-xl'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Red.right.xy(:,1), 'ro', 'displayname', 'red-pixels-xr'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,2), 'rs', 'displayname', 'red-pixels-yl'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Red.right.xy(:,2), 'rs', 'displayname', 'red-pixels-yr'); catch; end
%         legend('toggle')
%         grid on    
%     grid on
%     xlabel('time')
%     if 0
%         current_fig = gcf;
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
%         clear current_fig;
%     end
% % figure(703); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(703); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,1), 'bo', 'displayname', 'blue-pixels-xl'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Blue.right.xy(:,1), 'bo', 'displayname', 'blue-pixels-xr'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,2), 'bs', 'displayname', 'blue-pixels-yl'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Blue.right.xy(:,2), 'bs', 'displayname', 'blue-pixels-yr'); catch; end
%         legend('toggle')
%         grid on
%     xlabel('time')
%     if 0
%         current_fig = gcf;
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
%         clear current_fig;
%     end
% % figure(704); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(704); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,1), 'go', 'displayname', 'green-pixels-xl'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Green.right.xy(:,1), 'go', 'displayname', 'green-pixels-xr'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,2), 'gs', 'displayname', 'green--pixels-yl'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(data.ugvStereo.time, data.ugvStereo.Green.right.xy(:,2), 'gs', 'displayname', 'green-pixels-yr'); catch; end
%         legend('toggle')
%         grid on
%     xlabel('time')
%     if 0
%         current_fig = gcf;
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
%         clear current_fig;
%     end

%% disp('sorting pixel values by yaw cov')
% disp('sorting pixel values by yaw cov')
% try ugvMatlab.sorted = sort_pixels_by_yaw_uncert(ugvStereo, ugvMatlab); catch; end
% % figure(7002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(7002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Red.left.xy(:,1), 'ro', 'displayname', 'red-pixels-xl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Red.right.xy(:,1), 'ro', 'displayname', 'red-pixels-xr sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Red.left.xy(:,2), 'rs', 'displayname', 'red-pixels-yl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Red.right.xy(:,2), 'rs', 'displayname', 'red-pixels-yr sorted'); catch; end
%         legend('toggle')
%         grid on    
%     grid on
%     xlabel('time')
% if 0
%     %%
%     current_fig = gcf;
%     saveas(gcf, '~/benjamin/git/hast/data/pixelcomp/red_exp_sorted.png'); 
%     print('-depsc', '~/benjamin/git/hast/data/pixelcomp/red_exp_sorted.eps'); 
%     clear current_fig;
% end
% % figure(7003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(7003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Blue.left.xy(:,1), 'bo', 'displayname', 'blue-pixels-xl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Blue.right.xy(:,1), 'bo', 'displayname', 'blue-pixels-xr sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Blue.left.xy(:,2), 'bs', 'displayname', 'blue-pixels-yl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Blue.right.xy(:,2), 'bs', 'displayname', 'blue-pixels-yr sorted'); catch; end
%         legend('toggle')
%         grid on
%     xlabel('time')
%     if 0
%         current_fig = gcf;
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
%         clear current_fig;
%     end
% if 0
%     %%
%     current_fig = gcf;
%     saveas(gcf, '~/benjamin/git/hast/data/pixelcomp/blue_exp_sorted.png'); 
%     print('-depsc', '~/benjamin/git/hast/data/pixelcomp/blue_exp_sorted.eps'); 
%     clear current_fig;
% end
% % figure(7004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(7004); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     subplot(2,2,1)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Green.left.xy(:,1), 'go', 'displayname', 'green-pixels-xl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,2)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Green.right.xy(:,1), 'go', 'displayname', 'green-pixels-xr sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,3)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Green.left.xy(:,2), 'gs', 'displayname', 'green--pixels-yl sorted'); catch; end
%         legend('toggle')
%         grid on
%     subplot(2,2,4)
%         try  plot(data.ugvStereo.time, ugvMatlab.sorted.Green.right.xy(:,2), 'gs', 'displayname', 'green-pixels-yr sorted'); catch; end
%         legend('toggle')
%         grid on
%     xlabel('time')
%     if 0
%         current_fig = gcf;
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
%         clear current_fig;
%     end
% if 0
%     %
%     current_fig = gcf;
%     saveas(gcf, '~/benjamin/git/hast/data/pixelcomp/green_exp_sorted.png'); 
%     print('-depsc', '~/benjamin/git/hast/data/pixelcomp/green_exp_sorted.eps'); 
%     clear current_fig;
% end

%% figure(800); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     try pidfields = fields(data.pid);
%         % figure(800+10*pidfield); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%         for pidfield = 2:length(pidfields)
%             disp(pidfields{pidfield})
%         end
%     catch
%     end
% 
% figure(801); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% % title([''])
%     hold on
%         try plot(data.pid.time, data.pid.roll.state, '.', 'displayname', 'uav pid roll state'); catch; end
%         try plot(data.pid.time, data.pid.roll.input, 'o', 'displayname', 'uav pid roll input'); catch; end
%     hold off; grid on
%     ylabel('roll [rad?]'), xlabel('time [s]')
%     legend('toggle');legend('Location', 'SouthEast')
%     
%     
% figure(802); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     hold on
%         try plot(data.pid.time, data.pid.pitch.state, '.', 'displayname', 'uav pid pitch state'); catch; end
%         try plot(data.pid.time, data.pid.pitch.input, '.', 'displayname', 'uav pid pitch input'); catch; end
%     hold off; grid on
%     ylabel('pitch [rad?]'), xlabel('time [s]')
%     legend('toggle');legend('Location', 'SouthEast')
% figure(803); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     hold on
%         try plot(data.pid.time, data.pid.yaw.state, '.', 'displayname', 'uav pid yaw state'); catch; end
%         try plot(data.pid.time, data.pid.yaw.input, '.', 'displayname', 'uav pid yaw input'); catch; end
%     hold off; grid on
%     ylabel('yaw [rad?]'), xlabel('time [s]')
%     legend('toggle');legend('Location', 'SouthEast')
% figure(804); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     hold on
%         try plot(data.pid.time, data.pid.velocity_x.state, '.', 'displayname', 'uav pid velocity_x state'); catch; end
%         try plot(data.pid.time, data.pid.velocity_x.input, '.', 'displayname', 'uav pid velocity_x input'); catch; end
%     hold off; grid on
%     ylabel('velocity x [m/s]'), xlabel('time [s]')
%     legend('toggle');legend('Location', 'SouthEast')
% figure(805); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     hold on
%         try plot(data.pid.time, data.pid.velocity_y.state, '.', 'displayname', 'uav pid velocity_y state'); catch; end
%         try plot(data.pid.time, data.pid.velocity_y.input, '.', 'displayname', 'uav pid velocity_y input'); catch; end
%     hold off; grid on
%     ylabel('velocity y [m/s]'), xlabel('time [s]')
%     legend('toggle');legend('Location', 'SouthEast')
% figure(806); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     hold on
%         try plot(data.pid.time, data.pid.velocity_z.state, '.', 'displayname', 'uav pid velocity_z state'); catch; end
%         try plot(data.pid.time, data.pid.velocity_z.input, '.', 'displayname', 'uav pid velocity_z input'); catch; end
%     hold off; grid on
%     ylabel('velocity z [m/s]'), xlabel('time [s]')
%     legend('toggle');legend('Location', 'SouthEast')

