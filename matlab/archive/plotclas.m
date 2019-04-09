%% Turn plotting back on
set(0, 'DefaultFigureVisible', 'on');
figHandles = findall(0, 'Type', 'figure');
set(figHandles(:), 'visible', 'on');
clear figHandles

% try april = data.april;catch;end
% try uavCon = data.uavCon;catch;end
% try ugvStereo = data.ugvStereo;catch;end
% try experiment= data.experiment;catch;end
% try data.uavRecorder = data.uavRecorder;catch;end
% try ugvRecorder = data.ugvRecorder;catch;end
% try timers = data.timers;catch;end
% try vicon = data.vicon;catch;end


close all
tic
disp('Plotting...')

disp('agent xyz positions in various frames')
%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav x position ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1),'k.', 'displayname', 'uav Px global'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), 'b.', 'displayname', 'uav ckf est x'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,1), 'go', 'displayname', 'uav stereo x global'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,1), '.-', 'displayname', 'ugvstereo x'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), 'r.', 'displayname', 'global desired x'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,1), 'm.', 'displayname', 'global cmd x'); catch; end
        
    hold off
    ylabel('X [m]')
    xlabel('time [s]')
    try %current axis
    current_limits = axis; current_axes = gca; 
    current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
    clear current_axes current_limits
    end

    grid on
    legend('toggle')
%% figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav y position ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2),'k.', 'displayname', 'uav Py global'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), 'b.', 'displayname', 'uav ckf est y'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,2), 'go', 'displayname', 'uav stereo y global'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,2), 'displayname', 'ugvstereo y'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), 'r.', 'displayname', 'global desired y'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,2), 'm.', 'displayname', 'global cmd y'); catch; end
    hold off
    ylabel('Y [m]')
    xlabel('time [s]')
    try %current axis
    current_limits = axis; current_axes = gca; 
    current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
    clear current_axes current_limits
    end

    grid on
    legend('toggle')
%% figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['uav z position ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.P.ugv(:,3),'k.', 'displayname', 'uav Pz ugv'); catch; end        
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'uav ckf est z'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.Position_gl(:,3), 'go', 'displayname', 'uav stereo z global'); catch; end
        try plot(data.uavRecorder.navdata.time,uavRecorder.navdata.Alt, 'g.', 'displayname', 'echoAlt'); catch; end
        try plot(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,3), 'displayname', 'ugvstereo z'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,3), 'r.', 'displayname', 'global desired z'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,3), 'm.', 'displayname', 'global cmd z'); catch; end
        
    hold off
    ylabel('z [m]')
    xlabel('time [s]')
    try %current axis
    current_limits = axis; current_axes = gca; 
    current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
    clear current_axes current_limits
    end

    grid on
    legend('toggle')
%% figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(['uav yaw and command ' [meta.date meta.run]])
    hold on
        try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees,'k.', 'displayname', 'UAV yaw actual'); catch; end
        try plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, 'b.', 'displayname', 'UAV yaw estimated'); catch; end
        try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.yaw_ugv, 'go', 'displayname', 'uav stereo yaw global'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Yaw, 'r.', 'displayname', 'global desired yaw'); catch; end
        try plot(data.uavCon.time, 1000*data.uavCon.cmd.yawrate, 'm.', 'displayname', 'global cmd (400x) '); catch; end
    hold off
    ylabel('degrees')
    xlabel('time [s]')
    grid on
    try %current axis
    current_limits = axis; current_axes = gca; 
    current_axes.XLim = [(data.ugvStereo.time(1)-10) (data.uavRecorder.est.time(end)+10)];
    clear current_axes current_limits
    end

    legend('toggle')
%% figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    set(0, 'DefaultAxesFontSize', 32)
    set(0, 'DefaultTextFontSize', 32)
    linewidth = 3;

    subplot(2,1,1)
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
        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('x [m]')
        try %current axis
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 data.uavRecorder.est.time(end)];
        current_axes.YTick = [0:2:6];
        current_axes.YLim = [0 6];
        clear current_axes current_limits
        end
        %%
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
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('y [m]')
        try %current axis 
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XTick = [10 20 30 40 50 60];
            current_axes.XLim = [0 data.uavRecorder.est.time(end)];
            current_axes.YTick = [-1 0 1];
            current_axes.YLim = [-1 1];
            clear current_axes current_limits
        end

figure(6); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
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
        hold off; clear h1 ii
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('z [m]')
        try %current axis
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XTick = [10 20 30 40 50 60];
            current_axes.XLim = [0 data.uavRecorder.est.time(end)];
            current_axes.YTick = [0:2];
            current_axes.YLim = [0 2];
            clear current_axes current_limits
        end

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
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('yaw angle degrees')
        try %current axis
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [10 20 30 40 50 60];
        current_axes.XLim = [0 data.uavRecorder.est.time(end)];
        current_axes.YTick = [-90:90:90];
        current_axes.YLim = [-90 90];
        clear current_axes current_limits
        xlabel('time [s]')
        end
        
    set(0, 'DefaultAxesFontSize', 20)
    set(0, 'DefaultTextFontSize', 20)

%% figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(7); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%         title(['estimated errors ' meta.date meta.run])
    set(0, 'DefaultAxesFontSize', 32)
    set(0, 'DefaultTextFontSize', 32)
    
    try   
        tshift = 0.15;
        metrics.uav.time = data.uavRecorder.est.time .*(data.uavRecorder.est.time<(data.ugvRecorder.est.time(end))) .* (data.uavRecorder.est.time>(data.ugvRecorder.est.time(1)+tshift)); metrics.uav.time(metrics.uav.time==0) = [];
        metrics.uav.x.err = data.vicon.uav.splines.P.global.x_diff .*(data.uavRecorder.est.time<data.ugvRecorder.est.time(end)) .* (data.uavRecorder.est.time>(data.ugvRecorder.est.time(1)+tshift)); metrics.uav.x.err(metrics.uav.x.err==0) = [];
        metrics.uav.y.err = data.vicon.uav.splines.P.global.y_diff .*(data.uavRecorder.est.time<data.ugvRecorder.est.time(end)) .* (data.uavRecorder.est.time>(data.ugvRecorder.est.time(1)+tshift)); metrics.uav.y.err(metrics.uav.y.err==0) = [];
        metrics.uav.z.err = data.vicon.uav.splines.P.global.z_diff .*(data.uavRecorder.est.time<data.ugvRecorder.est.time(end)) .* (data.uavRecorder.est.time>(data.ugvRecorder.est.time(1)+tshift)); metrics.uav.z.err(metrics.uav.z.err==0) = [];
        metrics.uav.yaw.err = data.vicon.est_yaw.spline_error .*(data.uavRecorder.est.time<(data.ugvRecorder.est.time(end))) .* (data.uavRecorder.est.time>(data.ugvRecorder.est.time(1)+tshift)); metrics.uav.yaw.err(metrics.uav.yaw.err==0) = [];
        
        metrics.uav.x.mean = mean(metrics.uav.x.err); 
        metrics.uav.y.mean = mean(metrics.uav.y.err); 
        metrics.uav.z.mean = mean(metrics.uav.z.err);
        metrics.uav.yaw.mean = mean(metrics.uav.yaw.err);
        disp('metrics.uav.[ x y z yaw].mean'); disp([metrics.uav.x.mean metrics.uav.y.mean metrics.uav.z.mean metrics.uav.yaw.mean])

        metrics.uav.x.max = max(abs(metrics.uav.x.err)); 
        metrics.uav.y.max = max(abs(metrics.uav.y.err)); 
        metrics.uav.z.max = max(abs(metrics.uav.z.err));
        metrics.uav.yaw.max = max(abs(metrics.uav.yaw.err));
        disp('metrics.uav.[ x y z yaw].max'); disp([metrics.uav.x.max metrics.uav.y.max metrics.uav.z.max metrics.uav.yaw.max])

        
        metrivs.ugv.time = data.ugvRecorder.est.time;
        metrics.ugv.x.err = data.vicon.ugvc.splines.P.global.x_diff ; 
        metrics.ugv.y.err = data.vicon.ugvc.splines.P.global.y_diff ; 
        metrics.ugv.z.err = data.vicon.ugvc.splines.P.global.z_diff ;
        metrics.ugv.yaw.err = data.vicon.ugvc.splines.yaw.est_err;

        metrics.ugv.x.mean = mean(metrics.ugv.x.err); 
        metrics.ugv.y.mean = mean(metrics.ugv.y.err); 
        metrics.ugv.z.mean = mean(metrics.ugv.z.err);
        metrics.ugv.yaw.mean = mean(metrics.ugv.yaw.err);
        disp('metrics.ugv.[ x y z yaw].mean'); disp([metrics.ugv.x.mean metrics.ugv.y.mean metrics.ugv.z.mean metrics.ugv.yaw.mean])

        metrics.ugv.x.max = max(abs(metrics.ugv.x.err)); 
        metrics.ugv.y.max = max(abs(metrics.ugv.y.err)); 
        metrics.ugv.z.max = max(abs(metrics.ugv.z.err));
        metrics.ugv.yaw.max = max(abs(metrics.ugv.yaw.err));
        disp('metrics.ugv.[ x y z yaw].max'); disp([metrics.ugv.x.max metrics.ugv.y.max metrics.ugv.z.max metrics.ugv.yaw.max])


    catch; end
    linewidth = 4;
    t_offset = 0; % -8 for 20170914_040
    subplot(2,1,1)
        hold on; ii = 0; % ]
            try 
%                 h1 = plot(data.uavRecorder.est.time , data.vicon.uav.splines.P.global.x_diff, 'Color', [0,0,0], ...
                h1 = plot(metrics.uav.time, metrics.uav.x.err, 'Color', [0,0,0], ...
                    'LineWidth', linewidth, 'displayname', 'uav x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.ugvRecorder.est.time , data.vicon.ugvc.splines.P.global.x_diff, ':', 'Color', [0,0,0], ...
                    'LineWidth', linewidth,  'displayname', 'ugv x error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end    

            try 
%                 h1 = plot(data.uavRecorder.est.time , data.vicon.uav.splines.P.global.y_diff, 'Color', [0.5,0.5,0.5], ...
                h1 = plot(metrics.uav.time, metrics.uav.y.err, 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth,  'displayname', 'uav y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.ugvRecorder.est.time , data.vicon.ugvc.splines.P.global.y_diff, ':', 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth,  'displayname', 'ugv y error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end    

            try 
%                 h1 = plot(data.uavRecorder.est.time , data.vicon.uav.splines.P.global.z_diff, 'Color', [0.75,0.75,0.75], ...
                h1 = plot(metrics.uav.time, metrics.uav.z.err, 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth,  'displayname', 'uav z error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try h1 = plot(data.ugvRecorder.est.time , data.vicon.ugvc.splines.P.global.z_diff, ':', 'Color', [0.75,0.75,0.75], ...
                    'LineWidth', linewidth,  'displayname', 'ugv z error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
                        
        hold off; clear h1 ii
        grid on
        try columnlegend(3,legend_str); catch; end; clear legend_str
        ylabel('meters [m]')
try 
    current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XLim = [floor(data.ugvRecorder.est.time(1)) ceil(data.ugvRecorder.est.time(end))];
            current_axes.YLim = [-0.75 0.75];
            current_axes.XTick = [current_axes.XLim(1):5:current_axes.XLim(2)];
        
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
%                 h1 = plot(data.uavRecorder.est.time , data.vicon.est_yaw.spline_error, 'Color', [0,0,0], ...
                h1 = plot(metrics.uav.time, metrics.uav.yaw.err, 'Color', [0,0,0], ...
                    'LineWidth', linewidth, 'displayname', 'uav yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end
            try 
                h1 = plot(data.ugvRecorder.est.time , data.vicon.ugvc.splines.yaw.est_err, ':', 'Color', [0.5,0.5,0.5], ...
                    'LineWidth', linewidth, 'displayname', 'ugv yaw error'); ii = ii + 1; legend_str{ii} = h1.DisplayName;
            catch; end    
        hold off; clear h1 ii
        grid on
        try columnlegend(2,legend_str); catch; end; clear legend_str
%         legend('toggle')
        ylabel('degrees')
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
            current_axes.XLim = [floor(data.ugvRecorder.est.time(1)) ceil(data.ugvRecorder.est.time(end))];
            current_axes.XTick = [current_axes.XLim(1):5:current_axes.XLim(2)];
            current_axes.YLim = [-60 60];
            current_axes.YTick = [current_axes.YLim(1):20:current_axes.YLim(2)];
        catch; end
        try 
            hold on
            line([data.ugvRecorder.ckf.time(1) data.ugvRecorder.ckf.time(1)],[current_axes.YLim(1) current_axes.YLim(2)-20]);
            line([data.ugvRecorder.ckf.time(end) data.ugvRecorder.ckf.time(end)],[current_axes.YLim(1) current_axes.YLim(2)-20]);
            hold off
        catch; end


%         hold on
%             for m = 1: length(ugv_move_index)
%                 try line([data.ugvRecorder.ckf.time(ugv_move_index(m)) data.ugvRecorder.ckf.time(ugv_move_index(m))], ...
%                         [current_axes.YLim(1) current_axes.YLim(2)-2]);
%                 catch; end
%             end; clear m
%         hold off
%         clear linewidth ugv_move_index t_offset
                
        set(0, 'DefaultAxesFontSize', 20)
        set(0, 'DefaultTextFontSize', 20)
%% figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(4,1,1)
        hold on
%             try plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,1), 'rd', 'displayname', 'UAV x goal'); catch; end
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,1),'k.', 'displayname', 'UGV x actual'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'b.', 'displayname', 'UGV x estimated'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,1), 'gd', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end

            
        hold off
        ylabel('x [m]')
%         try 
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [10 20 30 40 50 60];
%         current_axes.XLim = [0 data.uavRecorder.est.time(end)];
%         current_axes.YTick = [0:2:6];
%         current_axes.YLim = [0 6];
%         clear current_axes current_limits
%         end

    subplot(4,1,2)
        hold on
%             try plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,2), 'rd', 'displayname', 'UAV y goal'); catch; end
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,2),'k.', 'displayname', 'UGV y actual'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'b.', 'displayname', 'UGV y estimated'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,2), 'gd', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end

            
        hold off
        ylabel('y [m]')
%         try 
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [10 20 30 40 50 60];
%         current_axes.XLim = [0 data.uavRecorder.est.time(end)];
%         current_axes.YTick = [-1 0 1];
%         current_axes.YLim = [-1 1];
%         clear current_axes current_limits
%         end

    subplot(4,1,3)
        hold on
%             try plot(data.experiment.uav.FlyTime, data.experiment.uav.DesiredState_gl(:,3), 'rd', 'displayname', 'UAV yaw goal'); catch; end
%             try plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,3), 'rd', 'displayname', 'UAV z goal'); catch; end
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,3),'k.', 'displayname', 'UGV z actual'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'UGV z estimated'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,3), 'gd', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end

        hold off
        ylabel('z [m]')
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [10 20 30 40 50 60];
%         current_axes.XLim = [0 data.uavRecorder.est.time(end)];
%         current_axes.YTick = [0:2];
%         current_axes.YLim = [0 2];
%         clear current_axes current_limits
%         end

    subplot(4,1,4)
        hold on
%             try plot(data.experiment.uav.FlyTime, data.experiment.uav.Waypoint(:,4), 'rd', 'displayname', 'UAV yaw goal'); catch; end
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.yaw.global*180/pi,'k.', 'displayname', 'UAV yaw actual'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'b.', 'displayname', 'UGV yaw estimated'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Yaw, 'gd', ...
                    'MarkerFaceColor', 'k', 'displayname', 'ckf estimate'); catch; end
        hold off
        ylabel('heading [deg]')
%         try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [10 20 30 40 50 60];
%         current_axes.XLim = [0 data.uavRecorder.est.time(end)];
%         current_axes.YTick = [-90:90:90];
%         current_axes.YLim = [-90 90];
%         clear current_axes current_limits
%         xlabel('time [s]')
%         end
%% figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    subplot(3,1,1)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.x_axis(:,1), 'ro', 'displayname', 'stereo-uav-x-axis(1)'); 
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    subplot(3,1,2)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.x_axis(:,2), 'bo', 'displayname', 'stereo-uav-x-axis(2)'); %catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    subplot(3,1,3)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.x_axis(:,3), 'go', 'displayname', 'stereo-uav-x-axis(3)'); %catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    hold off
    ylabel('degrees')
    xlabel('time [s]') 
%% figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    subplot(3,1,1)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.y_axis(:,1), 'ro', 'displayname', 'stereo-uav-y-axis(1)'); %catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    subplot(3,1,2)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.y_axis(:,2), 'bo', 'displayname', 'stereo-uav-y-axis(2)'); %catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    subplot(3,1,3)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.y_axis(:,3), 'go', 'displayname', 'stereo-uav-y-axis(3)'); %catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    hold off
    ylabel('degrees')
    xlabel('time [s]')
%% figure(11); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(11); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    subplot(3,1,1)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.z_axis(:,1), 'ro', 'displayname', 'stereo-uav-z-axis(1)');% catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    subplot(3,1,2)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.z_axis(:,2), 'bo', 'displayname', 'stereo-uav-z-axis(2)');% catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    subplot(3,1,3)
        try plot(data.ugvStereo.time, data.ugvStereo.uav.z_axis(:,3), 'go', 'displayname', 'stereo-uav-z-axis(3)');% catch; end
        grid on
        legend('toggle')
        axis([data.ugvStereo.time(1) data.ugvStereo.time(end) -1 1]); catch; end
    hold off
    ylabel('degrees')
    xlabel('time [s]')
%% figure(12); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(12); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on
            try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.x_diff_stereotime, 'o', 'displayname', 'uav x diff btw stereo and vicon'); catch; end    
            try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.y_diff_stereotime, 'o', 'displayname', 'uav y diff btw stereo and vicon'); catch; end    
            try plot(data.ugvStereo.time, data.vicon.uav.splines.P.global.z_diff_stereotime, 'o', 'displayname', 'uav z diff btw stereo and vicon'); catch; end   
        hold off
        grid on
        legend('toggle')
        legend('location', 'southeast')
        title(['stereo errors of UAV (ugv frame) ' meta.date meta.run])
        ylabel('linear difference [m]')

%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XTick = [0 10 20 30 40 50 60];
%             current_axes.XLim = [data.ugvStereo.time(1) data.ugvStereo.time(end)];
%             current_axes.YTick = [-0.5:0.05:0.5];
%             current_axes.YLim = [-0.2 0.2];
%             clear current_axes current_limits
%         end
    subplot(2,1,2)
        hold on
            try plot(data.ugvRecorder.stereo.time, data.vicon.diff_yaw.spline_error, 'o-', 'displayname', 'uav yaw diff btw stereo and vicon'); catch; end    
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
%% figure(13); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(13); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(['uav yaw and command ' [meta.date meta.run]])
%     hold on
%         try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees,'k.', 'displayname', 'uav yaw vicon'); catch; end        
%         try plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, 'bo', ...
%                 'MarkerSize', 5, 'displayname', 'uav yaw ref trajectory'); catch; end
%         try plot(data.uavCon.time, data.uavCon.Desired.Yaw, 'r.', 'displayname', 'global desired yaw'); catch; end
%         try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.yaw_ugv, 'go', 'displayname', 'uav stereo yaw global'); catch; end
%     hold off
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
        ylabel('degrees')
        try %current axis
        current_limits = axis;
        current_axes = gca; % current axes

        current_axes.XLim = [7 17];
        current_axes.XTick = [current_axes.XLim(1):2:current_axes.XLim(2)];
%         current_axes.YTick = [-90:90:90];
%         current_axes.YLim = [-90 90];
        clear current_axes current_limits
        xlabel('time [s]')
        end
    grid on
    
    
    set(0, 'DefaultAxesFontSize', 20)
    set(0, 'DefaultTextFontSize', 20)

%     axis([data.vicon.uav.time(1) data.vicon.uav.time(end) 0 12])
% axis([0 10 -150 150])
%         saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
% meta.saveplotroot = [meta.dataroot 'notable/' meta.date];
%         saveas(gcf, [meta.saveplotroot 'yaw_vs_time_022_exp.fig']); 
%         print('-depsc', [meta.saveplotroot 'yaw_vs_time_022_exp.eps']); 
%% figure(14); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(14); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        plot(data.uavRecorder.est.time, data.vicon.est_yaw.spline_error, 'o', 'displayname', 'uav yaw error')
        
        
        if 0
            %%
            uav_yaw_err = data.vicon.est_yaw.spline_error;
            uav_yaw_err_time = data.uavRecorder.est.time;
            %remove transients
            uav_yaw_err_time(uav_yaw_err<-9) = [];
            uav_yaw_err(uav_yaw_err<-9) = [];
            uav_yaw_mean_err = mean(uav_yaw_err)

        end; 
        
        
        try plot(uav_yaw_err_time, uav_yaw_err, 'x', 'displayname', 'uav yaw error'); catch; end
        clear uav_yaw_err uav_yaw_mean_err
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
    title(['uav yaw ugv frame ' [meta.date meta.run]])    
    ylabel('degrees')
    hold on
        try plot(data.ugvStereo.time, data.ugvStereo.uav.yaw_ugv, 'o', 'displayname', 'ugvStereo'); catch; end
        try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'b.', 'displayname', 'UGV yaw estimated'); catch; end
        try plot(data.ugvRecorder.veltwist.time, data.ugvRecorder.veltwist.Yawrate, 'rx', 'displayname', 'veltwist-yawrate'); catch; end
        try plot(data.ugvRecorder.veltwist.time, data.ugvRecorder.veltwist.Vdt, 'mo', 'displayname', 'veltwist-Vdt'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title(['uav yaw global frame ' [meta.date meta.run]])    
    ylabel('degrees')
    xlabel('range along optical axis, [m]')
    

    
    hold on
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
        
    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end    
%% figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['ugv vo position drift' [meta.date meta.run]])
    hold on
        try plot(data.vo2_odom.header.stamp, data.vo2_odom.pose.position.x, 'r.', 'displayname', 'ugv drift x'); catch; end
        try plot(data.vo2_odom.header.stamp, data.vo2_odom.pose.position.y, 'b.', 'displayname', 'ugv drift y'); catch; end
        try plot(data.vo2_odom.header.stamp, data.vo2_odom.pose.position.z, 'k.', 'displayname', 'ugv drift z'); catch; end
    hold off
    ylabel('drift [m]')
    xlabel('time [s]')
    grid on
    legend('toggle')
%% figure(19); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(19); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['ugv uav april positions in vicon frame' [meta.date meta.run]])
    hold on
        try plot(data.vicon.april_02_.P.vicon(:,1), data.vicon.april_02_.P.vicon(:,2), 'ko', 'displayname', 'april02'); catch; end
        try plot(data.vicon.april_03_.P.vicon(:,1), data.vicon.april_03_.P.vicon(:,2), 'ko', 'displayname', 'april03'); catch; end
        try plot(data.vicon.red_.P.vicon(:,1), data.vicon.red_.P.vicon(:,2), 'ro', 'displayname', 'red'); catch; end
        try plot(data.vicon.blue_.P.vicon(:,1), data.vicon.blue_.P.vicon(:,2), 'bo', 'displayname', 'blue'); catch; end
        try plot(data.vicon.green_.P.vicon(:,1), data.vicon.green_.P.vicon(:,2), 'go', 'displayname', 'green'); catch; end
        try plot(data.vicon.ugvc.P.vicon(:,1), data.vicon.ugvc.P.vicon(:,2), 'k.', 'displayname', 'ugvc'); catch; end
        
        
        
    hold off
    ylabel('y [m]')
    xlabel('x [m]')
    grid on
    legend('toggle')
    
%% figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(20); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(2,1,1)
        hold on
            try plot(data.uavRecorder.est.time, data.vicon.uav.splines.P.global.x_diff, 'o', 'displayname', 'uav x estimate error'); catch; end    
            try plot(data.uavRecorder.est.time, data.vicon.uav.splines.P.global.y_diff, 'o', 'displayname', 'uav y estimate error'); catch; end    
            try plot(data.uavRecorder.est.time, data.vicon.uav.splines.P.global.z_diff, 'o', 'displayname', 'uav z estimate error'); catch; end   
            try plot(data.ugvRecorder.est.time, data.vicon.ugvc.splines.P.global.x_diff, '.', 'displayname', 'ugv x estimate error'); catch; end    
            try plot(data.ugvRecorder.est.time, data.vicon.ugvc.splines.P.global.y_diff, '.', 'displayname', 'ugv y estimate error'); catch; end    
            try 
                plot(data.ugvRecorder.est.time, data.vicon.ugvc.splines.P.global.z_diff, '.', 'displayname', 'ugv z estimate error'); 
            catch; end   
        hold off
        grid on
        legend('toggle')
        legend('location', 'southeast')
        title(['stereo errors of UAV (ugv frame) ' meta.date meta.run])
        ylabel('linear difference [m]')

        try
            current_limits = axis;
            current_axes = gca; % current axes
            current_axes.XLim = [0 data.ugvStereo.time(end)];
            current_axes.YLim = [-1 1];
            current_axes.XTick = [20:10:10*ceil(data.ugvStereo.time(end)/10)];
            
            current_axes.YTick = [-0.5:0.5:0.5];
            
            clear current_axes current_limits
        end
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
%% disp('uav command gains')
disp('uav command gains')
%% figure(24); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(24); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
    title('uav command gains x global')
    hold on
        try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,1), '.', 'displayname', 'uav current p global'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,1), '.', 'displayname', 'uav desired p global'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,1), '.', 'displayname', 'uav cmd global'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(25); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(25); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav command gains x local')
    hold on
        try plot(data.uavCon.time, data.uavCon.Current.Position_dr(:,1), '.', 'displayname', 'uav current p local'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_dr(:,1), '.', 'displayname', 'uav desired p local'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,1), '.', 'displayname', 'uav cmd local'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(26); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(26); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
    title('uav command gains y global')
    hold on
        try plot(data.uavCon.time, data.uavCon.Current.Position_g(:,2), '.', 'displayname', 'uav current p global'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_g(:,2), '.', 'displayname', 'uav desired p global'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_g(:,2), '.', 'displayname', 'uav cmd global'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(27); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav command gains y local')
    hold on
        try plot(data.uavCon.time, data.uavCon.Current.Position_dr(:,2), '.', 'displayname', 'uav current p local'); catch; end
        try plot(data.uavCon.time, data.uavCon.Desired.Position_dr(:,2), '.', 'displayname', 'uav desired p local'); catch; end
        try plot(data.uavCon.time, data.uavCon.cmd.linear_dr(:,2), '.', 'displayname', 'uav cmd local'); catch; end
    hold off
    grid on
    legend('toggle')
%% disp('uav ckf debug plots')
disp('uav ckf debug plots')
%% figure(29); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

figure(29); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uavRecorder.ckf.zk x y z')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.zk(:,1), 'displayname', 'zk x'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.zk(:,2), 'displayname', 'zk y'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.zk(:,3), 'displayname', 'zk z'); catch; end
    hold off
    legend('toggle')
    grid on
%% figure(30); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(30); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uavRecorder.ckf.zk yaw yawbias')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.zk(:,4), 'displayname', 'zk yaw'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.zk(:,5), 'displayname', 'zk yaw bias'); catch; end
    hold off
    legend('toggle')
    grid on
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end
%% figure(31); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(31); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Yaw Bias')
    hold on
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.YawBias, 'displayname', 'EstYawBias'); catch; end
        try plot(data.uavRecorder.ckf.time, data.uavRecorder.ckf.PosteriorEst(:,5), 'displayname', 'PosteriorEst5'); catch; end
    hold off
    grid on
    legend('toggle')
%% figure(32); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(32); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('uav velocities')
    hold on
        try plot(data.uavRecorder.navdata.time, 0.001*uavRecorder.navdata.V); catch; end
    hold off
    grid on
    legend('toggle')
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
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
        try plot(data.ugvStereo.time, data.vicon.diff_yaw.spline_error, 'k', 'displayname', 'vicon/stereo yaw difference'); catch; end
    yyaxis right
    hold on
        try plot(data.ugvStereo.time, legs(:,1), 'displayname', 'RG leg'); catch; end
        try plot(data.ugvStereo.time, legs(:,2), 'displayname', 'GB leg'); catch; end
        try plot(data.ugvStereo.time, legs(:,3), 'displayname', 'BR leg'); catch; end
    hold off
        
    grid on
    legend('toggle')
    current_axes = axis;
    try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
    try
        X = [ugvRecorder.ckf.time(1) * ones(2,1), data.ugvRecorder.ckf.time(end) * ones(2,1)];
        Y = [current_axes(3) * ones(1,2); current_axes(4) * ones(1,2)]; 
        line(X,Y)
    catch
        warning('ugvRecorder.ckf.time missing..?')
    end
    clear current_axes X Y
%% figure(34); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(34); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
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
%% figure(35); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(35); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
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
%% disp('ugv odometry figures')
disp('ugv odometry figures')
% if exist('ugv.ckf')
%% figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

figure(50); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['Estimated UGV Position XY' [meta.date meta.run]])
    hold on
    try plot(data.vicon.ugvk.P.global(:,1), data.vicon.ugvk.P.global(:,2), '.', 'displayname', 'ugv vicon global y') ; catch; end
    try plot(data.vicon.ugvc.P.global(:,1), data.vicon.ugvc.P.global(:,2), '.', 'displayname', 'ugv vicon global y') ; catch; end
    try plot(data.uavRecorder.wheel.P_odom(:,1), data.ugvRecorder.wheel.P_odom(:,2), 'r','LineWidth',4, 'displayname', 'ugv wheel odom y'); catch; end
%     try plot(stateRecorder.PosLinOdom_g(:,1) - stateRecorder.PosLinOdom_g(1,1), stateRecorder.PosLinOdom_g(:,2) - stateRecorder.PosLinOdom_g(1,2), 'ko-', 'displayname', 'Stereopsis'); catch; end
    try plot(data.ugvRecorder.ckf.Position_gl(:,1), data.ugvRecorder.ckf.Position_gl(:,2),'o', 'displayname', 'ugv ckf global y'); catch; end
    try plot(data.ugvRecorder.est.Position_gl(:,1), data.ugvRecorder.est.Position_gl(:,2),'s', 'displayname', 'ugv est global y'); catch; end

    hold on
%         try plot(data.ugvRecorder.est.Position_gl(:,1), data.ugvRecorder.est.Position_gl(:,2), 'bx', 'displayname', 'ugv estimated position xy'); catch; end
%         try plot(data.vicon.ugvc.P.global(:,1), data.vicon.ugvc.P.global(:,2), 'ko', 'displayname', 'ugvc'); catch; end
    hold off

        
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
%     axis([-0.5 1.5 -1 1])
%     axis([-0.25 1 -0.1 0.1])
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end
%% figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(51); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
        title(['Estimated Position (x vs time)' [meta.date meta.run]])
            try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'Global Vicon Position'); catch; end
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,1), 'd-', 'displayname', 'Global Vicon Position'); catch; end
            try plot(data.ugvRecorder.wheel.time, data.ugvRecorder.wheel.P_odom(:,1), '*-', 'displayname', 'Global Wheel Odometry'); catch; end    
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 's', 'displayname', 'ugv x est'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Position_gl(:,1), 'x', 'displayname', 'ugv x ckf'); catch; end
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

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
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
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 's', 'displayname', 'ugv yaw est'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Yaw, 'x', 'displayname', 'ugv yaw ckf'); catch; end
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
        title('ugv Mech and Aiding, x')
        hold on
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Mech(:,1), 's-', 'displayname', 'ugv mech x'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Aiding(:,1), 'o-', 'displayname', 'ugv aid x'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), 'd-', 'displayname', 'ugv est x'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorEst(:,1), 's-', 'displayname', 'ugv PosteriorEst x'); catch; end
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
%% figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(56); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv Mech and Aiding, y')
        hold on
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Mech(:,2), 's-', 'displayname', 'ugv mech y'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Aiding(:,2), 'o-', 'displayname', 'ugv aid y'); catch; end
            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,2), 'd-', 'displayname', 'ugv est y'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorEst(:,2), 's-', 'displayname', 'ugv PosteriorEst y'); catch; end
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
%% figure(59); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(59); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv Mech and Aiding, yaw')
        hold on
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Mech(:,4), 's-', 'displayname', 'ugv mech yaw'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Aiding(:,4), 'x-', 'displayname', 'ugv aid yaw'); catch; end

            try plot(data.ugvRecorder.est.time, data.ugvRecorder.est.Yaw, 'd-', 'displayname', 'ugv est yaw'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorEst(:,4), 's-', 'displayname', 'ugv PosteriorEst yaw'); catch; end

            try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.yaw_ugv-mean(ugvRecorder.stereo.yaw_ugv), 'o', 'displayname', 'stereo-yaw-ugv'); catch; end
            try plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw-mean(ugvRecorder.stereo.yaw_ugv), 's', 'displayname', 'uav est yaw'); catch; end
        hold off
        grid on
        legend('toggle')
        current_axes = axis;
    %     try axis([ugvRecorder.ckf.time(1)-1.5 ugvRecorder.ckf.time(end)+1.5 -3 4]); catch; end
    %     try axis([ugvRecorder.ckf.time(1)-1 ugvRecorder.ckf.time(end)+1 current_axes(3) current_axes(4)]); catch; end
        clear current_axes

        if 0
            %%
            current_fig = gcf;
            meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
            saveas(gcf, [meta.figpath meta.savefilename '.png']); 
            clear current_fig;
        end
%% figure(60); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(60); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('est yaw of auv')
        hold on
            try plot(data.ugvRecorder.stereo.time, data.ugvRecorder.stereo.yaw_ugv, 'o', 'displayname', 'stereo yaw ugv2uav'); catch; end
            try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.degrees, 'kx', 'displayname', 'uav vicon global'); catch; end
            try plot(data.uavRecorder.est.time, data.uavRecorder.est.Yaw, 's', 'displayname', 'uav est yaw global'); catch ;end

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
        yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Rk_re(:,1), 'o-', 'displayname', 'Rk (1,1)'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorCov_re(:,1), 'x-.', 'displayname', 'PosteriorCov(1,1)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Qdk_re(:,1), 's-', 'displayname', 'Qdk(1,1)'); catch; end
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
%% figure(62); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(62); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on
        yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Rk_re(:,6), 'o-', 'displayname', 'Rk (2,2)'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorCov_re(:,6), 'x-.', 'displayname', 'PosteriorCov(2,2)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Qdk_re(:,6), 's-', 'displayname', 'Qdk(2,2)'); catch; end
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
%% figure(63); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(63); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on
        yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Rk_re(:,11), 'o-', 'displayname', 'Rk (3,3)'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorCov_re(:,11), 'x-.', 'displayname', 'PosteriorCov(3,3)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Qdk_re(:,11), 's-', 'displayname', 'Qdk(3,3)'); catch; end   
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
%% figure(64); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(64); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv ckf qw and rk')
        hold on
        yyaxis left; axleft = gca; ylabel(axleft, 'Rk'); %ylim(axleft, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Rk_re(:,16), 'o-', 'displayname', 'Rk (4,4)'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.PosteriorCov_re(:,16), 'x-.', 'displayname', 'PosteriorCov(4,4)'); catch; end
        yyaxis right; axright = gca; ylabel(axright, 'Qdk'); %ylim(axright, [-0.5 0.5]);
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.Qdk_re(:,16), 's-', 'displayname', 'Qdk(4,4)'); catch; end
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
%% figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(66); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv obs and est of uav in global')
        hold on
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.obs_uav_in_global(:,1), 's-', 'displayname', 'ugv obs of uav x in global'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.p_est_uav_in_global(:,1), 'o-', 'displayname', 'ugv est of uav x in global'); catch; end
            try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,1), 'b.', 'displayname', 'uav ckf est x'); catch; end
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
%% figure(67); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(67); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv obs and est of uav in global')
        hold on
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.obs_uav_in_global(:,2), 's-', 'displayname', 'ugv obs of uav y in global'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.p_est_uav_in_global(:,2), 'o-', 'displayname', 'ugv est of uav y in global'); catch; end
            try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,2), 'b.', 'displayname', 'uav est y global'); catch; end
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
%% figure(68); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(68); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title('ugv obs and est of uav in global')
        hold on
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.obs_uav_in_global(:,3), 's-', 'displayname', 'ugv obs of uav z in global'); catch; end
            try plot(data.ugvRecorder.ckf.time, data.ugvRecorder.ckf.p_est_uav_in_global(:,3), 'o-', 'displayname', 'ugv est of uav z in global'); catch; end
            try plot(data.uavRecorder.est.time, data.uavRecorder.est.Position_gl(:,3), 'b.', 'displayname', 'uav est z global'); catch; end
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
%% figure(70); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(70); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        subplot(4,1,1)
        title(['ugv motion estimates ' meta.date meta.run])
            hold on
                try plot(data.vicon.ugvk.time, data.vicon.ugvk.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
                try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
                try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
                try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
                try plot(data.vicon.ugvc.time, data.vicon.ugvc.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,1), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,2), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.P.global(:,3), 'd-', 'displayname', 'ugv vicon state'); catch; end
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
            try plot(data.vicon.ugvc.time, data.vicon.ugvc.yaw.global*180/pi, 'd-', 'displayname', 'ugv vicon state'); catch; end
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
%% disp('RGB pixels and marker locations')
disp('RGB pixels and marker locations')
%% figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(100); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['top down agent locations '  [meta.date meta.run]])    
    hold on
    try h1 = plot(0,0, 'k*', 'displayname', 'vicon-origin'); catch; end
    try h2 = plot(data.vicon.ugvk.P.vicon(:,1), data.vicon.ugvk.P.vicon(:,2), 'ms', 'displayname', 'ugv-vicon'); catch; end
    try h3 = plot(data.vicon.red_target.P.vicon(:,1), data.vicon.red_target.P.vicon(:,2), 'ro', 'displayname', 'red-vicon-xy'); catch; end
    try h4 = plot(data.vicon.blue_target.P.vicon(:,1), data.vicon.blue_target.P.vicon(:,2), 'bo', 'displayname', 'red-vicon-xy'); catch; end
    try h5 = plot(data.vicon.green_target.P.vicon(:,1), data.vicon.green_target.P.vicon(:,2), 'go', 'displayname', 'red-vicon-xy'); catch; end
    
    try % initial pointing vector
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
    try % initial pointing vector
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
    
    
%     disp(['mean error between x pixels [left rbg] :'])
%     disp([...
%         mean(data.vicon.red_.splines.left.xy(:,1) - data.ugvStereo.Red.left.xy(:,1)) ...
%         mean(data.vicon.blue_.splines.left.xy(:,1) - data.ugvStereo.Blue.left.xy(:,1)) ...
%         mean(data.vicon.green_.splines.left.xy(:,1) - data.ugvStereo.Green.left.xy(:,1)) ])
%     disp(['mean error between y pixels [left rbg] :'])
    
    hold off
    grid on
    title('left x pixels [px]')
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
    title('right x pixels [px]')

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

    
        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end
%% figure(103); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(103); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

    subplot(2,1,1)
    hold on
    try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,1) - data.ugvStereo.Red.right.xy(:,1), 'ro', 'displayname', 'red-disp-x'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,1) - data.ugvStereo.Blue.right.xy(:,1), 'bo', 'displayname', 'blue-disp-x'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,1) - data.ugvStereo.Green.right.xy(:,1), 'go', 'displayname', 'green-disp-x'); catch; end
    hold off
    grid on
    title('disparity [px]')

    subplot(2,1,2)
    hold on
    try  plot(data.ugvStereo.time, data.ugvStereo.Red.left.xy(:,2), 'ro', 'displayname', 'red-pixels-y'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Blue.left.xy(:,2), 'bo', 'displayname', 'blue-pixels-y'); catch; end
    try  plot(data.ugvStereo.time, data.ugvStereo.Green.left.xy(:,2), 'go', 'displayname', 'green-pixels-y'); catch; end
    hold off
    grid on

        
    if 0
        %%
        current_fig = gcf;
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')'];
        saveas(gcf, [meta.figpath meta.savefilename '.png']); 
        clear current_fig;
    end  
%% figure(104); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(104); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,1,1)
        hold on
%             try plot(data.vicon.red_target.time, data.vicon.red_target.P.cam(:,1), 'r.', 'displayname', 'red-vicon-x'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,1), 'r+', 'displayname', 'red-stereo-x-ugv'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.uav.yaw_ugv, 'go', 'displayname', 'uav stereo yaw ugv frame'); catch; end

        hold off
%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%             current_axes.XLim = [0 data.ugvStereo.time(end)];
%             current_axes.YLim = [-1.25 1.25];
%             current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%             clear current_axes current_limits
%         end
        grid on
        title(['vicon marker positions (x) in ugv frame' [meta.date meta.run]])
        ylabel('x [m]')
        legend('toggle')
    subplot(3,1,2)
        hold on
            try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.cam(:,1), 'b.', 'displayname', 'blue-vicon-x'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_ugv(:,1), 'b+', 'displayname', 'blue-stereo-x-ugv'); catch; end
        hold off
        try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
        end
        ylabel('x [m]')
        grid on
        legend('toggle')
    subplot(3,1,3)
        hold on
            try plot(data.vicon.green_target.time, data.vicon.green_target.P.cam(:,1), 'g.', 'displayname', 'green-vicon-x'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.Green.P_ugv(:,1), 'g+', 'displayname', 'green-stereo-x-ugv'); catch; end
        hold off
        try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
        end
        ylabel('x [m]')
        grid on
        legend('toggle')
%% figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(105); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,1,1)
        hold on
%             try plot(data.vicon.red_target.time, data.vicon.red_target.P.cam(:,1), 'r.', 'displayname', 'red-vicon-x'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.Red.P_ugv(:,2), 'r+', 'displayname', 'red-stereo-y-ugv'); catch; end
%             try plot(data.ugvStereo.time, data.ugvStereo.uav.yaw_ugv, 'go', 'displayname', 'uav stereo yaw ugv frame'); catch; end

        hold off
%         try
%             current_limits = axis;
%             current_axes = gca; % current axes
%             current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%             current_axes.XLim = [0 data.ugvStereo.time(end)];
%             current_axes.YLim = [-1.25 1.25];
%             current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%             clear current_axes current_limits
%         end
        grid on
        title(['vicon marker positions (x) in ugv frame' [meta.date meta.run]])
        ylabel('y [m]')
        legend('toggle')
    subplot(3,1,2)
        hold on
%             try plot(data.vicon.blue_target.time, data.vicon.blue_target.P.cam(:,1), 'b.', 'displayname', 'blue-vicon-x'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.Blue.P_ugv(:,2), 'b+', 'displayname', 'blue-stereo-y-ugv'); catch; end
        hold off
        try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
        end
        ylabel('y [m]')
        grid on
        legend('toggle')
    subplot(3,1,3)
        hold on
%             try plot(data.vicon.green_target.time, data.vicon.green_target.P.cam(:,1), 'g.', 'displayname', 'green-vicon-x'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.Green.P_ugv(:,2), 'g+', 'displayname', 'green-stereo-y-ugv'); catch; end
        hold off
        try
%         current_limits = axis;
%         current_axes = gca; % current axes
%         current_axes.XTick = [0:5:data.ugvStereo.time(end)];
%         current_axes.XLim = [0 data.ugvStereo.time(end)];
%         current_axes.YLim = [-1.25 1.25];
%         current_axes.YTick = [current_axes.YLim(1):0.5:current_axes.YLim(2)];
%         clear current_axes current_limits
        end
        ylabel('y [m]')
        grid on
        legend('toggle')
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
%% figure(109); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(109); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
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
%% figure(110); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(110); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
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
%% figure(111); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(111); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
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
%% figure(112); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(112); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
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


