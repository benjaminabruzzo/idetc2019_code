% Data used for triangle leg lengths:
% Vicon with RGB markers independent
set(0, 'DefaultFigureVisible', 'off');
setpref('debug', 'verbose', 'false');
disp('stat load class')

% meta.dataroot= '~/git/hast/tex/iros/irosfigs/experiment_data/';
% meta.dataroot = '/media/benjamin/devsdb/hast/data/';
meta.dataroot = '/Users/benjamin/ros/data/';
% meta.dataroot= '~/ros/data/';
meta.saveplots = false;
meta.trim_transients = false;
meta.frame = 'camera';
% meta.frame = 'global';

% meta.daterun = 'all';
% meta.daterun = 'stereo_cal20170203';
% meta.daterun = 'stereo_cal20170212';
meta.daterun = '20170112';
% meta.daterun = '20170912';
% meta.daterun = '20170913';
% meta.daterun = 'altitude_20170914';
% meta.daterun = 'azimuth_20170914';
switch meta.daterun
    case 'altitude_20170914'
        vprint('lab day 20170914');
        labday = '20170914/';
        daterun = {...
            [labday '002']; ...
            [labday '003']; ...
            [labday '004']; ...
            [labday '005']; ...
            [labday '006']; ...
        };
        meta.figroot = '~/ros/data/20170914/figs/';
    case 'azimuth_20170914'
        vprint('lab day 20170914');
        labday = '20170914/';
        daterun = {...
            [labday '007']; ...
            [labday '008']; ...
            [labday '009']; ...
            [labday '010']; ...
            [labday '011']; ...
        };
        meta.figroot = '~/ros/data/20170914/figs/';
    case 'all'
        % use all data
        vprint('using calibration both stereo_cal20170203 and stereo_cal20170212');
        daterun = {...
            '20170205/019';...
            '20170205/047';...

            '20170210/023';...
            '20170210/038';...
            '20170210/042';...
            '20170210/043';...

            '20170212/011';...
            '20170212/019';...
            '20170212/020';...
            };
        meta.figroot = '~/git/hast/tex/iros/irosfigs/stereocal_both/';
    case 'stereo_cal20170203'
        % use calibration 20170203
        vprint('using calibration stereo_cal20170203');
        daterun = {...
            '20170205/019';...
            '20170205/047';...

            '20170210/023';...
            '20170210/038';...
            '20170210/042';...
            '20170210/043';...
            };
        meta.figroot = '~/git/hast/tex/iros/irosfigs/stereo_cal20170203/';
    case 'stereo_cal20170212'
        % use calibration 20170212
        vprint('using calibration stereo_cal20170212');
        daterun = {...
            '20170212/011';...
            '20170212/019';...
            '20170212/020';...
            };
        meta.figroot = '~/git/hast/tex/iros/irosfigs/stereo_cal20170212/';
    case '20170112' % data used for IROS submission
        vprint('lab day 20170112');
        labday = '20170112/';
        daterun = {...
            [labday '020']; [labday '022']; [labday '024']; ...
            [labday '025']; [labday '026']; [labday '027']; ...
            [labday '028']; [labday '029']; [labday '030']; ...
            [labday '031']; [labday '032']; [labday '033']; ...
            [labday '034']; [labday '035']; [labday '036']; ...
            [labday '037']; [labday '038'];  ...
        };
        meta.figroot = '~/ros/data/20170112/figs/';
    case '20170913'
        vprint('lab day 20170913');
        labday = '20170913/';
        daterun = {...
            % Set #1
%             [labday '014']; [labday '015']; [labday '016'];...
%             [labday '017']; [labday '018']; ...
            % Set #2
            [labday '999']; ... % <1.00, this was actually run001 of 20170914
            [labday '033']; [labday '035']; ... % 1.50 2.00
            [labday '037']; [labday '039']; [labday '045']; ... % 2.50 3.00 3.25
            [labday '046']; [labday '047']; [labday '048']; ... % 3.50 3.75 4.00
            [labday '049']; [labday '050']; [labday '051']; ... % 4.25 4.50 4.75

% not using these            
% [labday '031']; ... %1
%             [labday '032']; ... %1.25
%             [labday '034']; ... %1.75
%             [labday '036'];...  % 2 
%             [labday '038']; ... %2.75
%             [labday '040']; ... %3.25
            % [labday '041']; [labday '042'];...
%             [labday '043']; [labday '044']; 
%             [labday '052']; [labday '053']; [labday '054'];... 5 5.25 5.5
%             [labday '055']; [labday '056']; ... %5.75 6
        };
    clear labday
    meta.figroot = '~/ros/data/20170913/figs/';
    otherwise
        return
end

switch meta.frame
    case {'camera', 'cam'}
        meta.figroot = [meta.figroot 'camera/'];
    case 'global'
        meta.figroot = [meta.figroot 'global/'];
    otherwise
        warning('frame not defined')
        return
end

if meta.trim_transients
    meta.figpath = [meta.figroot 'trim_transients/'];
else
    meta.figpath = [meta.figroot 'all_data/'];
end

clear loopout
loopout = struct;
loopout.stats.errtime = [];
loopout.stats.xerr = [];
loopout.stats.yerr = [];
loopout.stats.zerr = [];
loopout.stats.yawerr = [];
loopout.stats.uav_range = [];
loopout.stats.uav_range_yaw = [];
loopout.stats.red.p_err = [];
loopout.stats.blue.p_err = [];
loopout.stats.green.p_err = [];

vprint(datetime);
for run = 1:size(daterun,1)
    datestr = daterun{run};
    meta.date = datestr(1:end-3);
    meta.run = datestr(end-2:end);

    disp(meta.run)

    data = LoadRunData(meta);
try april = data.april;catch;end
try uavCon = data.uavCon;catch;end
try ugvStereo = data.ugvStereo;catch;end
try experiment= data.experiment;catch;end
try uavRecorder = data.uavRecorder;catch;end
try ugvRecorder = data.ugvRecorder;catch;end
try timers = data.timers;catch;end
try vicon = data.vicon;catch;end

        if meta.trim_transients
            stats = makeErrStatsStereoClipped(vicon, ugvStereo, meta, run, experiment);
%             stats = makeErrStatsClipped(vicon, uavRecorder, meta, run, experiment);
        else
%             stats = makeErrStats(vicon, uavRecorder, ugvStereo, meta, run);
            stats = makeErrStatsStereo(vicon, ugvStereo, ugvRecorder, meta, run, experiment);
        end
%     catch
%     end

    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).meta = meta; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).april = april; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).experiment = experiment; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).stats = stats; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).timers = timers; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).uavCon = uavCon; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).uavRecorder = uavRecorder; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).ugvRecorder = ugvRecorder; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).ugvStereo = ugvStereo; catch; end
    try loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).vicon = vicon; catch; end

    loopout.stats.errtime = [loopout.stats.errtime;stats.errtime];
    loopout.stats.xerr = [loopout.stats.xerr;stats.xerr];
    loopout.stats.yerr = [loopout.stats.yerr;stats.yerr];
    loopout.stats.zerr = [loopout.stats.zerr;stats.zerr];
    loopout.stats.yawerr = [loopout.stats.yawerr;stats.yawerr];
    loopout.stats.uav_range = [loopout.stats.uav_range;stats.uav_range];
    loopout.stats.uav_range_yaw = [loopout.stats.uav_range_yaw;stats.uav_range_yaw];
    try loopout.stats.red.p_err = [loopout.stats.red.p_err; stats.red.p_err]; catch; end
    try loopout.stats.blue.p_err = [loopout.stats.blue.p_err; stats.blue.p_err]; catch; end
    try loopout.stats.green.p_err = [loopout.stats.green.p_err; stats.green.p_err]; catch; end
    loopout.range_z(run) = mean(loopout.(matlab.lang.makeValidName(['set_' num2str(run)])).vicon.uav.P.cam(:,3));
    loopout.range_zstr{run} = num2str(loopout.range_z(run));
    
    

clear april timers uavRecorder ugvStereo stats
clear experiment uavCon ugvRecorder vicon

end

%% Turn plotting back on
set(0, 'DefaultFigureVisible', 'on');
figHandles = findall(0, 'Type', 'figure');
set(figHandles(:), 'visible', 'on')
loopout.meta.date = meta.date;

%% figure(700); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(700); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title(['yaw error and pixel propagated uncertanties, ' meta.date(1:end-1)])    
    ylabel('degrees')
    xlabel('range along optical axis, [m]')
    hold on
    
    for i = 1:size(fields(loopout),1)-4
        ordinate1 = mean(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
            ones(size(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error));
        
        ordinate2 = mean(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
            ones(size(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime));
        
        ordinate3 = mean(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
            ones(size(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var));

        try h1 = plot(ordinate1, ...
                loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error, ...
                'bo', 'displayname', 'error of stereo yaw angle measurement'); catch; end
        %for data of 20170112
        if strcmp(meta.daterun, '20170212')
            try h2 = plot(-0.025+ordinate3,180*sqrt(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var)/pi, ...
               'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
            try h3 = plot(-0.025+ordinate3,-180*sqrt(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var)/pi, ...
               'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
        else
            try h2 = plot(0.025+ordinate1, 180*sqrt(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvRecorder.stereo.yaw_cov)/pi, ...
                'rx', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
            try h3 = plot(0.025+ordinate1, -180*sqrt(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvRecorder.stereo.yaw_cov)/pi, ...
                'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
        end
        
        clear ordinate1 ordinate2
        
    end
    hold off
    grid on
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'southwest')
        clear h1 h2 h3
    catch
    end
            
    try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.YLim = [-60 60];
        current_axes.XLim = [0 5];
        current_axes.XTick = [current_axes.XLim(1):1:current_axes.XLim(2)];
        current_axes.YTick = [current_axes.YLim(1):15:current_axes.YLim(2)];
        clear current_axes current_limits
    end

    if meta.saveplots
        current_fig = gcf;
        saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
        clear current_fig;
    end    
%% figure(3000); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3000); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['x error, ' meta.frame ' frame'])
        hold on
            try plot(loopout.stats.uav_range(abs(loopout.stats.xerr)<1), loopout.stats.xerr(abs(loopout.stats.xerr)<1), 'b.', 'displayname', 'uav x error'); catch; end
        hold off
        grid on
        xlabel('range [m]')
        ylabel('x error [m]')
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_x_errors_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_x_errors_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_all_x_errors_v_range.eps']);
    end
%% figure(3001); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3001); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['y error, ' meta.frame ' frame'])
        hold on
%             try plot(loopout.stats.uav_range, loopout.stats.yerr, 'b.', 'displayname', 'uav y error'); catch; end
            try plot(loopout.stats.uav_range(abs(loopout.stats.yerr) < 1), loopout.stats.yerr(abs(loopout.stats.yerr) < 1), 'b.', 'displayname', 'uav y error'); catch; end
        hold off
        grid on
        xlabel('range [m]')
        ylabel('y error [m]')
    if meta.saveplots
        figure(3001);
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_y_errors_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_y_errors_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_all_y_errors_v_range.eps']);
    end
%% figure(3002); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3002); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['z error, ' meta.frame ' frame'])
        hold on
%             try plot(loopout.stats.uav_range, loopout.stats.zerr, 'b.', 'displayname', 'uav z error'); catch; end
            try plot(loopout.stats.uav_range(abs(loopout.stats.zerr) < 1), loopout.stats.zerr(abs(loopout.stats.zerr) < 1), 'b.', 'displayname', 'uav z error'); catch; end
        hold off
        grid on
        xlabel('range [m]')
        ylabel('z error [m]')
    if meta.saveplots
        figure(3002);
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_z_errors_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_z_errors_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_z_errors_v_range.eps']);
    end
%% figure(3003); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3003); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['yaw error, global frame']) % yaw is always global frame
        hold on
            try 
                plot(loopout.stats.uav_range_yaw, loopout.stats.yawerr, 'b.', 'displayname', 'uav yaw error'); 
            catch; end
        hold off
        grid on
        ylabel('yaw error [deg]')
        xlabel('range [m]')    
        axis([1 3.5 -10 10])
    if meta.saveplots
        figure(3003);
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_yaw_errors_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_yaw_errors_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_yaw_errors_v_range.eps']);
    end
%% figure(400); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(400); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['errors in x direction, independent of range or time, ' meta.frame ' frame'])
    hold on
        try 
            x = loopout.stats.xerr;
            x(abs(x)>1) = [];
            hist((x), 100, 'displayname', 'x errors'); 
            clear x
        catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_x_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_x_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_x_hist.eps']);
    end
%% figure(401); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(401); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['absolute errors in x direction, independent of range or time, ' meta.frame ' frame'])
    hold on
        try 
            x = loopout.stats.xerr;
            x(abs(x)>1) = [];
            hist(abs(x), 100, 'displayname', 'abs(x errors)'); 
            clear x
        catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_abs_x_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_abs_x_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_abs_x_hist.eps']);
    end
%% figure(402); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(402); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['errors in y direction, independent of range or time, ' meta.frame ' frame'])
    hold on
        try 
            y = loopout.stats.yerr;
            y(abs(y)>1) = [];
            hist((y), 100, 'displayname', 'y errors'); 
            clear y
        catch; end
%         try hist((loopout.stats.yerr), 100, 'displayname', 'y errors'); catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_y_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_y_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_y_hist.eps']);
    end
%% figure(403); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(403); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['absolute errors in y direction, independent of range or time, ' meta.frame ' frame'])
    hold on
        try 
            x = loopout.stats.yerr;
            x(abs(x)>1) = [];
            hist(abs(x), 100, 'displayname', 'abs(y errors)'); 
            clear x
        catch; end
%         try hist(abs(loopout.stats.yerr), 100, 'displayname', 'abs(y errors)'); catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_abs_y_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_abs_y_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_abs_y_hist.eps']);
    end
%% figure(404); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(404); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['errors in z direction, independent of range or time, ' meta.frame ' frame'])
    hold on
        try 
            x = loopout.stats.zerr;
            x(abs(x)>1) = [];
            hist((x), 100, 'displayname', 'z errors'); 
            clear x
        catch; end
    %         try hist((loopout.stats.zerr), 100, 'displayname', 'z errors'); catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_z_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_z_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_z_hist.eps']);
    end
%% figure(405); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(405); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['absolute errors in z direction, independent of range or time, ' meta.frame ' frame'])
    hold on
        try 
            x = loopout.stats.zerr;
            x(abs(x)>1) = [];
            hist(abs(x), 100, 'displayname', 'abs(z errors)'); 
            clear x
        catch; end
    %         try hist(abs(loopout.stats.zerr), 100, 'displayname', 'abs(z errors)'); catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_abs_z_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_abs_z_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_abs_z_hist.eps']);
    end
%% figure(406); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(406); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('errors in yaw direction, independent of range or time, global frame') %yaw is never in camera frame
    hold on
        try hist(loopout.stats.yawerr(abs(loopout.stats.yawerr)<100), 100, 'displayname', 'yaw errors'); catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_yaw_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_yaw_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_yaw_hist.eps']);
    end
%% figure(407); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(407); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('absolute errors in yaw direction, independent of range or time, global frame') %yaw is never in camera frame
    hold on
        try hist(abs(loopout.stats.yawerr(abs(loopout.stats.yawerr)<100)), 100, 'displayname', 'abs(yaw errors)'); catch; end
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_abs_yaw_hist.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_abs_yaw_hist.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_abs_yaw_hist.eps']);
    end
%% figure(408); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(408); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('test')
    subplot(2,2,1) % x
        try 
            x = loopout.stats.xerr;
            x(abs(x)>1) = [];
            hist((x), [-0.6:0.01:0.6], 'displayname', 'x errors'); 
            clear x
        catch; end
        current_limits = axis;
        current_axes = gca; % current axes
%         current_axes.XLim = [-max(abs([current_limits(1) current_limits(2)])) max(abs([current_limits(1) current_limits(2)]))];
        current_axes.XLim = [-0.6 0.6];
        current_axes.XTick = [-0.6:0.2:0.6];
        current_axes.YTick = [];
        current_axes.YLim = [0 3000];
        clear current_axes current_limits
        xlabel('x errors, meters')
        
    subplot(2,2,2) % y
        try 
            y = loopout.stats.yerr;
            y(abs(y)>1) = [];
            hist((y), [-0.6:0.01:0.6], 'displayname', 'y errors'); 
            clear y
        catch; end
        current_limits = axis;
        current_axes = gca; % current axes
%         current_axes.XLim = [-max(abs([current_limits(1) current_limits(2)])) max(abs([current_limits(1) current_limits(2)]))];
        current_axes.XLim = [-0.6 0.6];
        current_axes.XTick = [-0.6:0.2:0.6];
        current_axes.YTick = [];
        current_axes.YLim = [0 3000];
        clear current_axes current_limits
        xlabel('y errors, meters')
    subplot(2,2,3) % z
        try 
            x = loopout.stats.zerr;
            x(abs(x)>1) = [];
            hist((x), [-0.6:0.01:0.6], 'displayname', 'z errors'); 
            clear x
        catch; end
        current_limits = axis;
        current_axes = gca; % current axes
%         current_axes.XLim = [-max(abs([current_limits(1) current_limits(2)])) max(abs([current_limits(1) current_limits(2)]))];
        current_axes.XLim = [-0.6 0.6];
        current_axes.XTick = [-0.6:0.2:0.6];
        current_axes.YTick = [];
        current_axes.YLim = [0 3000];
        clear current_axes current_limits
        xlabel('z errors, meters')
    subplot(2,2,4) % yaw
        try hist(loopout.stats.yawerr(abs(loopout.stats.yawerr)<100), [-135:2:135], 'displayname', 'yaw errors'); catch; end
        current_limits = axis;
        current_axes = gca; % current axes
%         current_axes.XLim = [-max(abs([current_limits(1) current_limits(2)])) max(abs([current_limits(1) current_limits(2)]))];
        current_axes.XLim = [-135 135];
        current_axes.XTick = [-135:45:135];
        current_axes.YTick = [];
        current_axes.YLim = [0 800];
        clear current_axes current_limits
        xlabel('yaw errors, degrees')
    if 0
%     if meta.saveirosplots
            %%
            current_fig = gcf;
            meta.saveplotroot = ['/media/benjamin/devsdb/hast/tex/iros2017'];
            meta.savefilename = ['figure_18'];

            try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
            try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
            try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
            clear current_fig;
    end
%% figure(4002); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(4002); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['errors in x direction vs range, ' meta.frame ' frame'])
    hold on
        x = [loopout.stats.uav_range loopout.stats.xerr];
        x(abs(x(:,2))>1,:) = [];
        try hist3([x], [50 50], 'displayname', 'x error'); catch; end
%         try hist3([loopout.stats.uav_range loopout.stats.xerr], [50 50], 'displayname', 'x error'); catch; end
        clear x
    hold off
    grid on
    view(3);
    xlabel('range [m]')
    ylabel('error [m]')
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_x_hist_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_x_hist_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_x_hist_v_range.eps']);
    end
%% figure(4003); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(4003); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['errors in y direction vs range, ' meta.frame ' frame'])
    hold on
        y = [loopout.stats.uav_range loopout.stats.yerr];
        y(abs(y(:,2))>1,:) = [];
        try hist3([y], [50 50], 'displayname', 'y error'); catch; end
%         try hist3([x y], [50 50], 'displayname', 'y error'); catch; end
%         try hist3([loopout.stats.uav_range loopout.stats.yerr], [50 50], 'displayname', 'y error'); catch; end
        clear y
    hold off
    grid on
    view(3);
    xlabel('range [m]')
    ylabel('error [m]')
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_y_hist_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_y_hist_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_y_hist_v_range.eps']);
    end
%% figure(4004); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(4004); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['errors in z direction vs range, ' meta.frame ' frame'])
    hold on
        z = [loopout.stats.uav_range loopout.stats.zerr];
        z(abs(z(:,2))>0.2,:) = []; %remove bad outliers
        try hist3([z], [50 50], 'displayname', 'z error'); catch; end
%         try hist3([loopout.stats.uav_range loopout.stats.zerr], [50 50], 'displayname', 'z error'); catch; end
        clear z
    hold off
    grid on
    view(3);
    xlabel('range [m]')
    ylabel('error [m]')
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_z_hist_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_z_hist_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_z_hist_v_range.eps']);
    end
%% figure(4005); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(4005); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('errors in yaw vs range, global frame')% yaw is never in camera frame
    hold on
        try hist3([loopout.stats.uav_range loopout.stats.yawerr], [50 50], 'displayname', 'yaw error'); catch; end
    hold off
    grid on
    view(3);
    xlabel('range [m]')
    ylabel('error [deg]')
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_range/png/all_yaw_hist_v_range.png']);
        saveas(gcf, [meta.figpath 'errors_v_range/fig/all_yaw_hist_v_range.fig']);
        print('-depsc', [meta.figpath 'errors_v_range/eps/all_yaw_hist_v_range.eps']);
    end
%% figure(500x); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5000); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker disparity vs range')
    hold on
        for i = 1:size(fields(loopout),1)-4
            ordinate1 = mean(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
                ones(size(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.Red.left.xy(:,1)));
            d = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.Red.left.xy(:,1) - ...
                    loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.Red.right.xy(:,1);                
                try plot(ordinate1, d, 'r.'); catch; end
            d = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.Blue.left.xy(:,1) - ...
                    loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.Blue.right.xy(:,1);                
                try plot(ordinate1, d, 'b.'); catch; end
            d = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.Green.left.xy(:,1) - ...
                    loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.Green.right.xy(:,1);                
                try plot(ordinate1, d, 'g.'); catch; end
            x = mean(ordinate1);
            y = mean(d);
            txt = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).meta.run;
            text(x,y,txt, 'FontSize', 36)
        end; clear ordinate1 x y txt d

    hold off
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')
%     axis([0 5 -0.5 0.5])

figure(5001); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker x errors in camera frame')
    hold on
        try plot(loopout.stats.uav_range, loopout.stats.red.p_err(:,1), 'r.', 'displayname', 'red x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.blue.p_err(:,1), 'b.', 'displayname', 'blue x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.green.p_err(:,1), 'g.', 'displayname', 'green x error'); catch; end
    hold off
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')
%     axis([0 5 -0.50 0.5])

    
figure(5002); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker y errors in camera frame')
    hold on
        try plot(loopout.stats.uav_range, loopout.stats.red.p_err(:,2), 'r.', 'displayname', 'red x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.blue.p_err(:,2), 'b.', 'displayname', 'blue x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.green.p_err(:,2), 'g.', 'displayname', 'green x error'); catch; end
    hold off
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')
%     axis([0 5 -0.5 0.5])

    
figure(5003); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker z errors in camera frame')
    hold on
        try plot(loopout.stats.uav_range, loopout.stats.red.p_err(:,3), 'r.', 'displayname', 'red x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.blue.p_err(:,3), 'b.', 'displayname', 'blue x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.green.p_err(:,3), 'g.', 'displayname', 'green x error'); catch; end
    hold off
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')
%     axis([0 5 -0.5 0.5])
    
    

figure(5004); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker x errors in camera frame')
    hold on
        try plot(loopout.stats.uav_range, loopout.stats.red.p_err(:,1), 'r.', 'displayname', 'red x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.blue.p_err(:,1), 'b.', 'displayname', 'blue x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.green.p_err(:,1), 'g.', 'displayname', 'green x error'); catch; end
        previous = 1;
        for i = 1:size(fields(loopout),1)-4
            block = previous:(previous + length(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.time)-1);
            x = mean(loopout.stats.uav_range(block));
            y = mean(loopout.stats.red.p_err(block,1));
            txt = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).meta.run;
            text(x,y,txt, 'FontSize', 36)
            previous = previous + length(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.time)-1;
        end; clear i x y txt previous block
    hold off
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')

figure(5005); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker y errors in camera frame')
    hold on
        try plot(loopout.stats.uav_range, loopout.stats.red.p_err(:,2), 'r.', 'displayname', 'red x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.blue.p_err(:,2), 'b.', 'displayname', 'blue x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.green.p_err(:,2), 'g.', 'displayname', 'green x error'); catch; end
        previous = 1;
        for i = 1:size(fields(loopout),1)-4
            block = previous:(previous + length(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.time)-1);
            x = mean(loopout.stats.uav_range(block));
            y = mean(loopout.stats.red.p_err(block,2));
            txt = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).meta.run;
            text(x,y,txt, 'FontSize', 36)
            previous = previous + length(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.time)-1;
        end; clear i x y txt previous block
    hold off
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')
%     axis([0 5 -0.5 0.5])

    
figure(5006); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker z errors in camera frame')
    hold on
        try plot(loopout.stats.uav_range, loopout.stats.red.p_err(:,3), 'r.', 'displayname', 'red x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.blue.p_err(:,3), 'b.', 'displayname', 'blue x error'); catch; end
        try plot(loopout.stats.uav_range, loopout.stats.green.p_err(:,3), 'g.', 'displayname', 'green x error'); catch; end
        previous = 1;
        for i = 1:size(fields(loopout),1)-4
            block = previous:(previous + length(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.time)-1);
            x = mean(loopout.stats.uav_range(block));
            y = mean(loopout.stats.red.p_err(block,3));
            txt = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).meta.run;
            text(x,y,txt, 'FontSize', 36)
            previous = previous + length(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.time)-1;
        end; clear i x y txt previous block

    hold off
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')
%     axis([0 5 -0.5 0.5])    
%%
% for i = 1:size(fields(loopout),1)-4
%    disp(['mean(loopout.' loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).meta.run '.vicon.red_.P.cam)']); disp(mean(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.red_.P.cam)) 
% end
% disp(' ')
% for i = 1:size(fields(loopout),1)-4
%    disp(['mean(loopout.' loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).meta.run '.vicon.blue_.P.cam)']); disp(mean(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.blue_.P.cam)) 
% end
% disp(' ')
% for i = 1:size(fields(loopout),1)-4
%    disp(['mean(loopout.' loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).meta.run '.vicon.green_.P.cam)']); disp(mean(loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.green_.P.cam)) 
% end

%%
figure(5100); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    set(0, 'DefaultAxesFontSize', 32)
    set(0, 'DefaultTextFontSize', 32)
    hold on
        try plot(loopout.stats.uav_range, loopout.stats.zerr, 'k.', 'displayname', 'uav z error'); catch; end

    hold off
    grid on
    xlabel('Range along stereo optical axis [m]')
    ylabel('Camera frame z error [m]')
    axis([0 5 0 0.5])    
    set(0, 'DefaultAxesFontSize', 20)
    set(0, 'DefaultTextFontSize', 20)

figure(5101); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('Marker z errors in camera frame')
    group = [];
    box_data = [];
    for i = 1:size(fields(loopout),1)-4
        add_data = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_diff;
        box_data = [box_data; add_data];
        group = [group; repmat({loopout.range_zstr{i}(1:4)}, size(add_data,1), size(add_data,2))];
    end
    boxplot(box_data, group);
        ylabel('degrees')
%         yxis = gca; ylim([-1 3]); clear yxis
        xlabel('range along optical axis')
        xxis=gca; xxis.XTickLabelRotation = -60; clear xxis
    grid on
    xlabel('Distance between UAV and UGV along stereo optical axis [m]')
    ylabel('Error in marker location (vicon as truth) [m]')


%% figure(6000); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% https://www.mathworks.com/matlabcentral/answers/93785-how-can-i-compute-multiple-boxplots-with-vectors-of-different-length-using-the-statistics-toolbox-4
figure(6000); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    group = [];
    box_data = [];
    for i = 1:size(fields(loopout),1)-4
        add_data = loopout.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error;
        box_data = [box_data; add_data];
        group = [group; repmat({loopout.range_zstr{i}(1:4)}, size(add_data,1), size(add_data,2))];
    end
%     boxplot(data, group);
        ylabel('degrees')
        yxis = gca; ylim([-20 20]); clear yxis
        xlabel('range along optical axis')
        xxis=gca; xxis.XTickLabelRotation = -60; clear xxis

%% From earlier commits:

%% Final clearout

clear daterun datestr run