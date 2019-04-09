function stats = makeErrStatsStereo(vicon, ugvStereo, ugvRecorder, meta, run, experiment)
%% create error structure for statistics
    errtime = ugvStereo.time;
%     yawerr = vicon.diff_yaw.splineyaw;
    yawerr = vicon.diff_yaw.spline_error;
    yawtime = ugvRecorder.stereo.time;
    
    
    
switch meta.frame 
    case 'global'
        vprint('using global frame')
        xerr = vicon.uav.splines.P.global.x_diff_stereotime;
        yerr = vicon.uav.splines.P.global.y_diff_stereotime;
        zerr = vicon.uav.splines.P.global.z_diff_stereotime;
    case {'camera','cam'}
        vprint('using camera frame')
        xerr = vicon.uav.splines.P.cam.x_diff; % difference in stereo time
        yerr = vicon.uav.splines.P.cam.y_diff; % difference in stereo time
        zerr = vicon.uav.splines.P.cam.z_diff; % difference in stereo time
    otherwise
        warning('frame not defined')
        return
end




%% cut the early data
%     lowcut_vector = errtime<experiment.transient.start;
%     xerr(lowcut_vector) = [];
%     yerr(lowcut_vector) = [];
%     zerr(lowcut_vector) = [];
%     yawerr(lowcut_vector) = [];
%     errtime(lowcut_vector) = [];    
%% cut the late data
%     highcut_vector = errtime>experiment.transient.end;
%     xerr(highcut_vector) = [];
%     yerr(highcut_vector) = [];
%     zerr(highcut_vector) = [];
%     yawerr(highcut_vector) = [];
%     errtime(highcut_vector) = [];
%% spline the vicon data to generate range axis    
%     uav_spline.x = csapi(vicon.uav.time,vicon.uav.P.cam(:,1));
%     uav_spline.x_esttime = fnval(uav_spline.x, errtime);
%     uav_spline.y = csapi(vicon.uav.time,vicon.uav.P.cam(:,2));
%     uav_spline.y_esttime = fnval(uav_spline.y, errtime);
    uav_range.z_spline = csapi(vicon.uav.time,vicon.uav.P.cam(:,3));
    uav_range.z = fnval(uav_range.z_spline, errtime);    
    uav_range.z_yaw = fnval(uav_range.z_spline, yawtime);    
%% figure(100x); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(1000+run); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    subplot (2,1,1)
        hold on
            try plot(errtime(abs(xerr) < 1), xerr(abs(xerr) < 1), 'displayname', 'uav x error'); catch; end
            try plot(errtime(abs(yerr) < 1), yerr(abs(yerr) < 1), 'displayname', 'uav y error'); catch; end
            try plot(errtime(abs(zerr) < 0.2), zerr(abs(zerr) < 0.2), 'displayname', 'uav z error'); catch; end
        hold off
        grid on
        legend('toggle')
        xlabel('time [s]')
        ylabel('error [m]')
        title(['uav x y z yaw error vs time ' meta.date(1:end-1)  ' ' meta.run ', ' meta.frame ' frame'])
    subplot (2,1,2)
        try plot(yawtime, yawerr, 'displayname', 'uav yaw error'); catch; end
        legend('toggle')
        xlabel('time [s]')
        ylabel('degrees')
    hold off
    grid on
    if meta.saveplots
        saveas(gcf, [meta.figpath 'errors_v_time/png/' meta.date(1:end-1)  '_' meta.run 'xyzyaw_errors_v_time.png']);  
        saveas(gcf, [meta.figpath 'errors_v_time/fig/' meta.date(1:end-1)  '_' meta.run '_xyzyaw_errors_v_time.fig']);  
        print('-depsc', [meta.figpath 'errors_v_time/eps/' meta.date(1:end-1)  '_' meta.run '_xyzyaw_errors_v_time.eps']); 
    end        
%% figure(200x); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(2000+run); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot (2,1,1)
        hold on
            try plot(uav_range.z, xerr, '.', 'displayname', 'uav x error'); catch; end
            try plot(uav_range.z, yerr, '.', 'displayname', 'uav y error'); catch; end
            try plot(uav_range.z, zerr, '.', 'displayname', 'uav z error'); catch; end
        hold off
        grid on
        legend('toggle')
        xlabel('range [m]')
        ylabel('error [m]')
        title(['uav x y z yaw error vs optical range ' meta.date(1:end-1)  ' ' meta.run ', ' meta.frame ' frame'])
    subplot (2,1,2)
    hold on
        try plot(uav_range.z_yaw, yawerr, '.', 'displayname', 'uav yaw error'); catch; end
        legend('toggle')
        xlabel('range')
        ylabel('yaw error [deg]')
    hold off
    grid on
    if meta.saveplots; 
        saveas(gcf, [meta.figpath 'errors_v_range/png/' meta.date(1:end-1)  '_' meta.run '_xyzyaw_errors_v_range.png']);  
        saveas(gcf, [meta.figpath 'errors_v_range/fig/' meta.date(1:end-1)  '_' meta.run '_xyzyaw_errors_v_range.fig']);  
        print('-depsc', [meta.figpath 'errors_v_range/eps/' meta.date(1:end-1)  '_' meta.run '_xyzyaw_errors_v_range.eps']); 
    end
%% figure(3xxx); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(3100+run); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker error vs optical range ' meta.date(1:end-1)  ' ' meta.run ', ' meta.frame ' frame'])
    hold on
        try plot(uav_range.z, vicon.red_.splines.err_cam(:,1), '.', 'displayname', 'red x error'); catch; end
        try plot(uav_range.z, vicon.blue_.splines.err_cam(:,1), '.', 'displayname', 'blue x error'); catch; end
        try plot(uav_range.z, vicon.green_.splines.err_cam(:,1), '.', 'displayname', 'green x error'); catch; end
    hold off
    grid on
    legend('toggle')
    xlabel('range [m]')
    ylabel('error [m]')
    
figure(3200+run); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker error vs optical range ' meta.date(1:end-1)  ' ' meta.run ', ' meta.frame ' frame'])
    hold on
        try plot(uav_range.z, vicon.red_.splines.err_cam(:,2), '.', 'displayname', 'red y error'); catch; end
        try plot(uav_range.z, vicon.blue_.splines.err_cam(:,2), '.', 'displayname', 'blue y error'); catch; end
        try plot(uav_range.z, vicon.green_.splines.err_cam(:,2), '.', 'displayname', 'green y error'); catch; end
    hold off
    grid on
    legend('toggle')
    xlabel('range [m]')
    ylabel('error [m]')
    
figure(3300+run); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker error vs optical range ' meta.date(1:end-1)  ' ' meta.run ', ' meta.frame ' frame'])
    hold on
        try plot(uav_range.z, vicon.red_.splines.err_cam(:,3), '.', 'displayname', 'red z error'); catch; end
        try plot(uav_range.z, vicon.blue_.splines.err_cam(:,3), '.', 'displayname', 'blue z error'); catch; end
        try plot(uav_range.z, vicon.green_.splines.err_cam(:,3), '.', 'displayname', 'green z error'); catch; end
    hold off
    grid on
    legend('toggle')
    xlabel('range [m]')
    ylabel('error [m]')

    %% out the data
    stats.errtime = errtime;
    stats.xerr = xerr;
    stats.yerr = yerr;
    stats.zerr = zerr;
    stats.yawerr = yawerr;
    stats.yawtime = yawtime;
    stats.uav_range = uav_range.z;
    stats.uav_range_yaw = uav_range.z_yaw;
    try stats.red.p_err = vicon.red_.splines.err_cam; catch; end
    try stats.blue.p_err = vicon.blue_.splines.err_cam; catch; end
    try stats.green.p_err = vicon.green_.splines.err_cam; catch; end
end