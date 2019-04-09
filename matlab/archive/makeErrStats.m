function stats = makeErrStats(vicon, uavRecorder, ugvStereo, meta, run)
%% create error structure for statistics?
    xerr = vicon.uav.splines.P.global.x_diff;
    yerr = vicon.uav.splines.P.global.y_diff;
    zerr = vicon.uav.splines.P.global.z_diff;
    yawerr = vicon.est_yaw.spline_error;
    errtime = uavRecorder.est.time;
    
    %% cut the early data
    lowcut_vector = errtime<ugvStereo.time(1);
    xerr(lowcut_vector) = [];
    yerr(lowcut_vector) = [];
    zerr(lowcut_vector) = [];
    yawerr(lowcut_vector) = [];
    errtime(lowcut_vector) = [];    
    
    %% cut the late data
    highcut_vector = errtime>ugvStereo.time(end);
    xerr(highcut_vector) = [];
    yerr(highcut_vector) = [];
    zerr(highcut_vector) = [];
    yawerr(highcut_vector) = [];
    errtime(highcut_vector) = [];

    %% spline the vicon data to generate range axis    
%     uav_spline.x = csapi(vicon.uav.time,vicon.uav.P.cam(:,1));
%     uav_spline.x_esttime = fnval(uav_spline.x, errtime);
%     uav_spline.y = csapi(vicon.uav.time,vicon.uav.P.cam(:,2));
%     uav_spline.y_esttime = fnval(uav_spline.y, errtime);
    uav_range.z_spline = csapi(vicon.uav.time,vicon.uav.P.cam(:,3));
    uav_range.z = fnval(uav_range.z_spline, errtime);    
    
    
%% figure(100x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(1000+run); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    subplot (2,1,1)
        hold on
            try plot(errtime, xerr, 'displayname', 'uav x error'); catch; end
            try plot(errtime, yerr, 'displayname', 'uav y error'); catch; end
            try plot(errtime, zerr, 'displayname', 'uav z error'); catch; end
        hold off
        grid on
        legend('toggle')
        xlabel('time [s]')
        ylabel('error [m]')
        title(['uav x y z yaw error vs time ' meta.date meta.run])
    subplot (2,1,2)
        try plot(errtime, yawerr, 'displayname', 'uav yaw error'); catch; end
        legend('toggle')
        xlabel('time [s]')
        ylabel('degrees')
    hold off
    grid on
    if meta.saveplots; 
        saveas(gcf, [meta.figpath 'errors_v_time/png/' meta.date(1:end-1)  '_' meta.run 'xyzyaw_errors_v_time.png']);  
        saveas(gcf, [meta.figpath 'errors_v_time/fig/' meta.date(1:end-1)  '_' meta.run '_xyzyaw_errors_v_time.fig']);  
        print('-depsc', [meta.figpath 'errors_v_time/eps/' meta.date(1:end-1)  '_' meta.run '_xyzyaw_errors_v_time.eps']); 
    end
%% figure(200x); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(2000+run); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
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
        title(['uav x y z yaw error vs optical range ' meta.date meta.run])

    subplot (2,1,2)
    hold on
        try plot(uav_range.z, yawerr, '.', 'displayname', 'uav yaw error'); catch; end
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
%% figure(3000); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3000); current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
            try plot(uav_range.z, xerr, 'b.', 'displayname', 'uav x error'); catch; end
        hold off
        grid on
        xlabel('range [m]')
        ylabel('x error [m]')
    
%% figure(3001); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3001); current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
            try plot(uav_range.z, yerr, 'b.', 'displayname', 'uav y error'); catch; end
        hold off
        grid on
        xlabel('range [m]')
        ylabel('y error [m]')
    grid on
%% figure(3002); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3002); current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
            try plot(uav_range.z, zerr, 'b.', 'displayname', 'uav z error'); catch; end
        hold off
        grid on
        xlabel('range [m]')
        ylabel('z error [m]')
%% figure(3003); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    figure(3003); current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
            try plot(uav_range.z, yawerr, 'b.', 'displayname', 'uav yaw error'); catch; end
        hold off
        xlabel('yaw error [deg]')
        ylabel('range [m]')    
        grid on
    %% out the data
    stats.errtime = errtime;
    stats.xerr = xerr;
    stats.yerr = yerr;
    stats.zerr = zerr;
    stats.yawerr = yawerr;
    stats.uav_range = uav_range.z;
end