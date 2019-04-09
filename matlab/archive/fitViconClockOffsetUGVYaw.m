function [dt] = fitViconClockOffsetUGVYaw(ugvRecorder, vicon, meta)
% function [dt] = fitViconClockOffsetYaw(wheeltime, wheeldata, vicontime, vicondata, meta)
% fitViconClockOffsetYaw(ugvStereo.time, ugvStereo.uav.yaw_ugv, vicon.uav.yaw.global.degrees, vicon.uav.time, meta);
%% find vicon clock offset

    wheeltime = ugvRecorder.wheel.time;
    wheeldata = ugvRecorder.wheel.yaw + vicon.global.orientation;
    vicondata = vicon.ugvk.yaw.radians;
    vicontime = vicon.ugvk.time;
    
    disp('[wheeltime(1) wheeltime(end)]');disp([wheeltime(1) wheeltime(end)])
    disp('[vicontime(1) vicontime(end)]');disp([vicontime(1) vicontime(end)])

    %% coarse fit
    offset.viconspline = csapi(vicontime,vicondata);
    offset.time(1) = vicontime(1) - wheeltime(1);
    offset.wheeltime = wheeltime + offset.time(1);
    i = 1;
    while(offset.wheeltime(end) < vicontime(end))
        offset.splineugvtime = fnval(offset.viconspline, offset.wheeltime);
        offset.sum_difference(i) = sum(abs(offset.splineugvtime - (wheeldata)));
        i = i+1;
        offset.time(i) = offset.time(i-1) + 0.001;
        offset.wheeltime = wheeltime + offset.time(i);
    end
    [min_diff,offset.best] = min(offset.sum_difference);

    disp(['coarse fit, dt = ' num2str(-offset.time(offset.best))]);

    %% fine fit
    dt = -offset.time(offset.best);

    figure(200); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('dt and euler values')
        hold on
            try plot(eulerout, 'displayname', 'eulerout'); catch; end
            try plot(dtout, 'displayname', 'dtout'); catch; end
        hold off
        grid on
        legend('toggle')
        %%
    figure(201); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title(['clock not adjusted ' [meta.date meta.run]])    
        ylabel('meters')
        hold on
            try plot(vicontime, vicondata, 'kx', 'displayname', 'vicon-yaw-ugvfame'); catch; end
            try plot(wheeltime, wheeldata, 'rx', 'displayname', 'ugvdata') ; catch; end
        hold off
        grid on
        legend('toggle')
        current_axes = axis;
        %%
    figure(202); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title(['clock adjusted, coarse dt = ' num2str(-offset.time(offset.best)) ', i= ' num2str(i) ', ' meta.date meta.run])    
        ylabel('meters')
        hold on
            try plot(vicontime-offset.time(offset.best), vicondata, 'kx', 'displayname', 'vicon-yaw-ugvfame'); catch; end
            try plot(wheeltime, wheeldata, 'rx', 'displayname', 'ugvdata') ; catch; end
        hold off
        grid on
        legend('toggle')
        axis(current_axes)


    %%
    figure(203); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        title(['clock adjusted, dt = ' num2str(dt) ', i= ' num2str(i) ', ' meta.date meta.run])    
        ylabel('meters')
        hold on
            try plot(vicontime+dt, vicondata, 'kx', 'displayname', 'vicon-yaw-ugvfame'); catch; end
            try plot(wheeltime, wheeldata, 'rx', 'displayname', 'ugvdata') ; catch; end
        hold off
        grid on
        legend('toggle')
        axis(current_axes)
    
end