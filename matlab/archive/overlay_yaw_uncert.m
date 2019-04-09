function overlay_yaw_uncert(loopout1, loopout2)
%% combine results from two different loopout structures
% loopout1 should be the data from 20170112
% loopout2 should be the data from 20170913


%% figure(701); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(701); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['yaw error and pixel propagated uncertainties, ' loopout1.meta.date(1:end-1)])    
    ylabel('degrees')
    xlabel('range along optical axis, [m]')
    hold on
    
    for i = 1:size(fields(loopout1),1)-2
        ordinate1 = mean(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
            ones(size(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error));
        ordinate3 = mean(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
            ones(size(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var));

        try h1 = plot(ordinate1, ...
                loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error, ...
                'bo', 'displayname', 'error of stereo yaw angle measurement'); catch; end
        try h2 = plot(-0.025+ordinate3,180*sqrt(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var)/pi, ...
               'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
        try h3 = plot(-0.025+ordinate3,-180*sqrt(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var)/pi, ...
               'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
        
        clear ordinate1 ordinate3
        
    end; clear i
    hold off
    grid on
    axis([0 5 -135 135])
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'southwest')
        clear h1 h2 h3
    catch
    end
            
    try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0:1:5];
        current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
        current_axes.YLim = [-135 135];
        current_axes.YTick = [current_axes.YLim(1):45:current_axes.YLim(2)];
        clear current_axes current_limits
    end

    % if meta.saveplots
    %     current_fig = gcf;
    %     saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
    %     clear current_fig;
    % end    

    
%% figure(702); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(702); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['yaw error and pixel propagated uncertainties, ' loopout2.meta.date(1:end-1)])    
    ylabel('degrees')
    xlabel('range along optical axis, [m]')
    hold on
    
    for i = 1:size(fields(loopout2),1)-2
        ordinate1 = mean(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
            ones(size(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error));
        try h1 = plot(ordinate1, ...
                loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error, ...
                'ko', 'displayname', 'error of stereo yaw angle measurement'); catch; end
        try h2 = plot(0.025+ordinate1, 180*sqrt(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvRecorder.stereo.yaw_cov)/pi, ...
                'rx', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
        try h3 = plot(0.025+ordinate1, -180*sqrt(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvRecorder.stereo.yaw_cov)/pi, ...
                'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
        clear ordinate1 
        
    end; clear i
    hold off
    grid on
    axis([0 5 -135 135])
    try
        legend('toggle')
        legend([h1, h2], 'Location', 'southwest')
        clear h1 h2 h3
    catch
    end
            
    try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0:1:5];
        current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
        current_axes.YLim = [-135 135];
        current_axes.YTick = [current_axes.YLim(1):45:current_axes.YLim(2)];
        clear current_axes current_limits
    end

    % if meta.saveplots
    %     current_fig = gcf;
    %     saveas(gcf, [meta.figpath 'figure(' num2str(current_fig.Number) ').png']); 
    %     clear current_fig;
    % end    

%% figure(703); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(703); clf; current_fig = gcf; vprint(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['yaw error and pixel propagated uncertainties, both dates'])    
    ylabel('degrees')
    xlabel('range along optical axis, [m]')
    hold on
    % loopout 1
        for i = 1:size(fields(loopout1),1)-2
            ordinate1 = mean(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
                ones(size(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error));
            ordinate3 = mean(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
                ones(size(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var));

            try h2 = plot(-0.025+ordinate3,180*sqrt(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var)/pi, ...
                   'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty, low-res'); catch; end
            try h3 = plot(-0.025+ordinate3,-180*sqrt(loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvStereo.PostProc.uav.yaw_var)/pi, ...
                   'm*', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
            try h1 = plot(ordinate1, ...
                    loopout1.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error, ...
                    'bo', 'displayname', 'error of stereo yaw angle measurement, low-res'); catch; end
        end; clear i ordinate1 ordinate2 ordinate3
    %loopout 2
        for i = 1:size(fields(loopout2),1)-2
            ordinate1 = mean(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.uav.splines.P.cam.z_uavstereotime)*...
                ones(size(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error));
            try h5 = plot(0.025+ordinate1, 180*sqrt(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvRecorder.stereo.yaw_cov)/pi, ...
                    'rx', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty, high-res'); catch; end
            try h6 = plot(0.025+ordinate1, -180*sqrt(loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).ugvRecorder.stereo.yaw_cov)/pi, ...
                    'rx', 'displayname', 'pixel propagated uncertainty'); catch; end
            try h4 = plot(ordinate1, ...
                    loopout2.(matlab.lang.makeValidName(['set_' num2str(i)])).vicon.diff_yaw.spline_error, ...
                    'ko', 'displayname', 'error of stereo yaw angle measurement, high-res'); catch; end
        end; clear i ordinate1
    hold off 
    hold off
    grid on
    axis([0 5 -135 135])
    try
        legend('toggle')
        legend([h1, h2, h4, h5], 'Location', 'southwest')
        clear h1 h2 h3
    catch
    end
            
    try
        current_limits = axis;
        current_axes = gca; % current axes
        current_axes.XTick = [0:1:5];
        current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
        current_axes.YLim = [-135 135];
        current_axes.YTick = [current_axes.YLim(1):45:current_axes.YLim(2)];
        clear current_axes current_limits
    end
end