function [data, meta] = plotslam_func(data, meta)
default_colors = getpref('display','default_colors');

%% figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,1),'bs', 'displayname', 'uav x slam est'); catch; end
%         try plot(data.uavSLAM.time, data.uavSLAM.uav.aug.p.global(:,1),'bx', 'displayname', 'uav x slam aug'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,1),'k.', 'displayname', 'uav x gazebo mocap'); catch; end
%         try plot(data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).time, ...
%                 data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).uav.P_ugv(:,1), ...
%                 'g.', 'displayname', 'uav x stereo'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPosition_gl(:,1), 'r.', 'displayname', 'uav inertial x'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odom_gl(:,1),'m.', 'displayname', 'uav inertial odom x, global'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.augmentedState(:,1), 'mo', 'displayname', 'uav augmentedState x'); catch; end
        
    hold off; grid on
    title(['uav x position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('x [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -1.5 3.5];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end
   
    if meta.saveplots
      
          meta.savefilename = ['uavSLAM_x_' meta.run];
          try_save_plot(meta);
    end
%% figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(2); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,2),'bs', 'displayname', 'uav y slam est'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,2),'k.', 'displayname', 'uav y gazebo mocap'); catch; end
%         try plot(data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).time, ...
%                 data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).uav.P_ugv(:,2), ...
%                 'g.', 'displayname', 'uav y stereo'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPosition_gl(:,2), 'r.', 'displayname', 'uav inertial y'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odom_gl(:,2),'m.', 'displayname', 'uav inertial odom y, global'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.augmentedState(:,2), 'mo', 'displayname', 'uav augmentedState y'); catch; end
    hold off; grid on
    title(['uav y position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('y [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -2 2];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end
    if meta.saveplots
          meta.savefilename = ['uavSLAM_y_' meta.run];
          try_save_plot(meta);
    end

%% figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(3); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.global(:,3),'bs', 'displayname', 'uav z slam est'); catch; end
%         try plot(data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).time, ...
%                 data.(matlab.lang.makeValidName(meta.ugv_leader_stereo)).uav.P_ugv(:,3), ...
%                 'go', 'displayname', 'uav z stereo'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.P.global(:,3),'k.', 'displayname', 'uav z gazebo mocap'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPosition_gl(:,3), 'r.', 'displayname', 'uav inertial z'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.augmentedState(:,3), 'mo', 'displayname', 'uav augmentedState z'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odom_gl(:,3),'m.', 'displayname', 'uav inertial odom z, global'); catch; end
    hold off; grid on
    title(['uav z position, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('z [m]')
    legend('toggle');legend('Location', 'SouthEast')
    
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -1 2];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end    
    if meta.saveplots
          meta.savefilename = ['uavSLAM_z_' meta.run];
          try_save_plot(meta);
    end

%% figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(4); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
        try plot(data.uavSLAM.time, data.uavSLAM.uav.est.yaw.global,'bs', 'displayname', 'uav yaw slam est'); catch; end
        try plot(data.vicon.uav.time, data.vicon.uav.yaw.global.radians,'k.', 'displayname', 'uav yaw gazebo mocap'); catch; end
        try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.EstPhi_gl, 'r.', 'displayname', 'uav inertial phi'); catch; end
%         try plot(data.uavSLAM.inertial.time, data.uavSLAM.inertial.odomPhi, 'm.', 'displayname', 'uav inertial phi odom'); catch; end
% %         try plot(data.ugv1Stereo.time, data.ugv1Stereo.uav.yaw_ugv*pi/180, 'go', 'displayname', 'uav stereo yaw'); catch; end
        
        
    hold off; grid on
    title(['uav yaw, global frame ' [meta.date meta.run]])
    xlabel('time [s]'); ylabel('yaw [rad]')
    legend('toggle');legend('Location', 'SouthEast')
    
    if meta.saveplots
          meta.savefilename = ['uavSLAM_yaw_' meta.run];
          try_save_plot(meta);
    end

%% figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(5); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    ii = 0; 
    for i=1:length(data.uavSLAM.april.tagsDetected)
        
        try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,1), ...
                 data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,2), ...
                 'o', 'Color', default_colors{i}, 'MarkerFaceColor', default_colors{i}, 'displayname', data.uavSLAM.april.tagsDetected{i}); 
               ii = ii + 1; legend_str{ii} = h1.DisplayName;
               
        catch; end
    end

    for i=1:length(data.vicon.april_list)
        try
            xyz_avg = mean(data.vicon.(matlab.lang.makeValidName(data.vicon.april_list{i})).P.global);
            h2 = plot(xyz_avg(1), xyz_avg(2), 'ks', 'MarkerFaceColor', [0,0,0]);
        catch
            continue
        end
        clear xyz_avg
    end
    
    
    hold off; grid on
    title(['landmark estimated position, global frame ' [meta.date meta.run]])
    xlabel('X [m]'); ylabel('Y [m]')
    try columnlegend(1,legend_str); catch; end; clear legend_str

%     hold on
%         for i=1:length(data.uavSLAM.april.tagsDetected)
%             try plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,1), ...
%                      data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,2), ...
%                      '.', 'displayname', data.uavSLAM.april.tagsDetected{i} ); catch; end
%         end
%     hold off;

    if meta.saveplots
          meta.savefilename = ['uavSLAM_landmarks_' meta.run];
          try_save_plot(meta);
    end


%% figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(8); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

  hold on
    for i=1:length(data.uavSLAM.april.tagsDetected)
      try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.MeasPosition_uav(:,1), ...
                    'x', 'Color', default_colors{1}, 'displayname', ['x meas uav']); catch; end
      try h2 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.PredictedMeasurement_uav(:,1), ...
                    'o', 'Color', default_colors{2}, 'displayname', ['x pred uav']); catch; end
      try h3 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.EstPosition_gl(:,1), ...
                    's', 'Color', default_colors{3}, 'displayname', ['x est P']); catch; end

    end
  hold off
  grid on
  title(['tag z position, uav measurement frame ' [meta.date meta.run]])
  legend([h1, h2, h3], 'Location', 'northwest')
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -2 5];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end
%% figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(9); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

  hold on
    for i=1:length(data.uavSLAM.april.tagsDetected)
      try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.MeasPosition_uav(:,2), ...
                    'x', 'Color', default_colors{1}, 'displayname', ['y meas uav']); catch; end
      try h2 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.PredictedMeasurement_uav(:,2), ...
                    'o', 'Color', default_colors{2}, 'displayname', ['y pred uav']); catch; end
      try h3 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.EstPosition_gl(:,2), ...
                    's', 'Color', default_colors{3}, 'displayname', ['y est P']); catch; end

    end
  hold off
  grid on
  title(['tag z position, uav measurement frame ' [meta.date meta.run]])
  legend([h1, h2, h3], 'Location', 'northwest')
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -2 2];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end
%% figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(10); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig

  hold on
    for i=1:length(data.uavSLAM.april.tagsDetected)
      try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.MeasPosition_uav(:,3), ...
                    'x', 'Color', default_colors{1}, 'displayname', ['z meas uav']); catch; end
      try h2 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.PredictedMeasurement_uav(:,3), ...
                    'o', 'Color', default_colors{2}, 'displayname', ['z pred uav']); catch; end
      try h3 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.time, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).slam.EstPosition_gl(:,3), ...
                    's', 'Color', default_colors{3}, 'displayname', ['z est P']); catch; end

    end
  hold off
  title(['tag z position, uav measurement frame ' [meta.date meta.run]])
  legend([h1, h2, h3], 'Location', 'northwest')
    try %current axis
      current_limits = axis; current_axes = gca; 
      axes = [current_limits(1) current_limits(2) -1 2];
      current_axes.XTick = [axes(1):5:axes(2)];
      current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
      current_axes.YTick = [axes(3):0.5:axes(4)];
      current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
    end   
%% figure(15); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(15); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.correction(:,1),'bx', 'displayname', 'uav x slam correction'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')
%% figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(16); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.correction(:,2),'bx', 'displayname', 'uav y slam correction'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')
%% figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(17); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.p.correction(:,3),'bx', 'displayname', 'uav z slam correction'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')    
%% figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(18); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
  hold on
    try plot(data.uavSLAM.time, data.uavSLAM.uav.est.yaw.correction(:,1),'bx', 'displayname', 'uav yaw slam correction'); catch; end
    try plot(data.uavSLAM.time, data.uavSLAM.uav.aug.yaw.correction(:,1),'bx', 'displayname', 'uav yaw slam correction'); catch; end
  hold off; grid on
  title(['uav z position, global frame ' [meta.date meta.run]])
  xlabel('time [s]'); ylabel('z [m]')
  legend('toggle');legend('Location', 'SouthEast')
%% figure(51-65); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    for i=1:length(data.uavSLAM.april.tagsDetected)
        figure(50+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
            hold on
            try h1 = plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,1), ...
                     data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(end,2), ...
                     'o', 'displayname', data.uavSLAM.april.tagsDetected{i}); catch; end
            try plot(data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,1), ...
                     data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,2), ...
                     '.', 'displayname', data.uavSLAM.april.tagsDetected{i} ); catch; end
            hold off; grid on
            title(['landmark estimated position, global frame ' [meta.date meta.run]])
            xlabel('X [m]'); ylabel('Y [m]')
            legend('toggle');legend('Location', 'SouthEast')

%             try %current axis
%                 current_limits = axis; current_axes = gca; 
%                 axes = [-2 2 -2 2];
%                 current_axes.XTick = [axes(1):0.5:axes(2)];
%                 current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%                 current_axes.YTick = [axes(3):0.5:axes(4)];
%                 current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%             end
            
    end

%% figure(66-80); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    for i=1:length(data.uavSLAM.april.tagsDetected)
        figure(65+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
            hold on
            try plot(...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstTime, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,1), ...
                    'o', 'displayname', [data.uavSLAM.april.tagsDetected{i} ' x']);
            catch; end
            try plot(...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstTime, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,2), ...
                    'o', 'displayname', [data.uavSLAM.april.tagsDetected{i} ' y']);
            catch; end
            try plot(...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstTime, ...
                    data.uavSLAM.april.(matlab.lang.makeValidName(data.uavSLAM.april.tagsDetected{i})).EstPosition_gl(:,3), ...
                    'x', 'displayname', [data.uavSLAM.april.tagsDetected{i} ' z']);
            catch; end
            hold off; grid on
            title(['landmark estimated position, global frame ' [meta.date meta.run]])
            xlabel('time [s]'); ylabel('X/Y/Z [m]')
            legend('toggle');legend('Location', 'SouthEast')

%             try %current axis
%                 current_limits = axis; current_axes = gca; 
%                 axes = [-2 2 -2 2];
%                 current_axes.XTick = [axes(1):0.5:axes(2)];
%                 current_axes.XLim = [current_axes.XTick(1) current_axes.XTick(end)];
%                 current_axes.YTick = [axes(3):0.5:axes(4)];
%                 current_axes.YLim = [current_axes.YTick(1) current_axes.YTick(end)];
%             end
            
    end

end



function try_save_plot(meta)
        current_fig = gcf;
        disp("try save plot :")
        disp(["    " meta.saveplotroot '/figs/' meta.savefilename '.png'])
        try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
%         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
        clear current_fig;
end

