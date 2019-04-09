disp('running loopclas.m')
%% meta.date = '20170106/'; jstr = {...
%     '020'; '022'; '024'; '025'; '026';...
%     '027'; '031'; '028'; '032'; '029';...
%     '030'; '033'; '034'; '035';...
%     '036'; '037'; '038';...
%     }; %037 is mostly okay, except for what looks like a single data point
%% meta.date = '20170112/'; jstr = {...
meta.date = '20170112/'; 
jstr = {...
        '020'; '022'; ...
        '026'; '027'; ...
        '028'; '029'; ...
        '032'; ...
        '033'; '034'; ...
        '035'; '036'; ...
        '037'; ...
% Do not use these:
%         '024'; '025'; ...
%         '030'; ...
%         '038'; ...
    };
%% meta.date = '20170120/'; jstr = {...
%     '001'; '002'; '003'; '004'; '005';...
%     '006'; '007'; '008'; '009'; '010';...
%     '011'; '012'; '013'; '014'; '015';...
%     '016'; '017'; '018'; '019'; '020';...
%     '021'; '022'; '023'; '024'; '025';...
%     '026'; '027'; '028'; '029'; '030';...
%     '031'; '032'; '033'; '034'; '035';...
%     '036'; '037'; '038'; '039'; '040';...
%     '041'; '042'; '043'; '044'; '045';...
%     '046'; '047'; '048'; ...
%     }; %037 is mostly okay, except for what looks like a single data point
%% meta.date = '20170126/'; jstr = {...
%     '001'; '002'; '003'; '004'; '005';...
%     '006'; '007'; '008'; '009'; '010';...
%     '011'; '012'; '013'; '014'; '015';...
%     '016'; '017'; '018'; '019'; '020';...
%     '021'; '022'; '023'; '024'; '025';...
%     '026'; '027'; '028'; '029'; '030';...
%     '031'; '032'; '033'; '034'; '035';...
%     '036'; '037'; '038'; '039'; '040';...
%     '041'; '042'; '043'; '044'; '045';...
%     '046'; '047'; '048'; ...
%     };
%% meta.date = '20170120/'; jstr = {...
% meta.date = '20170310/'; 
% jstr = {...
%     '001'; '002'; '003'; '004'; '005';... 
%     '006'; '011'; '016'; '022'; ... %cal A
%     '007'; '012'; '017'; '023'; ... %cal B
%     '008'; '013'; '018'; '019'; '024';... %cal C
%     '009'; '014'; '020'; '025'; ... %cal D
%     '010'; '015'; '021'; '026'; ... %cal E

%     '001'; '002'; '003'; '004'; '005';...
%     '006'; '007'; '008'; '009'; '010';...
%     '011'; '012'; '013'; '014'; '015';...
%     '016'; '017'; '018'; '019'; '020';...
%     '021'; '022'; '023'; '024'; '025';...
%     '026'; ...
% };
%% meta.date = '20170317/'; All
% meta.date = '20170317/'; jstr = {...
%     '001'; '002'; '003'; '004'; '005';...
%     '006'; '007'; '008'; '009'; '010';...
%     '011'; '012'; '013'; '014'; '015';...
%     '016'; '017'; '018'; '019'; '020';...
%     '021'; '022'; '023'; '024'; '025';...
%     '026'; '027'; '028'; '029'; '030';...
%     '031'; '032'; '033'; '034'; '035';...
%     '036'; '037'; '038'; '039'; '040';...
%     '041'; '042'; '043'; '044'; '045';...
%     '046'; '047'; '048'; '049'; '050';...
%     '051'; '052'; '053'; '054'; '055';...
%     };
%% meta.date = '20170317/'; CalibrationA
% meta.calibration = 'A';
% meta.date = '20170317/'; jstr = {...
% '012'; '015'; '018'; '022'; '025'; '028'; '031'; '049'; '046'; '034'; '037'; '040'; '043'; ...
% };
%% meta.date = '20170317/'; CalibrationB
% meta.calibration = 'B';
% meta.date = '20170317/'; jstr = {...
% '013'; '016'; '019'; '023'; '026'; '029'; '032'; '050'; '047'; '035'; '038'; '041'; '044'; ...
% };
%% meta.date = '20170317/'; CalibrationC
% meta.calibration = 'C';
% meta.date = '20170317/'; jstr = {...
% '014'; '017'; '021'; '024'; '027'; '030'; '033'; '051'; '048'; '036'; '039'; '042'; '045'; ...
% };
%% meta.date = '20170317/'; 
% meta.date = '20170331/'; jstr = {...
% '016'; '019'; ...
% '017'; '018'; ...
% };
%%


meta.dataroot = '/media/benjamin/devsdb/hast/data/';
meta.saveplots = false;
meta.saveplotroot = [meta.dataroot 'notable/' meta.date];
if meta.saveplots
    try mkdir([meta.saveplotroot 'eps']); catch; end
    try mkdir([meta.saveplotroot 'fig']); catch; end
    try mkdir([meta.saveplotroot 'png']); catch; end
end
clear loopout
loopout = struct;
meta.datenoslash = meta.date(1:end-1);
disp(['try to load ' meta.dataroot meta.date 'loopout.mat'])
try 
    %%
    tic
    load([meta.dataroot meta.date 'loopout.mat'], 'loopout')
    disp(['      time to load ' meta.dataroot meta.date 'loopout.mat ' num2str(toc) ])
    %%
catch
    disp(['loopout.mat not found, load data directly'])
    for run = 1:size(jstr,1)
        meta.run = jstr{run};

        try data = LoadRunData(meta); catch; end
            
        data.ugvStereo.uav.yaw_JJt = pixel_uncert_from_ugvStereo(data.ugvStereo);

        try loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon = data.vicon; catch; end
        try loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo = data.ugvStereo; catch; end
        try loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).uavRecorder = data.uavRecorder; catch; end
        try loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).offlineStereo = data.offlineStereo; catch; end

        try loopout.mean_vicon_p_cam(run)  = mean(data.vicon.uav.P.cam(:,3)); catch; end
        try loopout.mean_diff_yaw_error(run) = data.vicon.diff_yaw.spline_mean; catch; end
        try loopout.std_diff_yaw_error(run)  = data.vicon.diff_yaw.spline_std; catch; end
        try loopout.var_diff_yaw_error(run)  = data.vicon.diff_yaw.spline_var; catch; end
%         try loopout.mean_quad_computed_yaw_cov(run) = mean(ugvStereo.uav.UAV_yaw_var(~isnan(ugvStereo.uav.UAV_yaw_var))); catch; end
%         try loopout.mean_quad_computed_yaw_std(run) = mean(sqrt(ugvStereo.uav.UAV_yaw_var(~isnan(ugvStereo.uav.UAV_yaw_var)))); catch; end
        try loopout.mean_est_yaw_error(run) = data.vicon.est_yaw.spline_mean; catch; end
        try loopout.std_est_yaw_error(run)  = data.vicon.est_yaw.spline_std; catch; end
        try loopout.var_est_yaw_error(run)  = data.vicon.est_yaw.spline_var; catch; end
        
        loopout.range_zcell{run} = num2str(mean(data.vicon.uav.P.cam(:,3)));


        clear data
  
    end

    %%    
%     save([meta.dataroot meta.date 'loopout.mat'], 'loopout')
end

%% figure(150); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(150); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title('yaw variance based on propagated pixel variances')
    hold on
        try for run = 1:size(jstr,1)
            try
                UAV_yaw_var = 180*sqrt(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.uav.UAV_yaw_var)/pi;                
            catch
                
                try 
                    UAV_yaw_var = 180*sqrt(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.uav.yaw_JJt)/pi;
                catch
                    UAV_yaw_var = 180*sqrt(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.uav.UAV_yaw_cov)/pi;                     
                end
            end
            try h1 = plot(loopout.mean_vicon_p_cam(run)*ones(length(UAV_yaw_var),1)+0.01,UAV_yaw_var, 'rx', 'displayname', 'pixel computed yaw var'); catch; end
            try h1 = plot(loopout.mean_vicon_p_cam(run)*ones(length(UAV_yaw_var),1)+0.01,-UAV_yaw_var, 'rx', 'displayname', 'pixel computed yaw var'); catch; end

            try 
                spline_error = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.diff_yaw.spline_error;
                mean_spline_error = mean(spline_error);
                h2 = plot(loopout.mean_vicon_p_cam(run)*ones(length(spline_error),1),spline_error, 'bo', 'displayname', 'yaw-diff'); 
            catch; end

        end; catch; end
            try h3 = plot(loopout.mean_vicon_p_cam, loopout.mean_diff_yaw_error, 'g-s', 'LineWidth', 2, 'displayname', 'mean'); catch; end

    hold off
    xlabel('range along optical axis')
    ylabel('degrees')
    grid on
    try legend([h1 h2 h3], 'location', 'NorthWest');  catch; end
    axis([0 5 -30 30])
    if meta.saveplots 
        %%
%         meta.savefilename = ['yaw_diff_with_mean_' meta.calibration];
        meta.savefilename = ['figure_11'];
        print('-depsc', [meta.saveplotroot 'eps/' meta.savefilename '.eps']); 
        saveas(gcf, [meta.saveplotroot 'png/' meta.savefilename '.png'])
        saveas(gcf, [meta.saveplotroot 'fig/' meta.savefilename '.fig'])
    end

    clear h1 h2 h3 UAV_yaw_var spline_error mean_spline_error
%% figure(151); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(151); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('yaw variance based on propagated pixel variances')
    hold on
        try for run = 1:size(jstr,1)
            try
                UAV_yaw_var = 180*sqrt(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.uav.UAV_yaw_var)/pi;                
            catch
                
                try 
                    UAV_yaw_var = 180*sqrt(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.uav.yaw_JJt)/pi;
                catch
                    UAV_yaw_var = 180*sqrt(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.uav.UAV_yaw_cov)/pi;                     
                end
            end
            try h1 = plot(loopout.mean_vicon_p_cam(run)*ones(length(UAV_yaw_var),1)+0.025,UAV_yaw_var, ...
                    'rx', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end
            try h1 = plot(loopout.mean_vicon_p_cam(run)*ones(length(UAV_yaw_var),1)+0.025,-UAV_yaw_var, ...
                    'rx', 'displayname', 'sqrt of yaw uncertainty propagated from pixel uncertainty'); catch; end

            try 
                spline_error = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.diff_yaw.spline_error;
                mean_spline_error = mean(spline_error);
                h2 = plot(loopout.mean_vicon_p_cam(run)*ones(length(spline_error),1),spline_error-mean(spline_error), ...
                    'bo', 'displayname', 'zero mean error of stereo yaw angle measurement'); 
            catch; end

        end; catch; end
%             try h3 = plot(loopout.mean_vicon_p_cam, loopout.mean_est_yaw_error, 'g-s', 'LineWidth', 2, 'displayname', 'mean'); catch; end

%         try h3 = plot(loopout.mean_vicon_p_cam(1:4), loopout.mean_est_yaw_error(1:4), 'g-s', 'LineWidth', 2, 'displayname', 'mean calA'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(1:4), loopout.mean_est_yaw_error(1:4), 'g-s', 'LineWidth', 2, 'displayname', 'mean calB'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(5:8), loopout.mean_est_yaw_error(5:8), 'g-s', 'LineWidth', 2, 'displayname', 'mean calB'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(1:5), loopout.mean_est_yaw_error(1:5), 'g-s', 'LineWidth', 2, 'displayname', 'mean calC'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(9:13), loopout.mean_est_yaw_error(9:13), 'g-s', 'LineWidth', 2, 'displayname', 'mean calC'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(14:17), loopout.mean_est_yaw_error(14:17), 'g-s', 'LineWidth', 2, 'displayname', 'mean calD'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(1:4), loopout.mean_est_yaw_error(1:4), 'g-s', 'LineWidth', 2, 'displayname', 'mean calD'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(18:21), loopout.mean_est_yaw_error(18:21), 'g-s', 'LineWidth', 2, 'displayname', 'mean calE'); catch; end
%         try h3 = plot(loopout.mean_vicon_p_cam(1:4), loopout.mean_est_yaw_error(1:4), 'g-s', 'LineWidth', 2, 'displayname', 'mean calE'); catch; end

    hold off
    xlabel('range along optical axis')
    ylabel('degrees')
    grid on
    try legend([h2 h1], 'location', 'SouthWest');  catch; end
    axis([0 5 -135 135])
    try
        current_axes = gca;
        current_axes.YTick=[-135:45:135];
        current_axes.XTick=[0:5];
    catch
    end
    if meta.saveplots 
        %%
%         meta.savefilename = ['yaw_uncert_experimental_' meta.calibration];
%         meta.saveplotroot = ['/media/benjamin/devsdb/hast/tex/iros2017'];
%         meta.savefilename = ['figure_11'];
% 
%         try print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.png']); catch; end
%         try     saveas(gcf, [meta.saveplotroot '/figs/' meta.savefilename '.fig']); catch; end
    end

    clear h1 h2 h3 UAV_yaw_var spline_error mean_spline_error
%% figure(105); clf; current_fig = gcf; disp(['  figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
figure(105); clf; current_fig = gcf; disp(['  figure(' num2str(current_fig.Number) ') ..']); clear current_fig    
    for i = 1:size(loopout.range_zcell,2)
        loopout.range_zstr(i,:) = loopout.range_zcell{i}(1:4);
    end
    clear i
    loopout.err_data = [];
    loopout.err_group = [];
    for run = 1:size(jstr,1)
        spline_error = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.diff_yaw.spline_error;
        mean_spline_error = mean(spline_error);

        loopout.err_data = [loopout.err_data ; abs(spline_error-mean(spline_error))];
        loopout.err_group = [loopout.err_group ; repmat(loopout.range_zstr(run,:), size(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.diff_yaw.spline_error))];
    end
    clear run spline_error mean_spline_error

    try 
        boxplot(loopout.err_data, loopout.err_group); 
    catch; end
        ylabel('degrees')
        yxis = gca; ylim([0 40]); clear yxis
        xlabel('range along optical axis, [m]')
        ylabel('yaw angle error, [deg]')
        xxis=gca; xxis.XTickLabelRotation = -60; clear xxis
%         title('actual yaw errors at experimental locations')
    if meta.saveplots
        %%
        current_fig = gcf;
        saveas(gcf, [meta.saveplotroot 'experimental_yaw_box.png']); 
        print('-depsc', [meta.saveplotroot 'experimental_yaw_box.eps']); 
        clear current_fig;
    end
%% figure(178); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(178); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    hold on
    title(['ugv locations for ' meta.date])
    for run = 1:size(jstr,1)
        try h1 = plot(0,0, 'k*', 'displayname', 'vicon-origin'); catch; end
        try h2 = plot(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.ugvk.P.vicon(1,1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.ugvk.P.vicon(1,2), 'ms', 'displayname', 'ugv-vicon'); catch; end
        try h3 = plot(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.P.vicon(1,1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.P.vicon(1,2), 'ro', 'displayname', 'red-vicon-xy'); catch; end
        try h4 = plot(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.P.vicon(1,1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.P.vicon(1,2), 'bo', 'displayname', 'red-vicon-xy'); catch; end
        try h5 = plot(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.P.vicon(1,1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.P.vicon(1,2), 'go', 'displayname', 'red-vicon-xy'); catch; end

        try
            x1 = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.ugvk.P.vicon(1,1);
            x2 = x1 + 0.1* loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.ugvk.R_vic2lo(1,1,1);
            y1 = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.ugvk.P.vicon(1,2);
            y2 = y1 + 0.1* loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.ugvk.R_vic2lo(1,2,1);
            text(x1,y1+.1,jstr{run})
            line([x1 x2], [y1 y2]);

        catch
        end
    end
    hold off
    grid on
    if meta.saveplots 
%         saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date '01 ugv_locations.png'])
%         saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date '01 ugv_locations.fig'])
    end
    clear x1 x2 y1 y2
    clear h1 h2 h3 h4 h5
%% figure(179); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(179); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title('marker position differences between vicon and stereo, x (camera frame)')
hold on
    for run = 1:size(jstr,1)
        
        try mean_red_x_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.x_diff); catch; end
        try mean_blue_x_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.x_diff); catch; end
        try mean_green_x_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.x_diff); catch; end

        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.x_diff, 'rx', 'displayname', 'red x diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.x_diff, 'bx', 'displayname', 'blue x diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.x_diff, 'gx', 'displayname', 'green x diff'); catch; end

    end
    
    try plot(loopout.mean_vicon_p_cam, mean_red_x_diff, 'r' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_blue_x_diff, 'b' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_green_x_diff, 'g' ); catch; end

hold off
grid on
xlabel('range along optical axis')
ylabel('meters')
axis([0 5 -.5 .5])

if meta.saveplots 
    %%
        meta.savefilename = ['02 marker_diff_by_range_x_' meta.calibration];
        print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); 
        saveas(gcf, [meta.saveplotroot '/png/' meta.savefilename '.png'])
        saveas(gcf, [meta.saveplotroot '/fig/' meta.savefilename '.fig'])
end

clear mean_red_x_diff mean_blue_x_diff mean_green_x_diff
%% figure(180); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(180); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title('marker position differences between vicon and stereo, y (camera frame)')
hold on
    for run = 1:size(jstr,1)
        try mean_red_y_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.y_diff); catch; end
        try mean_blue_y_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.y_diff); catch; end
        try mean_green_y_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.y_diff); catch; end

        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.y_diff, 'rx', 'displayname', 'red y diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.y_diff, 'bx', 'displayname', 'blue y diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.y_diff, 'gx', 'displayname', 'green y diff'); catch; end

    end
    try plot(loopout.mean_vicon_p_cam, mean_red_y_diff, 'r' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_blue_y_diff, 'b' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_green_y_diff, 'g' ); catch; end

hold off
grid on
xlabel('range along optical axis')
ylabel('meters')
axis([0 5 -.5 .5])

if meta.saveplots 
    %%
        meta.savefilename = ['03 marker_diff_by_range_y_' meta.calibration];
        print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); 
        saveas(gcf, [meta.saveplotroot '/png/' meta.savefilename '.png'])
        saveas(gcf, [meta.saveplotroot '/fig/' meta.savefilename '.fig'])

end

clear mean_red_y_diff mean_blue_y_diff mean_green_y_diff
%% figure(181); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(181); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title('marker position differences between vicon and stereo, z (camera frame)')
hold on
    for run = 1:size(jstr,1)
        try mean_red_z_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.z_diff); catch; end
        try mean_blue_z_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.z_diff); catch; end
        try mean_green_z_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.z_diff); catch; end

        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.z_diff, 'rx', 'displayname', 'red z diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.z_diff, 'bx', 'displayname', 'blue z diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.z_diff, 'gx', 'displayname', 'green z diff'); catch; end

    end
    try plot(loopout.mean_vicon_p_cam, mean_red_z_diff, 'r' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_blue_z_diff, 'b' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_green_z_diff, 'g' ); catch; end

hold off
grid on
xlabel('range along optical axis')
ylabel('meters')
axis([0 5 -.5 .5])

if meta.saveplots 
    %%
        meta.savefilename = ['04 marker_diff_by_range_z_' meta.calibration];
        print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); 
        saveas(gcf, [meta.saveplotroot '/png/' meta.savefilename '.png'])
        saveas(gcf, [meta.saveplotroot '/fig/' meta.savefilename '.fig'])

end

clear mean_red_z_diff mean_blue_z_diff mean_green_z_diff
%% figure(184); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(184); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title('marker position differences between vicon and stereo, rms')
    hold on
    for run = 1:size(jstr,1)
        try mean_red_rms_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.rms_diff); catch; end
        try mean_blue_rms_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.rms_diff); catch; end
        try mean_green_rms_diff(run) = mean(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.rms_diff); catch; end

        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.rms_diff, 'rx', 'displayname', 'vicon-red-ugvStereotime'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.rms_diff, 'bx', 'displayname', 'vicon-blue-ugvStereotime'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.rms_diff, 'gx', 'displayname', 'vicon-green-ugvStereotime'); catch; end

    end

    try plot(loopout.mean_vicon_p_cam, mean_red_rms_diff, 'r' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_blue_rms_diff, 'b' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_green_rms_diff, 'g' ); catch; end

    hold off
    xlabel('range along optical axis')
    ylabel('meters')
    grid on
    axis([0 5 -.5 .5])

    if meta.saveplots 
%%
        meta.savefilename = ['05 marker_diff_by_range_rms_' meta.calibration];
        print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); 
        saveas(gcf, [meta.saveplotroot '/png/' meta.savefilename '.png'])
        saveas(gcf, [meta.saveplotroot '/fig/' meta.savefilename '.fig'])

    end

clear mean_red_rms_diff mean_blue_rms_diff mean_green_rms_diff
%% figure(191); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(191); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('Actual yaw difference between vicon and ugv')
%     hold on
%     for run = 1:size(jstr,1)
%         disp(['run_' jstr{run}])
%         try
%             spline_error = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.est_yaw.spline_error;
%             try h1 = plot(loopout.mean_vicon_p_cam(run)*ones(length(spline_error),1),spline_error, 'bo', 'displayname', 'yaw-diff'); catch; end
%             x1 = mean_vicon_p_cam(run);
%             y1 = 0;
%             text(x1,y1-1.1*run,jstr{run})
%         catch
%         end
%     end
%     try h2 = plot(loopout.mean_vicon_p_cam, loopout.mean_est_yaw_error, 'r-s', ...
%             'LineWidth', 2, 'displayname', 'mean'); catch; end
% %     try h3 = plot(loopout.mean_vicon_p_cam, loopout.mean_est_yaw_error+loopout.std_est_yaw_error, 'k-d', ...
% %             'LineWidth', 2,  'displayname', 'zeromean std(yaw diff)'); catch; end
% %     try h4 = plot(loopout.mean_vicon_p_cam, loopout.mean_est_yaw_error-loopout.std_est_yaw_error, 'k-d', ...
% %             'LineWidth', 2,  'displayname', 'zeromean std(yaw diff)'); catch; end
% 
%     hold off
%     xlabel('range along optical axis')
%     ylabel('degrees')
%     grid on
%     axis([0 6 -30 30])
%     try legend([h1 h2 h3], 'location', 'NorthWest');  catch; end
%     if meta.saveplots 
%         saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date '07 estyaw_diff_by_range.png'])
%         saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date '07 estyaw_diff_by_range.fig'])
%     end
%     clear x1 y1  h1 h2 h3 h4
% 
% clear spline_error mean_vicon_p_cam i UAV_yaw_var
% clear std_spline_error mean_spline_error var_spline_error
% 
% clear RedJJT BlueJJT GreenJJT
% clear Redjjt Bluejjt Greenjjt
% 
% %      ugvStereo.uav.UAV_yaw_var
%% figure(179); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     figure(179); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('marker position differences in camera frame')
%     hold on
%     for run = 1:size(jstr,1)
%        try
%            len = size(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.x_diff,1);
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.y_diff, 'rx'); 
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.y_diff, 'bx'); 
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.y_diff, 'gx'); 
%            clear len
%        catch; end 
%     end
%     
%     
%     hold off
%     grid on
%     clear A B C D E F
%     xlabel('meters along optical axis')
%     ylabel('meters')
%     saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date 'cam_diff_x.png'])
%% figure(180); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     figure(180); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('marker position differences in camera frame')
%     hold on
%     for run = 1:size(jstr,1)
%        try
%            len = size(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.x_diff,1);
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.z_diff, 'rx'); 
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.z_diff, 'bx'); 
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.z_diff, 'gx'); 
%            clear len
%        catch; end 
%     end
%     
%     
%     hold off
%     grid on
%     clear A B C D E F
%     xlabel('meters along optical axis')
%     ylabel('meters')
%     saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date 'cam_diff_y.png'])
%% figure(181); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     figure(181); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('marker position differences in camera frame')
%     hold on
%     for run = 1:size(jstr,1)
%        try
%            len = size(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.x_diff,1);
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.red_target.splines.P.cam.x_diff, 'rx'); 
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.blue_target.splines.P.cam.x_diff, 'bx'); 
%            plot(loopout.mean_vicon_p_cam(run)*ones(len),loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.green_target.splines.P.cam.x_diff, 'gx'); 
%            clear len
%        catch; end 
%     end
%     
%     hold off
%     grid on
%     clear A B C D E F
%     xlabel('meters along optical axis')
%     ylabel('meters')
%     saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date 'cam_diff_z.png'])
%% figure(182); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% figure(182); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     title('Actual yaw difference between vicon and ugv')
%     hold on
%     for run = 1:size(jstr,1)
%         disp(['run_' jstr{run}])
%         try
%             spline_error = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon.diff_yaw.spline_error;
%             try h1 = plot(loopout.mean_vicon_p_cam(run)*ones(length(spline_error),1),spline_error, 'bo', 'displayname', 'yaw-diff'); catch; end
%             x1 = mean_vicon_p_cam(run);
%             y1 = 0;
%             text(x1,y1-1.1*run,jstr{run})
%         catch
%         end
%     end
%     try h2 = plot(loopout.mean_vicon_p_cam, loopout.mean_diff_yaw_error, 'r-s', ...
%             'LineWidth', 2, 'displayname', 'mean'); catch; end
%     try h3 = plot(loopout.mean_vicon_p_cam, loopout.mean_diff_yaw_error+loopout.std_diff_yaw_error, 'k-d', ...
%             'LineWidth', 2,  'displayname', 'zeromean std(yaw diff)'); catch; end
%     try h4 = plot(loopout.mean_vicon_p_cam, loopout.mean_diff_yaw_error-loopout.std_diff_yaw_error, 'k-d', ...
%             'LineWidth', 2,  'displayname', 'zeromean std(yaw diff)'); catch; end
% 
%     hold off
%     xlabel('range along optical axis')
%     ylabel('degrees')
%     grid on
%     axis([0 6 -30 30])
%     try legend([h1 h2 h3], 'location', 'NorthWest');  catch; end
%     if meta.saveplots 
%         saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date '05 yaw_diff_by_range.png'])
%         saveas(gcf, [meta.dataroot 'NotableFigures/' meta.date '05 yaw_diff_by_range.fig'])
%     %%
%         meta.savefilename = ['05 yaw_diff_by_range' meta.calibration];
%         print('-depsc', [meta.saveplotroot '/eps/' meta.savefilename '.eps']); 
%         saveas(gcf, [meta.saveplotroot '/png/' meta.savefilename '.png'])
%         saveas(gcf, [meta.saveplotroot '/fig/' meta.savefilename '.fig'])
% 
%     end
%     clear x1 y1  h1 h2 h3 h4
%% test block angles
% loopout.block_angles = {...
% % 14;20;22.5; ...
% % 12; 12.5;...
% 13; 13.5; ...
% 14; 14.5; ...
% 15; ...
% 19; ...
% 20; 20.858; 20.859; 20.86; 20.861; 20.862;...
% 21; ...
% };
% 
% for run = 1:size(jstr,1)
%     [loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks]= ...
%         block_sweep(loopout.block_angles, loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).vicon,loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo );
% end

% figure(500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
title('marker position differences between vicon and stereo, z (camera frame)')
hold on
    for run = 1:size(jstr,1)
        try mean_red_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.red_target.P.cam.mean_diff(3); catch; end
        try std_red_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.red_target.P.cam.std_diff(3); catch; end
        try mean_blue_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.blue_target.P.cam.mean_diff(3); catch; end
        try std_blue_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.blue_target.P.cam.std_diff(3); catch; end
        try mean_green_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.green_target.P.cam.mean_diff(3); catch; end
        try std_green_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.green_target.P.cam.std_diff(3); catch; end

        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.green_target.P.cam.diff(:,3), 'gx', 'displayname', 'green z diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.blue_target.P.cam.diff(:,3), 'bx', 'displayname', 'blue z diff'); catch; end
        try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.block_1.splines.red_target.P.cam.diff(:,3), 'rx', 'displayname', 'red z diff'); catch; end

    end
    try plot(loopout.mean_vicon_p_cam, mean_red_z_diff, 'r' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_red_z_diff+std_red_z_diff, 'r--' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_red_z_diff-std_red_z_diff, 'r--' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_blue_z_diff, 'b' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_blue_z_diff+std_blue_z_diff, 'b--' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_blue_z_diff-std_blue_z_diff, 'b--' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_green_z_diff, 'g' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_green_z_diff+std_green_z_diff, 'g--' ); catch; end
    try plot(loopout.mean_vicon_p_cam, mean_green_z_diff-std_green_z_diff, 'g--' ); catch; end

hold off
grid on
xlabel('range along optical axis')
ylabel('meters')
axis([0 5 -.5 .5])

clear mean_red_z_diff mean_blue_z_diff mean_green_z_diff

% 400 block plots: y diff in cam frame
try
    for i = 1:size(loopout.block_angles,1)
    figure(400+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker position differences between vicon and stereo, camera frame y, ' num2str(loopout.block_angles{i}) ' degree block'])
    hold on
        for run = 1:size(jstr,1)
            try mean_red_y_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.mean_diff(2); catch; end
            try mean_blue_y_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.mean_diff(2); catch; end
            try mean_green_y_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.mean_diff(2); catch; end

            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,2), ...
                    'gx', 'displayname', 'green z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,2), ...
                    'bx', 'displayname', 'blue z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,2), ...
                    'rx', 'displayname', 'red z diff'); catch; end

        end
        try plot(loopout.mean_vicon_p_cam, mean_red_y_diff, 'r' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_blue_y_diff, 'b' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_green_y_diff, 'g' ); catch; end

    hold off
    grid on
    xlabel('range along optical axis')
    ylabel('meters')
    axis([0 5 -.5 .5])

    clear mean_red_y_diff mean_blue_y_diff mean_green_y_diff

    end
end
% 500 block plots: z diff in cam frame
try
for i = 1:size(loopout.block_angles,1)
    figure(500+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker position differences between vicon and stereo, camera frame z, ' num2str(loopout.block_angles{i}) ' degree block'])
    hold on
        for run = 1:size(jstr,1)
            try mean_red_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.mean_diff(3); catch; end
            try mean_blue_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.mean_diff(3); catch; end
            try mean_green_z_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.mean_diff(3); catch; end

            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,3), ...
                    'gx', 'displayname', 'green z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,3), ...
                    'bx', 'displayname', 'blue z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,3), ...
                    'rx', 'displayname', 'red z diff'); catch; end

        end
        try plot(loopout.mean_vicon_p_cam, mean_red_z_diff, 'r' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_blue_z_diff, 'b' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_green_z_diff, 'g' ); catch; end

    hold off
    grid on
    xlabel('range along optical axis')
    ylabel('meters')
    axis([0 5 -.5 .5])

    clear mean_red_z_diff mean_blue_z_diff mean_green_z_diff

end
end
% 600 block plots: x diff in cam frame
try
for i = 1:size(loopout.block_angles,1)
    figure(600+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker position differences between vicon and stereo, camera frame x, ' num2str(loopout.block_angles{i}) ' degree block'])
    hold on
        for run = 1:size(jstr,1)
            try mean_red_x_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.mean_diff(1); catch; end
            try mean_blue_x_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.mean_diff(1); catch; end
            try mean_green_x_diff(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.mean_diff(1); catch; end

            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.cam.diff(:,1), ...
                    'gx', 'displayname', 'green z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.cam.diff(:,1), ...
                    'bx', 'displayname', 'blue z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.cam.diff(:,1), ...
                    'rx', 'displayname', 'red z diff'); catch; end

        end
        try plot(loopout.mean_vicon_p_cam, mean_red_x_diff, 'r' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_blue_x_diff, 'b' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_green_x_diff, 'g' ); catch; end

    hold off
    grid on
    xlabel('range along optical axis')
    ylabel('meters')
    axis([0 5 -.5 .5])

    clear mean_red_x_diff mean_blue_x_diff mean_green_x_diff

end
end
% 700 block plots: rms diff
try
for i = 1:size(loopout.block_angles,1)
    figure(700+i); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    title(['marker position differences between vicon and stereo, camera frame x, ' num2str(loopout.block_angles{i}) ' degree block'])
    hold on
        for run = 1:size(jstr,1)
            try mean_red_rms(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.rms_mean(1); catch; end
            try mean_blue_rms(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.rms_mean(1); catch; end
            try mean_green_rms(run) = loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.rms_mean(1); catch; end

            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.green_target.P.rms(:,1), ...
                    'gx', 'displayname', 'green z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.blue_target.P.rms(:,1), ...
                    'bx', 'displayname', 'blue z diff'); catch; end
            try plot(loopout.mean_vicon_p_cam(run)*ones(length(loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).ugvStereo.time),1), ...
                    loopout.(matlab.lang.makeValidName(['run_' jstr{run}])).angleblocks.(matlab.lang.makeValidName(['block_' num2str(i)])).splines.red_target.P.rms(:,1), ...
                    'rx', 'displayname', 'red z diff'); catch; end

        end
        try plot(loopout.mean_vicon_p_cam, mean_red_rms, 'r' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_blue_rms, 'b' ); catch; end
        try plot(loopout.mean_vicon_p_cam, mean_green_rms, 'g' ); catch; end

        red_rms(i) = sum(mean_red_rms);
        blue_rms(i) = sum(mean_blue_rms);
        green_rms(i) = sum(mean_green_rms);
        
    hold off
    grid on
    xlabel('range along optical axis')
    ylabel('meters')
    axis([0 5 -.5 .5])

    clear mean_red_rms mean_blue_rms mean_green_rms
end
end