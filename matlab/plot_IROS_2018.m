%% root data
meta.iros2018root = '/Users/benjamin/hast/tex/iros2018/figs/';


%% figure(1000); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
% Load data
%     load([meta.iros2018root mat_file], in)

%% plot
figure(1000); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    linewidth = 4;
    hold on; ii = 0; % ]
        try h1 = plot(data.kobuki_logger.global_plan.path.xyz{1}(:,1), data.kobuki_logger.global_plan.path.xyz{1}(:,2), ':', 'Color', [0.5,0.5,0.5], ...
                'LineWidth', linewidth,  'displayname', 'planned path'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        try h1 = plot(data.vicon.ugvk.P.global(:,1), data.vicon.ugvk.P.global(:,2), '-', 'Color', [0.5,0.5,0.5], ...
                'LineWidth', linewidth,  'displayname', 'actual path'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end
        try h1 = plot(data.ugvRecorder.est.Position_gl(:,1), data.ugvRecorder.est.Position_gl(:,2), '--', 'Color', [0,0,0], ...
                'LineWidth', linewidth,  'displayname', 'estimated path'); ii = ii + 1; legend_str{ii} = h1.DisplayName; catch; end

    hold off; grid on; clear h1 ii
    try columnlegend(3,legend_str); catch; end; clear legend_str
    xlabel('UGV x position (m)'); 
    ylabel('UGV y position (m)'); 
    %% save plot
    % meta.date = '20180224/'; % actual experiment data
    % meta.run = '011'; % 

    saveas(gcf, [meta.iros2018root 'pathplan/pathplan' meta.date(1:end-1) '_' meta.run '.fig']); 
    print('-depsc', [meta.iros2018root 'pathplan/pathplan' meta.date(1:end-1) '_' meta.run '.eps']); 
%     save([meta.iros2018root mat_file], in)

    
