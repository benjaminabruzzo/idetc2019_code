function data = plotuavnorms(data)
%%

data.ugvStereo.uav.norm_GmB = sqrt(data.ugvStereo.uav.GmB(:,1).*data.ugvStereo.uav.GmB(:,1) + ...
    data.ugvStereo.uav.GmB(:,2).*data.ugvStereo.uav.GmB(:,2) + ...
    data.ugvStereo.uav.GmB(:,3).*data.ugvStereo.uav.GmB(:,3));
data.ugvStereo.uav.norm_BmR = sqrt(data.ugvStereo.uav.BmR(:,1).*data.ugvStereo.uav.BmR(:,1) + ...
    data.ugvStereo.uav.BmR(:,2).*data.ugvStereo.uav.BmR(:,2) + ...
    data.ugvStereo.uav.BmR(:,3).*data.ugvStereo.uav.BmR(:,3));
data.ugvStereo.uav.norm_RmG = sqrt(data.ugvStereo.uav.RmG(:,1).*data.ugvStereo.uav.RmG(:,1) + ...
    data.ugvStereo.uav.RmG(:,2).*data.ugvStereo.uav.RmG(:,2) + ...
    data.ugvStereo.uav.RmG(:,3).*data.ugvStereo.uav.RmG(:,3));

plot_uav_norms(data)
end


function plot_uav_norms(data)
%% figure(301); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(301); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
        hold on
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_RmG, 'r+', 'displayname', 'norm-RmG'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_GmB, 'g+', 'displayname', 'norm-GmB'); catch; end
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_BmR, 'b+', 'displayname', 'norm-BmR'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
%% figure(302); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(302); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
    subplot(3,1,1)
        hold on
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_RmG, 'r+', 'displayname', 'norm-RmG'); catch; end
            mean_norm_RmG = mean(data.ugvStereo.uav.norm_RmG(1:40))
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_RmG mean_norm_RmG]'); catch; end
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_RmG+0.03 mean_norm_RmG+0.03]'); catch; end
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_RmG-0.03 mean_norm_RmG-0.03]'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
    subplot(3,1,2)
        hold on
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_GmB, 'g+', 'displayname', 'norm-GmB'); catch; end
            mean_norm_GmB = mean(data.ugvStereo.uav.norm_GmB(1:40))
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_GmB mean_norm_GmB]'); catch; end
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_GmB+0.03 mean_norm_GmB+0.03]'); catch; end
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_GmB-0.03 mean_norm_GmB-0.03]'); catch; end
        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
    subplot(3,1,3)
        hold on
            try plot(data.ugvStereo.time, data.ugvStereo.uav.norm_BmR, 'b+', 'displayname', 'norm-BmR'); catch; end
            mean_norm_BmR = mean(data.ugvStereo.uav.norm_BmR(1:40))
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_BmR mean_norm_BmR]'); catch; end
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_BmR+0.03 mean_norm_BmR+0.03]'); catch; end
            try line([data.ugvStereo.time(1) data.ugvStereo.time(end)]', [mean_norm_BmR-0.03 mean_norm_BmR-0.03]'); catch; end

        hold off
        grid on
        legend('toggle')
        ylabel('x [m]')
        xlabel('t [sec]')
end