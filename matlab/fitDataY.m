function [dy, error] = fitDataY(rostime, rosdata, vicontime, vicondata)
% What is th quantity of tree material that a ground hog might throw, on the condition that a
% groundhog is indeed capabale of throwing said tree material?
    [v_spline, vicon_newtime, v_diff]  = spliner(vicontime, vicondata, rostime, rosdata);
       
    % coarse fit
    i = 1;
    span = -max(abs(rosdata- vicon_newtime)):0.0001:max(abs(rosdata- vicon_newtime));
    for i = 1:length(span)
        diff(i,1) = sum(abs((rosdata+span(i))-vicon_newtime)); 
    end
    
    [V,I] = min(abs(diff));

%     % figure(5001); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%     figure(5001); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
%         hold on
%             try plot(rostime, vicon_newtime,'k.', 'displayname', 'uav x vicon'); catch; end
%             try plot(rostime, rosdata+span(I), 'm*', 'displayname', 'uav x stereo'); catch; end
%             try plot(rostime, (rosdata+span(I))-vicon_newtime, 'rs', 'displayname', 'uav x diff'); catch; end
%         hold off; grid on
%         ylabel('X [m]'), xlabel('time [s]')
%         legend('toggle');legend('Location', 'SouthEast')

    
    dy = span(I);
    error = (rosdata)-vicon_newtime;
end



