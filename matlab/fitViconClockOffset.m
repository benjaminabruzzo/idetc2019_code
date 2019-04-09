function [dt, varargout] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
% function [dt] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
    % find vicon clock offset
%     disp('[rostime(1) rostime(end) length(rostime)]');disp([rostime(1) rostime(end) length(rostime)])
%     disp('[vicontime(1) vicontime(end) ]');disp([vicontime(1) vicontime(end) ])

    % coarse fit
    offset.viconspline = csapi(vicontime,vicondata);
    offset.time(1) = vicontime(1) - rostime(1);
    disp(); disp()
    offset.rostime = rostime + offset.time(1);
    i = 1;
    while(offset.rostime(end) < vicontime(end))
%         disp(['i, offset.time(i)']);disp([i, offset.time(i)])
%         disp(['offset.rostime(end) vicontime(end)']); disp([offset.rostime(end) vicontime(end)])
        offset.splinerostime = fnval(offset.viconspline, offset.rostime);
        offset.sum_difference(i) = sum(abs(offset.splinerostime - (rosdata)));
        i = i+1;
        offset.time(i) = offset.time(i-1) + 0.001;
        offset.rostime = rostime + offset.time(i);
%         disp(['i, offset.time(i)']);disp([i, offset.time(i)])
%         disp(['offset.rostime(end) vicontime(end)']); disp([offset.rostime(end) vicontime(end)])

    end
    [min_diff,offset.best] = min(offset.sum_difference);

    dt = -offset.time(offset.best);
%     disp([' dt = ' num2str(dt) ', mindiff = ' num2str(min_diff/length(rostime))]);
    
    varargout{1} = min_diff/length(rostime);
    
end

%%

