clc
disp('uav starting location')
disp(mean([data.vicon.uav.P.vicon(1:10,:) data.vicon.uav.yaw.radians(1:10)]))

disp('ugvk starting location')
disp(mean([data.vicon.ugvk.P.vicon(1:10,:) data.vicon.ugvk.yaw.radians(1:10)]))

try
disp('april02_ starting location')
disp(mean([data.vicon.april02_.P.vicon(1:10,:) data.vicon.april02_.yaw.radians(1:10)]))
catch; end

try
disp('april03_ starting location')
disp(mean([data.vicon.april03_.P.vicon(1:10,:) data.vicon.april03_.yaw.radians(1:10)]))
catch; end

try
disp('april06_ starting location')
disp(mean([data.vicon.april06_.P.vicon(1:10,:) data.vicon.april06_.yaw.radians(1:10)]))
catch; end

try
disp('april07_ starting location')
disp(mean([data.vicon.april07_.P.vicon(1:10,:) data.vicon.april07_.yaw.radians(1:10)]))
catch; end

%%
clear ugv_path
ugv_path.x = data.vicon.ugvk.atEstTime.P.global(:,1); 
    ugv_path.x(data.vicon.ugvk.time>data.metrics.kobuki.tend) = [];
    ugv_path.x(data.vicon.ugvk.time<data.metrics.kobuki.tstart) = [];
ugv_path.y = data.vicon.ugvk.atEstTime.P.global(:,2); 
    ugv_path.y(data.vicon.ugvk.time>data.metrics.kobuki.tend) = [];
    ugv_path.y(data.vicon.ugvk.time<data.metrics.kobuki.tstart) = [];
    size(ugv_path.x)
    size(ugv_path.y)    
    getDisplacement([ugv_path.x ugv_path.y]')
    
    
%%
% A = [[0 0 0]; data.vicon.ugvk.atEstTime.P.global];
% 
% size(A)
% size(data.vicon.ugvk.atEstTime.P.global)
% for i = 1:length(data.vicon.ugvk.atEstTime.P.global)
%     data.vicon.ugvk.atEstTime.odom.global(i,:) = A(i,)
% end



%%
A = data.vicon.ugvk.atEstTime.P.global';

p = vecnorm(A)';

dp(1) = 0;
sump(1) = 0;
for i = 1:(length(p)-1)
    dp(i+1,1) = abs(p(i+1)-p(i));
    sump(i+1,1) = sump(i,1) + dp(i+1,1);
end

%%



