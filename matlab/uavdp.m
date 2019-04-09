clc 

uavP = data.vicon.uav.P.vicon;
uavt = data.vicon.uav.time;

ugvP = data.vicon.ugvk.P.vicon;
ugvt = data.vicon.ugvk.time;

for i = length(uavP):-1:1
   if mod(i,10)~=0
       uavP(i,:) = [];
       ugvP(i,:) = [];
       uavt(i) = [];
       ugvt(i) = [];
   end
end; clear i


cutoff = 85;
ugvP(ugvt>cutoff,:) = [];
uavP(uavt>cutoff,:) = [];
ugvt(ugvt>cutoff,:) = [];
uavt(uavt>cutoff,:) = [];

uavPA = [uavP(1,:);uavP];
uavPB = [uavP; uavP(end,:)];

ugvPA = [ugvP(1,:);ugvP];
ugvPB = [ugvP; ugvP(end,:)];

uavdP = (uavPB - uavPA);
uavdP(end,:) = [];
uavabsdP = abs(uavdP);

ugvdP = (ugvPB - ugvPA);
ugvdP(end,:) = [];
ugvabsdP = abs(ugvdP);

uavPabsdxy = sqrt(uavdP(:,1).^2 + uavdP(:,2).^2);
uavPabsxy = sqrt(uavP(:,1).^2 + uavP(:,2).^2);

ugvPabsdxy = sqrt(ugvdP(:,1).^2 + ugvdP(:,2).^2);
ugvPabsxy = sqrt(ugvP(:,1).^2 + ugvP(:,2).^2);

disp([[meta.date meta.run] ' UAV total travel distance: ' num2str(sum(uavPabsdxy))])
disp([[meta.date meta.run] ' UGV total travel distance: ' num2str(sum(ugvPabsdxy))])

%% figure(1500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1500); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(uavt, uavP(:,1), '.', 'displayname', 'x'); catch; end
    try plot(uavt, uavP(:,2), '.', 'displayname', 'y'); catch; end
    try plot(uavt, uavP(:,3), '.', 'displayname', 'z'); catch; end
hold off
legend('toggle')


%% figure(1501); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1501); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(uavt, uavdP(:,1), '.', 'displayname', 'x'); catch; end
    try plot(uavt, uavdP(:,2), '.', 'displayname', 'y'); catch; end
    try plot(uavt, uavdP(:,3), '.', 'displayname', 'z'); catch; end
hold off
legend('toggle')

%% figure(1502); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1502); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(uavt, uavabsdP(:,1), '.', 'displayname', 'x'); catch; end
    try plot(uavt, uavabsdP(:,2), '.', 'displayname', 'y'); catch; end
    try plot(uavt, uavabsdP(:,3), '.', 'displayname', 'z'); catch; end
hold off
legend('toggle')

%% figure(1503); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1503); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(uavt, uavPabsxy, '.', 'displayname', 'xy'); catch; end
hold off
legend('toggle')

%% figure(1504); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1504); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(ugvt, ugvP(:,1), '.', 'displayname', 'x'); catch; end
    try plot(ugvt, ugvP(:,2), '.', 'displayname', 'y'); catch; end
    try plot(ugvt, ugvP(:,3), '.', 'displayname', 'z'); catch; end
hold off
legend('toggle')


%% figure(1501); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1501); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(uavt, uavdP(:,1), '.', 'displayname', 'x'); catch; end
    try plot(uavt, uavdP(:,2), '.', 'displayname', 'y'); catch; end
    try plot(uavt, uavdP(:,3), '.', 'displayname', 'z'); catch; end
hold off
legend('toggle')

%% figure(1502); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1502); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(uavt, uavabsdP(:,1), '.', 'displayname', 'x'); catch; end
    try plot(uavt, uavabsdP(:,2), '.', 'displayname', 'y'); catch; end
    try plot(uavt, uavabsdP(:,3), '.', 'displayname', 'z'); catch; end
hold off
legend('toggle')

%% figure(1503); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
figure(1503); clf; current_fig = gcf; disp(['figure(' num2str(current_fig.Number) ') ..']); clear current_fig
hold on
    try plot(uavt, uavPabsxy, '.', 'displayname', 'xy'); catch; end
hold off
legend('toggle')

%%
clear A B ans uavP uavPA uavPB uavPabsxy uavabsdP uavdP uavt 