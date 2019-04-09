disp('running clas.m')
% set(0, 'DefaultFigureVisible', 'off');
% set(0, 'DefaultFigureVisible', 'on');

%% meta.date = '20190313/'; % gazebo testing uavSLAM
    meta.date = '20190404/'; % 
    meta.run = '001'; 
    
    %% define meta data
% meta.dataroot = '/media/benjamin/devsdb/hast/data/';
% meta.dataroot = '/Users/benjamin/hast/data/';
meta.dataroot = '/home/benjamin/ros/data/';
meta.saveplots = false;
meta.saveirosplots = false;
meta.saveplotroot = ['/home/benjamin/ros/data/' [meta.date meta.run] '/'];

%% data = loadData(meta);
meta.ugv_leader = "ugv1_";
meta.ugv_leader_stereo = "ugv1Stereo";
data = loadSlamData(meta);

%% plotclas
% [data, meta] = plotclas_func(data, meta);

for i = 1:length(data.uavSLAM.april.landmarksOrdered{end})
    data.uavSLAM.april.tagsDetected{1,i} = sprintf('tag_%02d', data.uavSLAM.april.landmarksOrdered{1,end}(i));
end; clear i

% vicon_fields = fields(data.vicon);
% for i = 1:length(vicon_fields)
%     if length(vicon_fields{i})==8
%         if (strcmp('april', vicon_fields{i}))
%         end
%     end
% end; clear i vicon_fields

[data, meta] = plotslam_func(data, meta);



for i = 2:length(data.uavSLAM.correction)
    uav.aug(i-1,:) = data.uavSLAM.augmentedState_k{i-1}(1,1:4);
    uav.correction(i,:) = data.uavSLAM.correction{i}(1:4,1)';
    uav.vk(i,:) = data.uavSLAM.vk{i}(1:4)';
end