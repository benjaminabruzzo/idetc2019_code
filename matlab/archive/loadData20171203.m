function data = loadData(meta)
%% Load experiment run data
    current_dir = pwd;
    cd([meta.dataroot meta.date meta.run])
    debugprint(['loading data, ' meta.date meta.run])
    try [data.april] = loadrundotm('april', meta); catch; disp(['         issue loading april data']); end
    try [data.uavCon] = loadrundotm('uavCon', meta); catch; disp(['         issue loading uavCon']); end
    try [data.ugvStereo] = loadrundotm('ugvStereo', meta); catch; disp(['         issue loading ugvStereo']); end
    try [data.experiment] = loadrundotm('experiment', meta); catch; disp(['         issue loading experiment']); end
    try [data.ckfRecorder] = loadrundotm('ckfRecorder', meta); catch; disp(['         issue loading ckfRecorder']); end
    try [data.uavRecorder] = loadrundotm('uavRecorder', meta); catch; disp(['         issue loading uavRecorder']); end
    try   [data.ugvRecorder] = loadrundotm('ugvRecorder', meta); catch; disp(['         issue loading ugvRecorder']); end
    try  [data.ugvAutopilot] = loadrundotm('ugvAutopilot', meta); catch; disp(['         issue loading ugvAutopilot']); end
    try [data.ugvController] = loadrundotm('ugvController', meta); catch; disp(['         issue loading ugvController']); end
    
    % try [data.vo2_info] = loadrundotm('vo2_info', meta); catch; warning(['issue loading vo2_info']); end
    % try [data.vo2_odom] = loadrundotm('vo2_odom', meta); catch; warning(['issue loading vo2_info']); end
    % try [data.vo2_pose] = loadrundotm('vo2_pose', meta); catch; warning(['issue loading vo2_info']); end
    % try [data.droneCon] = loadrundotm('droneCon', meta); catch; warning(['issue loading april']); end
    % try [data.flyDrone] = loadrundotm('flyDrone', meta); catch; warning(['issue loading april']); end
    % try [data.ckfRecorder] = loadrundotm('ckfRecorder', meta); catch; warning(['issue loading april']); end
    % try [data.stateRecorder] = loadrundotm('stateRecorder', meta); catch; warning(['issue loading april']); end
    % try [data.offlineStereo] = loadrundotm('offlineStereo', meta); catch; warning(['issue loading offlineStereo']); end
    % try [data.markerRecorder] = loadrundotm('markerRecorder', meta); catch; warning(['issue loading april']); end
    vprint('Data Loaded')
    try [data] = reshapeUAV_UGV(data); catch; disp(['         issue reshaping UAV_UGV oneckf']); end
    %% This runs after the matfile check
%     data.ugvStereo.PostProc = ugvStereoPostProc(data.ugvStereo);
    
    try data = syncTimers(data, meta); catch; end
    data = loadViconData(data, meta); 
    
    %% spline the compass yaw to determine yaw drift    

try
    [data.uavRecorder.navdata.compassSpline, data.uavRecorder.navdata.vicon_yaw, data.uavRecorder.navdata.compassError] = ... 
            spliner(data.vicon.uav.time, data.vicon.uav.yaw.degrees, ...
            data.uavRecorder.navdata.time, data.uavRecorder.navdata.CompassYaw); 
catch; end

    cd(current_dir)
end

function [out] = loadrundotm(in, meta)
    here = pwd;
    root = [meta.dataroot meta.date meta.run '/'];
    mat_file = [in '_' meta.run '.mat'];
    m_file = [in '_' meta.run '.m'];
    cd(root);
    
    if exist(mat_file)
        disp(['    Loading ' mat_file ' ...'])
        load(mat_file);
    elseif exist(m_file, 'file')
        
        disp(['    Loading ' root m_file ' ...'])
        try
            eval([in '_prealloc_' meta.run])
        catch
            debugprint(['     ' in ': no preallocation found'])
        end
        eval([in '_' meta.run])
        if exist(in, 'var')
            vprint(['     Saving ' root mat_file ' ...'])
            save([root mat_file], in)
        end
    end
    out = eval(in);
    cd(here);
end
function [data] = reshapeUAV_UGV(data)
%%
    SimpleCells = {...
            'data.uavRecorder.oneckf.Rk'; ...
            'data.uavRecorder.oneckf.Kk'; ...
            'data.uavRecorder.oneckf.Qdk'; ...
            'data.uavRecorder.oneckf.Sk'; ...
            'data.uavRecorder.oneckf.Skinv'; ...
            'data.uavRecorder.oneckf.PriorCov'; ...
            'data.uavRecorder.oneckf.PosteriorCov'; ...
            'data.uavRecorder.oneckf.uav_Qdk'; ...
            'data.ugvRecorder.oneckf.ugv_Qdk'; ...
            };
    for n = 1:length(SimpleCells)
        try
            simplestring = [SimpleCells{n} '_re = reshape(' SimpleCells{n} ', size(' SimpleCells{n} ',1)*size(' SimpleCells{n} ',2),size(' SimpleCells{n} ',3),1);'];
            eval(simplestring);
        catch
            disp(['failed:: ' simplestring])
        end
    end

end
function data = syncTimers(data, meta)
%% synchronize ros times
    try
        vprint(['    Loading timers_' meta.run '.mat ... '])
        load([meta.dataroot meta.date meta.run '/timers_' meta.run '.mat']);
    catch
        timers.StartTime = [];
        timers.EndTime = [0];
        timers.SimpleCells = {...
            'data.april.marker_00.time'; ...
            'data.april.marker_01.time'; ...
            'data.april.marker_02.time'; ...
            'data.april.marker_03.time'; ...
            'data.april.marker_04.time'; ...
            'data.april.marker_05.time'; ...
            'data.april.marker_06.time'; ...
            'data.april.marker_07.time'; ...
            'data.april.marker_08.time'; ...
            'data.april.marker_09.time'; ...
            'data.april.marker_10.time'; ...
            'data.april.marker_11.time'; ...
            'data.april.marker_12.time'; ...
%             'data.experiment.uav.FlyTime'; ...
%             'data.experiment.ugv.time';...
            'data.uavCon.hover.Time';...
            'data.uavCon.time'; ...
%             'data.uavCon.picket.time'; ...
            'data.uavRecorder.est.time';...
%             'data.uavRecorder.ckf.time';...
            'data.uavRecorder.cmd.time';...
            'data.uavRecorder.navdata.time';...
%             'data.ugvRecorder.ckf.time';...
            'data.ugvRecorder.est.time';...
            'data.ugvRecorder.stereo.time';...
            'data.ugvRecorder.veltwist.time';...
            'data.ugvRecorder.wheel.time';...
            'data.ugvStereo.Guide.time';...
            'data.ugvStereo.Odom.time';...
            'data.ugvStereo.time'; ...
%             'data.vo2_info.header.stamp'; ...
%             'data.vo2_odom.header.stamp'; ...
%             'data.vo2_pose.header.stamp'; ...
            'data.uavRecorder.oneckf.time';...
            'data.ugvController.cmd.time';...
            'data.ugvAutopilot.cmd.time';...
            'data.ugvAutopilot.guide.time'; ...
            'data.ugvAutopilot.est.time';...
            };
    for n = 1:length(timers.SimpleCells)
        try
            disp([timers.SimpleCells{n} '[min max] = [' num2str(min(eval(timers.SimpleCells{n}))) '  ' num2str(max(eval(timers.SimpleCells{n}))) ']'])
        catch
        end
        try
            timers.StartTime = min([ timers.StartTime min(eval(timers.SimpleCells{n})) ] );
        catch
            disp(['does ' timers.SimpleCells{n} ' exist?'])
        end
    end
    
    disp(['    Saving timers_' meta.run '.mat ... '])
    save([meta.dataroot meta.date meta.run '/timers_' meta.run '.mat'], 'timers')
   
    end

    for n = 1:length(timers.SimpleCells)
        try
%             disp([timers.SimpleCells{n} ' = ' timers.SimpleCells{n} ' - timers.StartTime;']);
            eval([timers.SimpleCells{n} ' = ' timers.SimpleCells{n} ' - timers.StartTime;']);
            timers.EndTime = max([timers.EndTime max( eval(timers.SimpleCells{n}) ) ]);
        catch
        end
    end
    
    data.timers = timers;
end
function data = loadViconData(data, meta)
%% load in data from c++/ros
    try april = data.april; catch; end
    try uavCon = data.uavCon;catch; end
    try ugvStereo = data.ugvStereo;catch; end
    try experiment= data.experiment;catch; end
    try uavRecorder = data.uavRecorder;catch; end
    try ugvRecorder = data.ugvRecorder;catch; end
    try timers = data.timers;catch; end

%% Set csv file names

    [csvlist, april_csv_list] = getCSVlists(meta);

    if exist(['viconData_' meta.run '.mat'])
        tic;
        disp(['    Loading viconData_' meta.run '.mat'])
        load(['viconData_' meta.run '.mat']);
        disp('vicon.offset'); 
        disp(vicon.offset);
    else       
        clear vicon
        cd([meta.dataroot meta.date meta.run])

        disp(['    Loading vicon csv data ....'])
        tic;
        for csv_i = 1: length(csvlist)
            filename = ['/Users/benjamin/ros/data/'  meta.date 'csv/' csvlist{csv_i} meta.run '.csv'];
            try
                vicon.(genvarname(csvlist{csv_i})) = trackerQCSV(filename);
            catch
                warning(['    error with loading ' filename]);
                % vicon.(genvarname(csvlist{csv_i})) = trackerCSV(filename);
            end

        end
        clear csv_i filename

        try vicon.ugvc = vicon.create_01_; catch; end
        % update with short names (needs to be replaced with base serial id)
        csvlist = {...
            'april02_'; ...
            'april03_'; ...
            'red_'; ...
            'blue_'; ...
            'green_'; ...
            'ugvc'; ...
            'kobuki_02_'; ...
            };
    end
        
%% Set up "global" from ugv initial location
        try
            vicon.global.origin = vicon.kobuki_02_.P.vicon(1,:);
            vicon.global.orientation = mean(vicon.kobuki_02_.yaw.radians(1:100));  % radians
            vicon.global.Hvic2gl = wrapH(Rz(vicon.global.orientation),vicon.global.origin');
        catch; end
        try 
            vicon.global.origin = vicon.ugvc.P.vicon(1,:);
            vicon.global.orientation = mean(vicon.ugvc.yaw.radians(1:100));  % radians
            vicon.global.Hvic2gl = wrapH(Rz(vicon.global.orientation),vicon.global.origin');
        catch; end

        for i = 1:size(csvlist,1)
            try
    %             disp(['     Setting global origin for ' csvlist{i}])
                for j = 1:size(vicon.(matlab.lang.makeValidName(csvlist{i})).P.vicon,1)
                    vicon.(matlab.lang.makeValidName(csvlist{i})).P.global(j,:) = [...
                        (vicon.global.Hvic2gl * [vicon.(matlab.lang.makeValidName(csvlist{i})).P.vicon(j,:) 1]')'];
                end
                % radians?
                vicon.(matlab.lang.makeValidName(csvlist{i})).yaw.global = vicon.(matlab.lang.makeValidName(csvlist{i})).yaw.radians - vicon.global.orientation;
            catch
            end
        end

    
    

%% Calculate uav yaw from vicon data (in vicon frame?)

        try
            vicon.uav.BR = LEDextrema(vicon.blue_.P.vicon  - vicon.red_.P.vicon);
            vicon.uav.RG = LEDextrema(vicon.red_.P.vicon   - vicon.green_.P.vicon);
            vicon.uav.GB = LEDextrema(vicon.green_.P.vicon - vicon.blue_.P.vicon);
            [vicon] = yawfromviconRGB(vicon);
        catch
        end

%% calculate ugv (and globalrelative positions of markers
        try
            P_red = vicon.red_.P.vicon - vicon.kobuki_02_.P.vicon;
            P_blue = vicon.blue_.P.vicon - vicon.kobuki_02_.P.vicon;
            P_green = vicon.green_.P.vicon - vicon.kobuki_02_.P.vicon;
            for i = 1:size(P_red,1)
                vicon.red_.P.ugv(i,:) = vicon.kobuki_02_.R_vic2lo(:,:,i) * P_red(i,:)';
                vicon.blue_.P.ugv(i,:) = vicon.kobuki_02_.R_vic2lo(:,:,i) * P_blue(i,:)';
                vicon.green_.P.ugv(i,:) = vicon.kobuki_02_.R_vic2lo(:,:,i) * P_green(i,:)';
            end
            vicon.uav.P.ugv = 0.5*(vicon.red_.P.ugv + vicon.blue_.P.ugv);
    %         disp(['     Setting global origin for uav'])
            vicon.uav.P.global = 0.5*(vicon.red_.P.global + vicon.blue_.P.global);
            vicon.uav.yaw.global.radians = vicon.uav.yaw.radians - vicon.global.orientation;
            vicon.uav.yaw.global.degrees = vicon.uav.yaw.global.radians * 180/pi;
        catch
        end
        clear i P_blue P_green P_red

        try
            P_red = vicon.red_.P.vicon - vicon.ugvc.P.vicon;
            P_blue = vicon.blue_.P.vicon - vicon.ugvc.P.vicon;
            P_green = vicon.green_.P.vicon - vicon.ugvc.P.vicon;
            for i = 1:size(P_red,1)
                vicon.red_.P.ugv(i,:) = vicon.ugvc.R_vic2lo(:,:,i) * P_red(i,:)';
                vicon.blue_.P.ugv(i,:) = vicon.ugvc.R_vic2lo(:,:,i) * P_blue(i,:)';
                vicon.green_.P.ugv(i,:) = vicon.ugvc.R_vic2lo(:,:,i) * P_green(i,:)';
            end
            vicon.uav.P.ugv = 0.5*(vicon.red_.P.ugv + vicon.blue_.P.ugv);
    %         disp(['     Setting global origin for uav'])
            vicon.uav.P.global = 0.5*(vicon.red_.P.global + vicon.blue_.P.global);
            vicon.uav.yaw.global.radians = vicon.uav.yaw.radians - vicon.global.orientation;
            vicon.uav.yaw.global.degrees = vicon.uav.yaw.global.radians * 180/pi;
        catch
        end
        clear i P_blue P_green P_red
        
%% Calculate camera-frame positions of markers
        try
            Rcam2ugv = Rz(-pi/2)*Rx(-(90-20)*pi/180);
            try Hcam2ugv = [Rcam2ugv [ugvStereo.pgryaml.camXOffset ugvStereo.pgryaml.camYOffset ugvStereo.pgryaml.camZOffset]';[0 0 0 1]];catch;end
            try Hcam2ugv = ugvStereo.cam2ugv; catch; end
            H = invertH(Hcam2ugv);
            
            
            P_red = [vicon.red_.P.ugv ones(length(vicon.red_.P.ugv),1)];
            P_blue = [vicon.blue_.P.ugv ones(length(vicon.red_.P.ugv),1)];
            P_green = [vicon.green_.P.ugv ones(length(vicon.red_.P.ugv),1)];
            for i = 1:size(P_red,1)
                vicon.red_.P.cam(i,:) = H * P_red(i,:)';
                vicon.blue_.P.cam(i,:) = H * P_blue(i,:)';
                vicon.green_.P.cam(i,:) = H * P_green(i,:)';
            end
            vicon.uav.P.cam = 0.5*(vicon.red_.P.cam + vicon.blue_.P.cam);

        catch
        end
        clear i P_blue P_green P_red H

        try
            H = invertH(ugvStereo.cam2tb);
            P_red = [vicon.red_.P.ugv ones(length(vicon.red_.P.ugv),1)];
            P_blue = [vicon.blue_.P.ugv ones(length(vicon.red_.P.ugv),1)];
            P_green = [vicon.green_.P.ugv ones(length(vicon.red_.P.ugv),1)];
            for i = 1:size(P_red,1)
                vicon.red_.P.cam(i,:) = H * P_red(i,:)';
                vicon.blue_.P.cam(i,:) = H * P_blue(i,:)';
                vicon.green_.P.cam(i,:) = H * P_green(i,:)';
            end
            vicon.uav.P.cam = 0.5*(vicon.red_.P.cam + vicon.blue_.P.cam);

        catch
        end
        clear i P_blue P_green P_red H

%% Synchronize vicon data to experimental times
%     csvlist = {'kobuki_02_'; 'red_'; 'blue_'; 'green_'; 'april00_'};
    csvlist = {...
        'april02_'; ...
        'april03_'; ...
        'red_'; ...
        'blue_'; ...
        'green_'; ...
        'ugvc'; ...
        'kobuki_02_'; ...
        };

    % [vicon.offset] = ViconClockOffset(meta.date, meta.run, metvicon.uav.yaw.globala);
    % function [dt] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
    % [vicon.offset] = fitViconClockOffset(...
    %     ugvRecorder.stereo.time, ugvRecorder.stereo.Position_gl(:,1), ...
    %     vicon.uav.time, vicon.uav.P.global(:,1));
    
%% not sure what this is doing?
    try ugvRecorder.wheel.yaw = 2*acos(ugvRecorder.wheel.quat(:,4)); catch; end

    

if (~exist('experiment'))
    try
    %     disp('try kobuki_02_ linear pose')
    %         [vicon.offset] = fitViconClockOffset(...
    %             ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,1), ...
    %             vicon.kobuki_02_.time, vicon.kobuki_02_.P.global(:,1));
        disp('try kobuki_02_ angular pose')
            [vicon.offset] = fitViconClockOffset(...
                    ugvRecorder.wheel.time, 2*acos(ugvRecorder.wheel.quat(:,4)), ...
                    vicon.kobuki_02_.time, vicon.kobuki_02_.yaw.radians);
    catch
        disp('try ugvc')
        [vicon.offset] = fitViconClockOffset(...
                ugvRecorder.est.time, data.ugvRecorder.est.Position_gl(:,1), ...
                vicon.ugvc.time, vicon.ugvc.P.global(:,1));
    end
else
    %% Using just UAV yaw angle
    try 
        if(experiment.type.pull)
            disp('fitViconClockOffset using uav yaw')
            try
            [vicon.offset] = fitViconClockOffset(...
                ugvStereo.time, ugvStereo.uav.yaw_ugv, ...
                vicon.uav.time, vicon.uav.yaw.global.degrees);
            catch; end 

    %         try   
    %         [vicon.offset] = fitViconClockOffset(...
    %             ugvRecorder.stereo.time, ugvRecorder.stereo.yaw_ugv, ...
    %             vicon.uav.time, vicon.uav.yaw.global.degrees);
    %         catch; end

        end
    catch
    end
    %% Using just UAV altitude
    try
    if(experiment.type.abcde)
        disp('fitViconClockOffset using uav altitude')
        try
            [vicon.offset] = fitViconClockOffset(...
                ugvStereo.time, ugvStereo.uav.P_ugv(:,3), ...
                vicon.uav.time, vicon.uav.P.ugv(:,3));
        catch; end
    end
    catch
    end
    try
    if(experiment.type.picket)
        disp('fitViconClockOffset using uav altitude')
        try
            [vicon.offset] = fitViconClockOffset(...
                ugvStereo.time, ugvStereo.uav.P_ugv(:,3), ...
                vicon.uav.time, vicon.uav.P.ugv(:,3));
        catch; end
    end
    catch
    end
    %% for ugv linear pose  
    try
    if(experiment.type.ugvfore)
        disp(['ugvfore == true'])
%         try 
%             disp('trying vicon.kobuki_02_.time')
%             [vicon.offset] = fitViconClockOffset(...
%                 ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,1), ...
%                 vicon.kobuki_02_.time, vicon.kobuki_02_.P.global(:,1));
%         catch % crate base
%             disp('trying vicon.ugvc.time')
%             [vicon.offset] = fitViconClockOffset(...
%                 ugvRecorder.wheel.time, ugvRecorder.wheel.P_odom(:,1), ...
%                 vicon.ugvc.time, vicon.ugvc.P.global(:,1));            
%         end
        
            disp('trying vicon.red_.time')
            [vicon.offset] = fitViconClockOffset(...
                ugvStereo.time, ugvStereo.Red.P_cam(:,3), ...
                vicon.red_.time, vicon.red_.P.cam(:,3));

%         try plot(data.vicon.red_.time, data.vicon.red_.P.cam(:,3), 'r.', 'displayname', 'red marker vicon'); catch; end
%         try plot(data.ugvStereo.time, data.ugvStereo.Red.P_cam(:,3), 'ro', 'displayname', 'red marker pgr'); catch; end

    end
    catch
    end
    
    %% using april tag position
    try 
        if (experiment.type.april)
            disp(['april == true'])

            P_april02 = vicon.april02_.P.vicon - vicon.uav.P.vicon;
            [vicon.offset] = fitViconClockOffset(...
                april.marker_01.time, april.marker_01.Position_uav(:,3), ...
                vicon.april02_.time, P_april02(:,3));


        end
    catch
    end
    
end
    
%% If none of these work, set the offset to zero:
try
    disp(num2str(vicon.offset))
catch
    vicon.offset = 0;
end


    %%
    vicon.TimerCell = csvlist;
    try 
    for n = 1:length(vicon.TimerCell)
            timerstring = vicon.TimerCell{n};
            if isfield(vicon, timerstring)
                vicon.(genvarname(timerstring)).time = vicon.(genvarname(timerstring)).time+vicon.offset;
                timers.StartTime = min([timers.StartTime vicon.(genvarname(timerstring)).time(1)]);
                timers.EndTime = max([timers.EndTime vicon.(genvarname(timerstring)).time(1)]);
            end
        end
    catch; end
        try vicon.uav.time = vicon.red_.time; catch; end

    %% Compute splines for comparing vicon and ugv data:
%     vicon = viconsplines(vicon, ugvStereo, uavRecorder, ugvRecorder, meta);
    

    try
        vicon = viconsplines(vicon, ugvStereo, uavRecorder, ugvRecorder, meta);
    catch
        warning('viconsplines failed, skipping ..')
    end
    
    %% calculate april tag positions relative to uav
    for i = 1:length(april_csv_list)
        try 
            vicon = vicon_aprilfromUAV(vicon, april_csv_list{i}, data.april); 
        catch; end
    end



        %% Save mat file
        if exist('vicon', 'var')
            save(['viconData_' meta.run '.mat'], 'vicon')
        end



    %% Done.
        clear n tempstr timerstring csvlist
        cd(meta.root)

        
        data.timers = timers;
        data.vicon = vicon;
end
function [vicon] = yawfromviconRGB(vicon)
%% Simulate stereoObs.cpp function

    GmB = vicon.green_.P.vicon - vicon.blue_.P.vicon;
    RmB = vicon.red_.P.vicon - vicon.blue_.P.vicon;
    BmR = vicon.blue_.P.vicon - vicon.red_.P.vicon;
    
    
 
    %%
    for i = 1: length(GmB)
        znum = OrthoCross(GmB(i,:)', RmB(i,:)');
        normz = norm(znum);
        z_axis = znum/normz;
        
        normy = norm(BmR(i,:)');
        y_axis = BmR(i,:)'/normy;
        
        x_axis = OrthoCross(y_axis, z_axis);
        x_axis = x_axis/norm(x_axis);    

        vicon.uav.x_axis(i,:) = x_axis;
        vicon.uav.y_axis(i,:) = y_axis;
        vicon.uav.z_axis(i,:) = z_axis;
        vicon.uav.yaw.radians(i,1) = atan2(x_axis(2), x_axis(1)); %//heading angle in degrees in vicon frame
        vicon.uav.yaw.degrees(i,1) = atan2(x_axis(2), x_axis(1)) * 180 / pi; %//heading angle in degrees in vicon frame
        vicon.uav.Rugv2uav(:,:,i) = [x_axis(1:3)'; y_axis(1:3)'; z_axis(1:3)'];
    end
    vicon.uav.P.vicon = 0.5*(vicon.red_.P.vicon + vicon.blue_.P.vicon);
    try vicon.diff_yaw.uav_ugv = differentialYaw(vicon.kobuki_02_.x_axis, vicon.uav.x_axis(:,1:3)); catch; end
    try vicon.diff_yaw.uav_ugv = differentialYaw(vicon.ugvc.x_axis, vicon.uav.x_axis(:,1:3)); catch; end
    vicon.diff_yaw.mean_uav_ugv = mean(vicon.diff_yaw.uav_ugv);
    vicon.uav.time = vicon.red_.time;
    
    %% Use just mean positions
    GmBm = mean(vicon.green_.P.vicon) - mean(vicon.blue_.P.vicon);
    RmBm = mean(vicon.red_.P.vicon) - mean(vicon.blue_.P.vicon);
    BmRm = mean(vicon.blue_.P.vicon) - mean(vicon.red_.P.vicon);

    znumm = OrthoCross(GmBm', RmBm');
    normzm = norm(znumm);
    z_axism = znumm(1:3)/normzm;

    normym = norm(BmRm');
    y_axism = BmRm'/normym;
    
    x_axism = OrthoCross(y_axism, z_axism);
    x_axism = x_axism(1:3)/norm(x_axism);
    
    vicon.uav.x_axis_m = x_axism;
    vicon.uav.y_axis_m = y_axism;
    vicon.uav.z_axis_m = z_axism;
    vicon.uav.yaw_m.radians = atan2(x_axism(2), x_axism(1)); %//heading angle in degrees in vicon frame
    vicon.uav.yaw_m.degrees = atan2(x_axism(2), x_axism(1)) * 180 / pi; %//heading angle in degrees in vicon frame
    vicon.uav.Rugv2uav = [x_axism'; y_axism'; z_axism'];

end
function C = OrthoCross(A,B)
    C(1,1) =   ((A(2,1) * B(3,1)) - (A(3,1) * B(2,1)));
    C(2,1) = - ((A(1,1) * B(3,1)) - (A(3,1) * B(1,1)));
    C(3,1) =   ((A(1,1) * B(2,1)) - (A(2,1) * B(1,1)));
    C(4,1) = 0;
    normC = norm(C);
    C = C/normC;

end
function [dt] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
% function [dt] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
    % find vicon clock offset
    disp('[rostime(1) rostime(end)]');disp([rostime(1) rostime(end)])
    disp('[vicontime(1) vicontime(end)]');disp([vicontime(1) vicontime(end)])

    % coarse fit
    offset.viconspline = csapi(vicontime,vicondata);
    offset.time(1) = vicontime(1) - rostime(1);
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
    disp([' dt = ' num2str(dt)]);
end

% try
%     [data.uavRecorder.navdata.compassSpline, data.uavRecorder.navdata.vicon_yaw, data.uavRecorder.navdata.compassError] = ... 
%             spliner(data.vicon.uav.time, data.vicon.uav.yaw.degrees, ...
%             data.uavRecorder.navdata.time, data.uavRecorder.navdata.CompassYaw); 
% catch; end
% 

function [spline, vector_newtime, diff]  = spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector)
    %% spline data and compare to ref
    % timeofvector : time vector corresponding to vector that will be splined
    % vectorToBeSplined : data vector to be splined
    % timeOfRefVector : time vector corresponding to times which will be interpolated
    % referenceVector : data vector corresponding to reference time to compute diff

    spline = csapi(timeofvector,vectorToBeSplined);
    vector_newtime = fnval(spline, timeOfRefVector);
    diff = vector_newtime - referenceVector;
end
function vicon = viconsplines(vicon, ugvStereo, uavRecorder, ugvRecorder, meta)
%% Spline the ugv positions of markers
% try 


try
    [uavRecorder.navdata.compassSpline, ...
        data.uavRecorder.navdata.vicon_yaw, ...
        data.uavRecorder.navdata.compassError] = ... 
            spliner(data.vicon.uav.time, data.vicon.uav.yaw.degrees, ...
                    data.uavRecorder.navdata.time, data.uavRecorder.navdata.CompassYaw); 
catch; end



    vicon.red_.splines.P.ugv.x = csapi(vicon.red_.time,vicon.red_.P.ugv(:,1));
    vicon.red_.splines.P.ugv.y = csapi(vicon.red_.time,vicon.red_.P.ugv(:,2));
    vicon.red_.splines.P.ugv.z = csapi(vicon.red_.time,vicon.red_.P.ugv(:,3));
    vicon.red_.splines.P.ugv.x_ugvStereotime = fnval(vicon.red_.splines.P.ugv.x, ugvStereo.time);
    vicon.red_.splines.P.ugv.y_ugvStereotime = fnval(vicon.red_.splines.P.ugv.y, ugvStereo.time);
    vicon.red_.splines.P.ugv.z_ugvStereotime = fnval(vicon.red_.splines.P.ugv.z, ugvStereo.time);    
    vicon.red_.splines.P.ugv.x_diff = vicon.red_.splines.P.ugv.x_ugvStereotime - ugvStereo.Red.P_ugv(:,1);
    vicon.red_.splines.P.ugv.y_diff = vicon.red_.splines.P.ugv.y_ugvStereotime - ugvStereo.Red.P_ugv(:,2);
    vicon.red_.splines.P.ugv.z_diff = vicon.red_.splines.P.ugv.z_ugvStereotime - ugvStereo.Red.P_ugv(:,3);

    vicon.blue_.splines.P.ugv.x = csapi(vicon.blue_.time,vicon.blue_.P.ugv(:,1));
    vicon.blue_.splines.P.ugv.y = csapi(vicon.blue_.time,vicon.blue_.P.ugv(:,2));
    vicon.blue_.splines.P.ugv.z = csapi(vicon.blue_.time,vicon.blue_.P.ugv(:,3));
    vicon.blue_.splines.P.ugv.x_ugvStereotime = fnval(vicon.blue_.splines.P.ugv.x, ugvStereo.time);
    vicon.blue_.splines.P.ugv.y_ugvStereotime = fnval(vicon.blue_.splines.P.ugv.y, ugvStereo.time);
    vicon.blue_.splines.P.ugv.z_ugvStereotime = fnval(vicon.blue_.splines.P.ugv.z, ugvStereo.time);
    vicon.blue_.splines.P.ugv.x_diff = vicon.blue_.splines.P.ugv.x_ugvStereotime - ugvStereo.Blue.P_ugv(:,1);
    vicon.blue_.splines.P.ugv.y_diff = vicon.blue_.splines.P.ugv.y_ugvStereotime - ugvStereo.Blue.P_ugv(:,2);
    vicon.blue_.splines.P.ugv.z_diff = vicon.blue_.splines.P.ugv.z_ugvStereotime - ugvStereo.Blue.P_ugv(:,3);

    vicon.green_.splines.P.ugv.x = csapi(vicon.green_.time,vicon.green_.P.ugv(:,1));
    vicon.green_.splines.P.ugv.y = csapi(vicon.green_.time,vicon.green_.P.ugv(:,2));
    vicon.green_.splines.P.ugv.z = csapi(vicon.green_.time,vicon.green_.P.ugv(:,3));
    vicon.green_.splines.P.ugv.x_ugvStereotime = fnval(vicon.green_.splines.P.ugv.x, ugvStereo.time);
    vicon.green_.splines.P.ugv.y_ugvStereotime = fnval(vicon.green_.splines.P.ugv.y, ugvStereo.time);
    vicon.green_.splines.P.ugv.z_ugvStereotime = fnval(vicon.green_.splines.P.ugv.z, ugvStereo.time);
    vicon.green_.splines.P.ugv.x_diff = vicon.green_.splines.P.ugv.x_ugvStereotime - ugvStereo.Green.P_ugv(:,1);
    vicon.green_.splines.P.ugv.y_diff = vicon.green_.splines.P.ugv.y_ugvStereotime - ugvStereo.Green.P_ugv(:,2);
    vicon.green_.splines.P.ugv.z_diff = vicon.green_.splines.P.ugv.z_ugvStereotime - ugvStereo.Green.P_ugv(:,3);
% catch
% end

%%
    vicon.red_.splines.P.rms_diff = sqrt( ...
        vicon.red_.splines.P.ugv.x_diff.^2 + ...
        vicon.red_.splines.P.ugv.y_diff.^2 + ...
        vicon.red_.splines.P.ugv.z_diff.^2);
    vicon.blue_.splines.P.rms_diff = sqrt( ...
        vicon.blue_.splines.P.ugv.x_diff.^2 + ...
        vicon.blue_.splines.P.ugv.y_diff.^2 + ...
        vicon.blue_.splines.P.ugv.z_diff.^2);
    vicon.green_.splines.P.rms_diff = sqrt( ...
        vicon.green_.splines.P.ugv.x_diff.^2 + ...
        vicon.green_.splines.P.ugv.y_diff.^2 + ...
        vicon.green_.splines.P.ugv.z_diff.^2);

    vicon.red_.splines.P_ugv = [...
        vicon.red_.splines.P.ugv.x_ugvStereotime ...
        vicon.red_.splines.P.ugv.y_ugvStereotime ...
        vicon.red_.splines.P.ugv.z_ugvStereotime];
    vicon.blue_.splines.P_ugv = [...
        vicon.blue_.splines.P.ugv.x_ugvStereotime ...
        vicon.blue_.splines.P.ugv.y_ugvStereotime ...
        vicon.blue_.splines.P.ugv.z_ugvStereotime];
    vicon.green_.splines.P_ugv = [...
        vicon.green_.splines.P.ugv.x_ugvStereotime ...
        vicon.green_.splines.P.ugv.y_ugvStereotime ...
        vicon.green_.splines.P.ugv.z_ugvStereotime];

%% Spline the cam positions of markers

    vicon.red_.splines.P.cam.x = csapi(vicon.red_.time,vicon.red_.P.cam(:,1));
    vicon.red_.splines.P.cam.y = csapi(vicon.red_.time,vicon.red_.P.cam(:,2));
    vicon.red_.splines.P.cam.z = csapi(vicon.red_.time,vicon.red_.P.cam(:,3));
    vicon.red_.splines.P.cam.x_ugvStereotime = fnval(vicon.red_.splines.P.cam.x, ugvStereo.time);
    vicon.red_.splines.P.cam.y_ugvStereotime = fnval(vicon.red_.splines.P.cam.y, ugvStereo.time);
    vicon.red_.splines.P.cam.z_ugvStereotime = fnval(vicon.red_.splines.P.cam.z, ugvStereo.time);    
    vicon.red_.splines.P.cam.x_diff = vicon.red_.splines.P.cam.x_ugvStereotime - ugvStereo.Red.P_cam(:,1);
    vicon.red_.splines.P.cam.y_diff = vicon.red_.splines.P.cam.y_ugvStereotime - ugvStereo.Red.P_cam(:,2);
    vicon.red_.splines.P.cam.z_diff = vicon.red_.splines.P.cam.z_ugvStereotime - ugvStereo.Red.P_cam(:,3);

    vicon.blue_.splines.P.cam.x = csapi(vicon.blue_.time,vicon.blue_.P.cam(:,1));
    vicon.blue_.splines.P.cam.y = csapi(vicon.blue_.time,vicon.blue_.P.cam(:,2));
    vicon.blue_.splines.P.cam.z = csapi(vicon.blue_.time,vicon.blue_.P.cam(:,3));
    vicon.blue_.splines.P.cam.x_ugvStereotime = fnval(vicon.blue_.splines.P.cam.x, ugvStereo.time);
    vicon.blue_.splines.P.cam.y_ugvStereotime = fnval(vicon.blue_.splines.P.cam.y, ugvStereo.time);
    vicon.blue_.splines.P.cam.z_ugvStereotime = fnval(vicon.blue_.splines.P.cam.z, ugvStereo.time);
    vicon.blue_.splines.P.cam.x_diff = vicon.blue_.splines.P.cam.x_ugvStereotime - ugvStereo.Blue.P_cam(:,1);
    vicon.blue_.splines.P.cam.y_diff = vicon.blue_.splines.P.cam.y_ugvStereotime - ugvStereo.Blue.P_cam(:,2);
    vicon.blue_.splines.P.cam.z_diff = vicon.blue_.splines.P.cam.z_ugvStereotime - ugvStereo.Blue.P_cam(:,3);

    vicon.green_.splines.P.cam.x = csapi(vicon.green_.time,vicon.green_.P.cam(:,1));
    vicon.green_.splines.P.cam.y = csapi(vicon.green_.time,vicon.green_.P.cam(:,2));
    vicon.green_.splines.P.cam.z = csapi(vicon.green_.time,vicon.green_.P.cam(:,3));
    vicon.green_.splines.P.cam.x_ugvStereotime = fnval(vicon.green_.splines.P.cam.x, ugvStereo.time);
    vicon.green_.splines.P.cam.y_ugvStereotime = fnval(vicon.green_.splines.P.cam.y, ugvStereo.time);
    vicon.green_.splines.P.cam.z_ugvStereotime = fnval(vicon.green_.splines.P.cam.z, ugvStereo.time);
    vicon.green_.splines.P.cam.x_diff = vicon.green_.splines.P.cam.x_ugvStereotime - ugvStereo.Green.P_cam(:,1);
    vicon.green_.splines.P.cam.y_diff = vicon.green_.splines.P.cam.y_ugvStereotime - ugvStereo.Green.P_cam(:,2);
    vicon.green_.splines.P.cam.z_diff = vicon.green_.splines.P.cam.z_ugvStereotime - ugvStereo.Green.P_cam(:,3);

    vicon.red_.splines.P_cam = [...
        vicon.red_.splines.P.cam.x_ugvStereotime ...
        vicon.red_.splines.P.cam.y_ugvStereotime ...
        vicon.red_.splines.P.cam.z_ugvStereotime ...
        ones(length(vicon.red_.splines.P.cam.x_ugvStereotime),1)];
    vicon.blue_.splines.P_cam = [...
        vicon.blue_.splines.P.cam.x_ugvStereotime ...
        vicon.blue_.splines.P.cam.y_ugvStereotime ...
        vicon.blue_.splines.P.cam.z_ugvStereotime ...
        ones(length(vicon.red_.splines.P.cam.x_ugvStereotime),1)];
    vicon.green_.splines.P_cam = [...
        vicon.green_.splines.P.cam.x_ugvStereotime ...
        vicon.green_.splines.P.cam.y_ugvStereotime ...
        vicon.green_.splines.P.cam.z_ugvStereotime ...
        ones(length(vicon.red_.splines.P.cam.x_ugvStereotime),1)];

    
%% Map spline positions to pixels
% red_left_x = - Cx + ugvStereo.Red.left.rawxy(:,1) ;   
% red_left_y  = Cy - ugvStereo.Red.left.rawxy(:,2);
try 
    Cx = ugvStereo.Original.Cx; 
    Cy = ugvStereo.Original.Cy;
catch
    Cx = ugvStereo.Cx; 
    Cy = ugvStereo.Cy;
end


for i = 1:length(vicon.red_.splines.P_cam)
    uvw_l = ugvStereo.Projection.Left*vicon.red_.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
    uvw_r = ugvStereo.Projection.Right*vicon.red_.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
    vicon.red_.splines.left.rawxy(i,:) =  uvw_l;
    vicon.red_.splines.right.rawxy(i,:) = uvw_r;
    vicon.red_.splines.left.xy(i,:) =  [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
    vicon.red_.splines.right.xy(i,:) = [(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
    vicon.red_.splines.err_cam = vicon.red_.splines.P_cam(:,1:3) - ugvStereo.Red.P_cam;
    clear uvw_l uvw_r

    uvw_l = ugvStereo.Projection.Left*vicon.blue_.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
    uvw_r = ugvStereo.Projection.Right*vicon.blue_.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
    vicon.blue_.splines.left.rawxy(i,:) =  uvw_l;
    vicon.blue_.splines.right.rawxy(i,:) = uvw_r;
    vicon.blue_.splines.left.xy(i,:) = [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
    vicon.blue_.splines.right.xy(i,:) =[(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
    vicon.blue_.splines.err_cam = vicon.blue_.splines.P_cam(:,1:3) - ugvStereo.Blue.P_cam;
    clear uvw_l uvw_r
    
    uvw_l = ugvStereo.Projection.Left*vicon.green_.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
    uvw_r = ugvStereo.Projection.Right*vicon.green_.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
    vicon.green_.splines.left.rawxy(i,:) =  uvw_l;
    vicon.green_.splines.right.rawxy(i,:) = uvw_r;
    vicon.green_.splines.left.xy(i,:) = [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
    vicon.green_.splines.right.xy(i,:) =[(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
    vicon.green_.splines.err_cam = vicon.green_.splines.P_cam(:,1:3) - ugvStereo.Green.P_cam;
    clear uvw_l uvw_r
    
end

%% UAV position splines
    % stereotime in camera frame
    vicon.uav.splines.P.cam.x = csapi(vicon.uav.time,vicon.uav.P.cam(:,1));
    vicon.uav.splines.P.cam.y = csapi(vicon.uav.time,vicon.uav.P.cam(:,2));
    vicon.uav.splines.P.cam.z = csapi(vicon.uav.time,vicon.uav.P.cam(:,3));
    vicon.uav.splines.P.cam.x_uavstereotime = fnval(vicon.uav.splines.P.cam.x, ugvStereo.time);
    vicon.uav.splines.P.cam.y_uavstereotime = fnval(vicon.uav.splines.P.cam.y, ugvStereo.time);
    vicon.uav.splines.P.cam.z_uavstereotime = fnval(vicon.uav.splines.P.cam.z, ugvStereo.time);
    vicon.uav.splines.P.cam.x_diff = vicon.uav.splines.P.cam.x_uavstereotime - ugvStereo.uav.Obs_cam(:,1);
    vicon.uav.splines.P.cam.y_diff = vicon.uav.splines.P.cam.y_uavstereotime - ugvStereo.uav.Obs_cam(:,2);
    vicon.uav.splines.P.cam.z_diff = vicon.uav.splines.P.cam.z_uavstereotime - ugvStereo.uav.Obs_cam(:,3);
    
    % inertial time in global frame
    vicon.uav.splines.P.global.x = csapi(vicon.uav.time,vicon.uav.P.global(:,1));
    vicon.uav.splines.P.global.y = csapi(vicon.uav.time,vicon.uav.P.global(:,2));
    vicon.uav.splines.P.global.z = csapi(vicon.uav.time,vicon.uav.P.global(:,3));
    vicon.uav.splines.P.global.x_uavesttime = fnval(vicon.uav.splines.P.global.x, uavRecorder.est.time);
    vicon.uav.splines.P.global.y_uavesttime = fnval(vicon.uav.splines.P.global.y, uavRecorder.est.time);
    vicon.uav.splines.P.global.z_uavesttime = fnval(vicon.uav.splines.P.global.z, uavRecorder.est.time);
    vicon.uav.splines.P.global.x_diff = vicon.uav.splines.P.global.x_uavesttime - uavRecorder.est.Position_gl(:,1);
    vicon.uav.splines.P.global.y_diff = vicon.uav.splines.P.global.y_uavesttime - uavRecorder.est.Position_gl(:,2);
    vicon.uav.splines.P.global.z_diff = vicon.uav.splines.P.global.z_uavesttime - uavRecorder.est.Position_gl(:,3);

    vicon.uav.splines.P.global.x_uavStereotime = fnval(vicon.uav.splines.P.global.x, ugvStereo.time);
    vicon.uav.splines.P.global.y_uavStereotime = fnval(vicon.uav.splines.P.global.y, ugvStereo.time);
    vicon.uav.splines.P.global.z_uavStereotime = fnval(vicon.uav.splines.P.global.z, ugvStereo.time);
    vicon.uav.splines.P.global.x_diff_stereotime = vicon.uav.splines.P.global.x_uavStereotime - ugvStereo.uav.P_ugv(:,1);
    vicon.uav.splines.P.global.y_diff_stereotime = vicon.uav.splines.P.global.y_uavStereotime - ugvStereo.uav.P_ugv(:,2);
    vicon.uav.splines.P.global.z_diff_stereotime = vicon.uav.splines.P.global.z_uavStereotime - ugvStereo.uav.P_ugv(:,3);
    
%% ugv position splines
if (strcmp('20170914/', meta.date))
    disp('splining ugvc data')
    % spline of 
    vicon.ugvc.splines.P.global.x = csapi(vicon.ugvc.time,vicon.ugvc.P.global(:,1));
    vicon.ugvc.splines.P.global.y = csapi(vicon.ugvc.time,vicon.ugvc.P.global(:,2));
    vicon.ugvc.splines.P.global.z = csapi(vicon.ugvc.time,vicon.ugvc.P.global(:,3));
    
    %Vicon data in ugvRecorder time
    vicon.ugvc.splines.P.global.x_ugvcesttime = fnval(vicon.ugvc.splines.P.global.x, ugvRecorder.est.time);
    vicon.ugvc.splines.P.global.y_ugvcesttime = fnval(vicon.ugvc.splines.P.global.y, ugvRecorder.est.time);
    vicon.ugvc.splines.P.global.z_ugvcesttime = fnval(vicon.ugvc.splines.P.global.z, ugvRecorder.est.time);
    
    % Vicon position - est position
    vicon.ugvc.splines.P.global.x_diff = vicon.ugvc.splines.P.global.x_ugvcesttime - ugvRecorder.est.Position_gl(:,1);
    vicon.ugvc.splines.P.global.y_diff = vicon.ugvc.splines.P.global.y_ugvcesttime - ugvRecorder.est.Position_gl(:,2);
    vicon.ugvc.splines.P.global.z_diff = vicon.ugvc.splines.P.global.z_ugvcesttime - ugvRecorder.est.Position_gl(:,3);

    % yaw
    vicon.ugvc.splines.yaw.est = csapi(vicon.ugvc.time,vicon.ugvc.yaw.global); 
    vicon.ugvc.splines.yaw.yaw_ugvcesttime = fnval(vicon.ugvc.splines.yaw.est, ugvRecorder.est.time);
    vicon.ugvc.splines.yaw.est_err = vicon.ugvc.splines.yaw.yaw_ugvcesttime*180/pi - ugvRecorder.est.Yaw;
    
end


%% Compute yaw splines of vicon data
    % this is the yaw diff between vehicles from vicon data
    vicon.diff_yaw.spline = csapi(vicon.uav.time,vicon.diff_yaw.uav_ugv); 
    vicon.est_yaw.spline = csapi(vicon.uav.time,vicon.diff_yaw.uav_ugv);  
    % interpolate the data at the ugvStereo rate
if (strcmp('20170112/', meta.date))
    vicon.diff_yaw.splineyaw = fnval(vicon.diff_yaw.spline, ugvStereo.time);
    vicon.diff_yaw.spline_error = vicon.diff_yaw.splineyaw - ugvStereo.uav.yaw_ugv;
    vicon.diff_yaw.spline_mean = mean(vicon.diff_yaw.spline_error);
    vicon.diff_yaw.spline_std = std(vicon.diff_yaw.spline_error);
    vicon.diff_yaw.spline_var = var(vicon.diff_yaw.spline_error);

      
    vicon.est_yaw.splineyaw = fnval(vicon.est_yaw.spline, uavRecorder.est.time);
    vicon.est_yaw.spline_error = vicon.est_yaw.splineyaw - uavRecorder.est.Yaw;
    vicon.est_yaw.spline_mean = mean(vicon.est_yaw.spline_error);
    vicon.est_yaw.spline_std = std(vicon.est_yaw.spline_error);
    vicon.est_yaw.spline_var = var(vicon.est_yaw.spline_error);
else
    vicon.diff_yaw.splineyaw = fnval(vicon.diff_yaw.spline, ugvRecorder.stereo.time);
    vicon.diff_yaw.spline_error = vicon.diff_yaw.splineyaw - ugvRecorder.stereo.yaw_ugv;
    vicon.diff_yaw.spline_mean = mean(vicon.diff_yaw.spline_error);
    vicon.diff_yaw.spline_std = std(vicon.diff_yaw.spline_error);
    vicon.diff_yaw.spline_var = var(vicon.diff_yaw.spline_error);

      
    vicon.est_yaw.splineyaw = fnval(vicon.est_yaw.spline, uavRecorder.est.time);
    vicon.est_yaw.spline_error = vicon.est_yaw.splineyaw - uavRecorder.est.Yaw;
    vicon.est_yaw.spline_mean = mean(vicon.est_yaw.spline_error);
    vicon.est_yaw.spline_std = std(vicon.est_yaw.spline_error);
    vicon.est_yaw.spline_var = var(vicon.est_yaw.spline_error);
end
end
function vicon = vicon_aprilfromUAV(vicon, tag_id, down_data)
%%
    switch tag_id
        case 'april02_'
            marker_id = 'marker_01';
    end

    for i = 1:length(vicon.(matlab.lang.makeValidName(tag_id)).yaw.radians)

        R_april = Rz(vicon.(matlab.lang.makeValidName(tag_id)).yaw.radians(i));
        R_uav = Rz(vicon.uav.yaw.radians(i));
        R_april_in_uav = R_uav * R_april';
        vicon.(matlab.lang.makeValidName(tag_id)).yaw.uav(i,1) = -atan2(R_april_in_uav(1,2), R_april_in_uav(1,1));

        t = vicon.uav.P.vicon(i,1:3)';
        R = [...
            vicon.uav.x_axis(i,1:3);...
            vicon.uav.y_axis(i,1:3);...
            vicon.uav.z_axis(i,1:3)];
        H = wrapH(R,t);
        P = [vicon.(matlab.lang.makeValidName(tag_id)).P.vicon(i,:) 1]';
        vicon.(matlab.lang.makeValidName(tag_id)).P.uav(i,:) = H*P;
    end    
    
    [x_spline, x_vector_newtime, x_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(tag_id)).P.uav(:,1), ...
        down_data.(matlab.lang.makeValidName(marker_id)).time, ...
        down_data.(matlab.lang.makeValidName(marker_id)).Position_uav(:,1));
    
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.downtime = down_data.(matlab.lang.makeValidName(marker_id)).time;
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.x_spline_uav = x_spline;
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.x_uav_at_downtime = x_vector_newtime;
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.x_diff = x_diff;
    
    [y_spline, y_vector_newtime, y_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(tag_id)).P.uav(:,2), ...
        down_data.(matlab.lang.makeValidName(marker_id)).time, ...
        down_data.(matlab.lang.makeValidName(marker_id)).Position_uav(:,2));
    
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.y_spline_uav = y_spline;
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.y_uav_at_downtime = y_vector_newtime;
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.y_diff = y_diff;

    [z_spline, z_vector_newtime, z_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(tag_id)).P.uav(:,3), ...
        down_data.(matlab.lang.makeValidName(marker_id)).time, ...
        down_data.(matlab.lang.makeValidName(marker_id)).Position_uav(:,3));
    
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.z_spline_uav = z_spline;
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.z_uav_at_downtime = z_vector_newtime;
    vicon.(matlab.lang.makeValidName(tag_id)).P.spline.z_diff = z_diff;
    
    [yaw_spline, yaw_vector_newtime, yaw_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(tag_id)).yaw.uav, ...
        down_data.(matlab.lang.makeValidName(marker_id)).time, ...
        down_data.(matlab.lang.makeValidName(marker_id)).yaw_uav);

    vicon.(matlab.lang.makeValidName(tag_id)).yaw.spline.yaw_spline_uav = yaw_spline;
    vicon.(matlab.lang.makeValidName(tag_id)).yaw.spline.yaw_uav_at_downtime = yaw_vector_newtime;
    vicon.(matlab.lang.makeValidName(tag_id)).yaw.spline.yaw_diff = yaw_diff;
    
    
end
function [csvlist, april_csv_list] = getCSVlists(meta)
%%
files = dir([meta.dataroot meta.date 'csv']);

clear csvlist april_csv_list
csv_count = 1;
april_count = 1;
for i = 1:length(files)
    try 
        switch files(i).name(end-6:end-4)
            case meta.run
                if strcmp(files(i).name(1:4), 'hast')
                    %skip
                else
%                     disp(files(i).name)
                    csvlist{csv_count,1} = files(i).name(1:end-7);
                    csv_count = csv_count+1;
                end
            %keep track of april files seperately
            if strcmp(files(i).name(1:5), 'april')
%                 disp(['april file: ' files(i).name])
                april_csv_list{april_count} = files(i).name(1:end-7);
            end
        end
    end
end

end






