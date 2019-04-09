function data = loadData(meta)
%% Load experiment mfiles
clc
    current_dir = pwd;
    cd([meta.dataroot meta.date meta.run])
    debugprint(['loading data, ' meta.date meta.run])
    
    data_files = what;
    
    
    for i = 1:length(data_files.m)
%         disp(data_files.m{i})
        file_str = data_files.m{i}(1:end-6);
        if (length(file_str) > 8)
            if strcmp('prealloc',file_str(end-7:end))
%                 disp(file_str(end-7:end))
                continue; % this file is a prealloc file
            end
        end
        try 
            str_call = ['[data.' file_str '] = loadrundotm(file_str, meta);'];
%             disp(str_call);
            eval(str_call);
        catch
            disp(['    ** Issue loading ' file_str]); 
        end
    end
    
    vprint('Data Loaded')
    try [data] = reshapeUAV_UGV(data); catch; disp(['         issue reshaping UAV_UGV oneckf']); end
    try [data] = reshapeApril(data); catch; disp(['         issue reshaping slam data']); end
%% This runs after the matfile check
%     data.ugvStereo.PostProc = ugvStereoPostProc(data.ugvStereo);
    
    try data = syncTimers(data, meta); catch; end
    try data = loadViconData(data, meta); catch; end
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
    disp(root)
    if exist(mat_file)
        disp(['    Loading ' mat_file ' ...'])
        load(mat_file);
    elseif exist(m_file, 'file')
        
        disp(['    Loading ' root m_file ' ...'])
        try
            eval([in '_prealloc_' meta.run])
        catch
%             debugprint(['     ' in ': no preallocation found'])
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


function [data] = reshapeApril(data)
%%
    SimpleCells = {...
            'data.slam.uav.Ck'; ...
            'data.slam.tag_00.Ck'; ...
            'data.slam.tag_01.Ck'; ...
            'data.slam.tag_02.Ck'; ...
            'data.slam.tag_03.Ck'; ...
            'data.slam.tag_04.Ck'; ...
            'data.slam.tag_05.Ck'; ...
            'data.slam.tag_06.Ck'; ...
            'data.slam.tag_07.Ck'; ...
            };
    for n = 1:length(SimpleCells)
        try
            simplestring = [SimpleCells{n} '_re = reshape(' SimpleCells{n} ', size(' SimpleCells{n} ',1)*size(' SimpleCells{n} ',2),size(' SimpleCells{n} ',3),1);'];
            eval(simplestring);
        catch
%             disp(['failed:: ' simplestring])
        end
    end
    
    SimpleCellsRk = {...
            'data.slam.tag_00.Rk'; ...
            'data.slam.tag_01.Rk'; ...
            'data.slam.tag_02.Rk'; ...
            'data.slam.tag_03.Rk'; ...
            'data.slam.tag_04.Rk'; ...
            'data.slam.tag_05.Rk'; ...
            'data.slam.tag_06.Rk'; ...
            'data.slam.tag_07.Rk'; ...
            };
    
    
    sCk = size(data.slam.uav.Ck_re,2);
    for n = 1:length(SimpleCellsRk)
        try
            simplestring = [SimpleCellsRk{n} '_re = reshape(' SimpleCellsRk{n} ', size(' SimpleCellsRk{n} ',1)*size(' SimpleCellsRk{n} ',2),size(' SimpleCellsRk{n} ',3),1);'];
            eval(simplestring);
            sRk = size(eval([SimpleCellsRk{n} '_re']),2);
            delta = sCk - sRk;
            if delta > 0
                zeropad = zeros(size(data.slam.uav.Ck_re,1), delta);
            end
%             disp(['size(' SimpleCellsRk{n} '_re)= ' num2str(sRk)])
            
%             disp([SimpleCellsRk{n} '_re= [' SimpleCellsRk{n} '_re zeropad] '])
            eval([SimpleCellsRk{n} '_re= [' SimpleCellsRk{n} '_re zeropad];']);
            
%             size(data.slam.tag_03.Ck_re,2) - size(data.slam.tag_03.Rk_re,2);
            clear sRk delta zeropad
        catch
%             disp(['failed:: ' simplestring])
        end
    end
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
            
            'data.ckfRecorder.Rk'; ...
            'data.ckfRecorder.Kk'; ...
            'data.ckfRecorder.Qdk'; ...
            'data.ckfRecorder.uav.Qdk';...
            'data.ckfRecorder.ugv.Qdk';...
            'data.ckfRecorder.Sk'; ...
            'data.ckfRecorder.Skinv'; ...
            'data.ckfRecorder.PriorCov'; ...
            'data.ckfRecorder.PosteriorCov'; ...
            };
    for n = 1:length(SimpleCells)
        try
            simplestring = [SimpleCells{n} '_re = reshape(' SimpleCells{n} ', size(' SimpleCells{n} ',1)*size(' SimpleCells{n} ',2),size(' SimpleCells{n} ',3),1);'];
            eval(simplestring);
        catch
%             disp(['failed:: ' simplestring])
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
            'data.april.tag_00.time'; ...
            'data.april.tag_01.time'; ...
            'data.april.tag_02.time'; ...
            'data.april.tag_03.time'; ...
            'data.april.tag_04.time'; ...
            'data.april.tag_05.time'; ...
            'data.april.tag_06.time'; ...
            'data.april.tag_07.time'; ...
            'data.april.tag_08.time'; ...
            'data.april.tag_09.time'; ...
            'data.april.tag_10.time'; ...
            'data.april.tag_11.time'; ...
            'data.april.tag_12.time'; ...
            'data.experiment.uav.FlyTime'; ...
            'data.uavCon.hover.Time';...
            'data.uavCon.time'; ...
            'data.uavCon.cmd.time'; ...
            'data.uavRecorder.est.time';...
            'data.uavRecorder.cmd.time';...
            'data.uavRecorder.navdata.time';...
            'data.ugvRecorder.est.time';...
            'data.ugvRecorder.stereo.time';...
            'data.ugvRecorder.veltwist.time';...
            'data.ugvRecorder.wheel.time';...
            'data.ugvStereo.Guide.time';...
            'data.ugvStereo.Odom.time';...
            'data.ugvStereo.time'; ...
            'data.ugvStereo.leftraw.time'; ...
            'data.ugvStereo.rightraw.time'; ...
            'data.uavRecorder.oneckf.time';...
            'data.ugvController.cmd.time';...
            'data.ugvAutopilot.cmd.time';...
            'data.ugvAutopilot.guide.time'; ...
            'data.ugvAutopilot.est.time';...
            'data.ckfRecorder.time';...
            'data.slam.time';...
            'data.kobuki_logger.cmd_vel.time';...
            'data.kobuki_logger.local_plan.time';...
            'data.kobuki_logger.global_plan.time';...
            
            'data.ugvStereo.Red.solo.Time';...
            'data.ugvStereo.Blue.solo.Time';...
            'data.ugvStereo.Green.solo.Time';...
            };
        
        for n = 1:length(timers.SimpleCells)
            try
                if strcmp(timers.SimpleCells{n}, 'data.kobuki_logger.cmd_vel.time')
                    if (log10(data.kobuki_logger.cmd_vel.time(1)))>10
                        warning('data.kobuki_logger.cmd_vel.time = data.kobuki_logger.cmd_vel.time/1000000000')
                        data.kobuki_logger.cmd_vel.time = data.kobuki_logger.cmd_vel.time/1000000000;
                    end
                end
                disp(['    ' timers.SimpleCells{n} '[min max] = [' num2str(min(eval(timers.SimpleCells{n}))) '  ' num2str(max(eval(timers.SimpleCells{n}))) ']'])
            catch
            end
            try
                timers.StartTime = min([ timers.StartTime min(eval(timers.SimpleCells{n})) ] );
            catch
                disp(['   ** does ' timers.SimpleCells{n} ' exist?'])
            end
        end

        disp(['    Saving timers_' meta.run '.mat ... '])
        save([meta.dataroot meta.date meta.run '/timers_' meta.run '.mat'], 'timers')
    end % end try

    for n = 1:length(timers.SimpleCells)
        try
                if strcmp(timers.SimpleCells{n}, 'data.kobuki_logger.cmd_vel.time')
                    if (log10(data.kobuki_logger.cmd_vel.time(1)))>10
                        warning('data.kobuki_logger.cmd_vel.time = data.kobuki_logger.cmd_vel.time/1000000000')
                        data.kobuki_logger.cmd_vel.time = data.kobuki_logger.cmd_vel.time/1000000000;
                    end
                end
            eval([timers.SimpleCells{n} ' = ' timers.SimpleCells{n} ' - timers.StartTime;']);
            timers.EndTime = max([timers.EndTime max( eval(timers.SimpleCells{n}) ) ]);
        catch
        end
    end
    
    data.timers = timers;
end
function data = loadViconData(data, meta)
%% load in data from c++/ros
%     try april = data.april; catch; end
%     try uavCon = data.uavCon;catch; end
%     try ugvStereo = data.ugvStereo;catch; end
%     try experiment= data.experiment;catch; end
%     try uavRecorder = data.uavRecorder;catch; end
%     try ugvRecorder = data.ugvRecorder;catch; end
%     try timers = data.timers;catch; end    
%%    
datafields = fields(data);
    for i = 1:length(datafields)
        eval([datafields{i} ' = data.' datafields{i} ';'])
    end
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
            filename = [meta.dataroot  meta.date 'csv/' csvlist{csv_i} meta.run '.csv'];
            try
                vicon.(genvarname(csvlist{csv_i})) = trackerQCSV(filename);
            catch
                warning(['    error with loading ' filename]);
                % vicon.(genvarname(csvlist{csv_i})) = trackerCSV(filename);
            end

        end
        clear csv_i filename

        for i  = 1:size(csvlist,1)
            try
                if (strcmp('kobuki', csvlist{i}(1:6)))
                    disp(['     vicon.ugvk = vicon.(matlab.lang.makeValidName(' csvlist{i} '));'])
                    vicon.ugvk = vicon.(matlab.lang.makeValidName(csvlist{i}));
                end
            catch
%                 disp(['warning: vicon.ugvk = vicon.(matlab.lang.makeValidName(' csvlist{i} '));'])
            end
            try 
                if (strcmp('create', csvlist{i}(1:6)))
                    vicon.ugvc = vicon.(matlab.lang.makeValidName(csvlist{i}));
                end 
            catch
%                 disp(['warning: vicon.ugvk = vicon.(matlab.lang.makeValidName(' csvlist{i} '));'])
            end
        end    

%         try vicon.ugvc = vicon.create_01_; catch; end
%         try vicon.ugvk = vicon.kobuki_01_; catch; end
        % update with short names (needs to be replaced with base serial id)
        csvlist = {...
            'april02_'; ...
            'april03_'; ...
            'april06_'; ...
            'april07_'; ...
            'red_'; ...
            'blue_'; ...
            'green_'; ...
            'ugvc'; ...
            'ugvk'; ...
            };
    end
%% Set up "global" from ugv initial location
        i = 1;
        vicon.global.origin = vicon.ugvk.P.vicon(i,:);
        while isequal(vicon.global.origin, [0 0 0])
            i = i+1;
            vicon.global.origin = vicon.ugvk.P.vicon(i,:);
        end
        global_range = [i i+100];

        try
            vicon.global.origin = vicon.ugvk.P.vicon(global_range(1),:);
            vicon.global.orientation = mean(vicon.ugvk.yaw.radians(global_range(1):global_range(2)));  % radians
            vicon.global.Hvic2gl = wrapH(Rz(vicon.global.orientation),vicon.global.origin');
        catch; end
%         try 
%             vicon.global.origin = vicon.ugvc.P.vicon(global_range(1),:);
%             vicon.global.orientation = mean(vicon.ugvc.yaw.radians(global_range(1):global_range(2)));  % radians
%             vicon.global.Hvic2gl = wrapH(Rz(vicon.global.orientation),vicon.global.origin');
%         catch; end

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
        catch
        end
        try
            [vicon] = yawfromviconRGB(vicon);
        catch
            warning(['issue with [vicon] = yawfromviconRGB(vicon);']);
        end

%% calculate ugv (and globalrelative positions of markers
        try
            P_red = vicon.red_.P.vicon - vicon.ugvk.P.vicon;
            P_blue = vicon.blue_.P.vicon - vicon.ugvk.P.vicon;
            P_green = vicon.green_.P.vicon - vicon.ugvk.P.vicon;
            for i = 1:size(P_red,1)
                vicon.red_.P.ugv(i,:) = vicon.ugvk.R_vic2lo(:,:,i) * P_red(i,:)';
                vicon.blue_.P.ugv(i,:) = vicon.ugvk.R_vic2lo(:,:,i) * P_blue(i,:)';
                vicon.green_.P.ugv(i,:) = vicon.ugvk.R_vic2lo(:,:,i) * P_green(i,:)';
            end
            vicon.uav.P.ugv = 0.5*(vicon.red_.P.ugv + vicon.blue_.P.ugv);
    %         disp(['     Setting global origin for uav'])
            vicon.uav.P.global = 0.5*(vicon.red_.P.global + vicon.blue_.P.global);
            vicon.uav.yaw.global.radians = vicon.uav.yaw.radians - vicon.global.orientation;
            vicon.uav.yaw.global.degrees = vicon.uav.yaw.global.radians * 180/pi;
        catch
        end
        clear i P_blue P_green P_red

%         try
%             P_red = vicon.red_.P.vicon - vicon.ugvc.P.vicon;
%             P_blue = vicon.blue_.P.vicon - vicon.ugvc.P.vicon;
%             P_green = vicon.green_.P.vicon - vicon.ugvc.P.vicon;
%             for i = 1:size(P_red,1)
%                 vicon.red_.P.ugv(i,:) = vicon.ugvc.R_vic2lo(:,:,i) * P_red(i,:)';
%                 vicon.blue_.P.ugv(i,:) = vicon.ugvc.R_vic2lo(:,:,i) * P_blue(i,:)';
%                 vicon.green_.P.ugv(i,:) = vicon.ugvc.R_vic2lo(:,:,i) * P_green(i,:)';
%             end
%             vicon.uav.P.ugv = 0.5*(vicon.red_.P.ugv + vicon.blue_.P.ugv);
%     %         disp(['     Setting global origin for uav'])
%             vicon.uav.P.global = 0.5*(vicon.red_.P.global + vicon.blue_.P.global);
%             vicon.uav.yaw.global.radians = vicon.uav.yaw.radians - vicon.global.orientation;
%             vicon.uav.yaw.global.degrees = vicon.uav.yaw.global.radians * 180/pi;
%         catch
%         end
%         clear i P_blue P_green P_red   
%% Calculate camera-frame positions of markers
        try
            Rcam2ugv = Rz(-pi/2)*Rx(-(90-20)*pi/180);
%             Rcam2ugv = Rz(-pi/2)*Rx(-(90-21.5)*pi/180);
            try 
                Hcam2ugv = [Rcam2ugv [ugvStereo.pgryaml.camXOffset ugvStereo.pgryaml.camYOffset ugvStereo.pgryaml.camZOffset]';[0 0 0 1]];
            catch
            end
            try 
                Hcam2ugv = ugvStereo.cam2ugv; 
            catch
            end % this will overwrite H becuase it is encoded in the ros/c++ data
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

%         try
%             H = invertH(ugvStereo.cam2tb);
%             P_red = [vicon.red_.P.ugv ones(length(vicon.red_.P.ugv),1)];
%             P_blue = [vicon.blue_.P.ugv ones(length(vicon.red_.P.ugv),1)];
%             P_green = [vicon.green_.P.ugv ones(length(vicon.red_.P.ugv),1)];
%             for i = 1:size(P_red,1)
%                 vicon.red_.P.cam(i,:) = H * P_red(i,:)';
%                 vicon.blue_.P.cam(i,:) = H * P_blue(i,:)';
%                 vicon.green_.P.cam(i,:) = H * P_green(i,:)';
%             end
%             vicon.uav.P.cam = 0.5*(vicon.red_.P.cam + vicon.blue_.P.cam);
% 
%         catch
%         end
%         clear i P_blue P_green P_red H
%% calculate april tag positions relative to uav
    for i = 1:length(april_csv_list)
        try 
            vicon = vicon_aprilfromUAV(vicon, april_csv_list{i}, data.april); 
        catch
            disp(['   ** vicon_aprilfromUAV(vicon, ' april_csv_list{i} ', data.april); failed.'])
        end
        
        
    end
%% Synchronize vicon data to experimental times
%     csvlist = {'ugvk'; 'red_'; 'blue_'; 'green_'; 'april00_'};
    csvlist = {...
        'april02_'; ...
        'april03_'; ...
        'april06_'; ...
        'april07_'; ...
        'red_'; ...
        'blue_'; ...
        'green_'; ...
        'ugvc'; ...
        'ugvk'; ...
        };

    % [vicon.offset] = ViconClockOffset(meta.date, meta.run, metvicon.uav.yaw.globala);
    % function [dt] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
    % [vicon.offset] = fitViconClockOffset(...
    %     ugvRecorder.stereo.time, ugvRecorder.stereo.Position_gl(:,1), ...
    %     vicon.uav.time, vicon.uav.P.global(:,1));
    
% I think this code is used to generate one of the vecotr used to compute offset correlations
    try 
        x = ugvRecorder.wheel.quat(:,1);
        y = ugvRecorder.wheel.quat(:,2);
        z = ugvRecorder.wheel.quat(:,3);
        w = ugvRecorder.wheel.quat(:,4);
        ugvRecorder.wheel.yaw = -atan2(2*(x.*y - z.*w ) , 1 - 2*(y.^2 + z.^2)); %atan2(2,1);
        clear x y z w
    catch; end

% set vicon time using vicon.offset
    if (exist('april'))
        [vicon.syncoffset] = viconTimerSync(ugvRecorder, vicon, ugvStereo, april, uavRecorder);
    else
        april = 1;
        [vicon.syncoffset] = viconTimerSync(ugvRecorder, vicon, ugvStereo, april, uavRecorder);
        clear april
    end
    vicon.offset = vicon.syncoffset.dt_i;
    disp(['vicon time offset: ' num2str(vicon.offset)])
    
    % If none of these work, set the offset to zero:
    % try
    %     disp(num2str(vicon.offset))
    % catch
    %     vicon.offset = 0;
    % end


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
%% Compute splines for comparing data:
%     vicon = viconsplines(vicon, ugvStereo, uavRecorder, ugvRecorder, meta);
    
    try 
        vicon = viconsplines(vicon, ugvStereo, uavRecorder, ugvRecorder, meta); 
    catch
        warning('viconsplines failed, skipping ..'); 
    end
    try
        for i=1:length(ugvStereo.uav.P_ugv)
            ugvStereo.uav.P_gl(i,:) = (vicon.ugvk.splines.H_stereotime.lo2gl(:,:,i)*[ugvStereo.uav.P_ugv(i,:) 1]')';
        end
    catch
        disp('ugvStereo.uav.P_gl failed')
    end
    
    try 
        tagnames = fieldnames(data.april);
    catch
        disp(['   ** tagnames = fieldnames(data.april); failed.'])
    end
    
    csvk = 1;
    try 
        for i = 1:length(tagnames)
            try
              aprilOrd{csvk,1} = tagnames{i}(end-1:end);
              csvk = csvk+1;
            catch
            end 
        end
    catch
        disp(['   ** for i = 1:length(tagnames) failed.'])
    end

    try 
        for i = 1:length(tagnames)

            try
                vicon = vicon_aprilMeasError_uav(vicon, ['april' aprilOrd{i} '_'], data.april, tagnames{i});
            catch; warning(['vicon_aprilfromUAVspline ' ['april' aprilOrd{i} '_'] 'failed, skipping ..']); end

            try
                vicon = vicon_aprilfromSLAMspline(vicon, ['april' aprilOrd{i} '_'], data.slam, tagnames{i}); 
            catch; warning(['vicon_aprilfromSLAMspline ' ['april' aprilOrd{i} '_'] 'failed, skipping ..']); end

            try
                vicon = vicon_aprilMeasError_gl(vicon, ['april' aprilOrd{i} '_'], data.april, tagnames{i}); 
            catch; warning(['vicon_aprilMeasError ' ['april' aprilOrd{i} '_'] 'failed, skipping ..']); end
        end
    catch
        disp(['   ** for i = 1:length(tagnames) failed.'])
    end

    try vicon = vicon_uavfromSLAMspline(vicon, data.slam); catch; warning('vicon_uavfromSLAMspline failed, skipping ..'); end
    
    try vicon = vicon_SplineGatorboard(vicon, ugvStereo, caldata); catch; warning('vicon_SplineGatorboard failed, skipping ..'); end
%% Save mat file
        if exist('vicon', 'var')
            save(['viconData_' meta.run '.mat'], 'vicon')
        end
%% Done.
        clear n tempstr timerstring csvlist
        cd(meta.root)
        
        data.timers = timers;
        data.vicon = vicon;
        data.ugvStereo = ugvStereo;
end
function [out] = viconTimerSync(ugvRecorder, vicon, ugvStereo, april, uavRecorder)
% [out] = viconTimerSync(data.ugvRecorder, data.vicon, data.ugvStereo, data.april)

    i = 1;

        
    try
        [offset.dt(i,1), offset.mindiff(i,1)] = fitViconClockOffset(...
                ugvRecorder.stereo.time, ugvRecorder.stereo.Position_gl(:,2), ...
                vicon.uav.time, vicon.uav.P.global(:,2));
            i = i + 1;
    catch
    end

% 
%     try
%         [offset.dt(i,1), offset.mindiff(i,1)] = fitViconClockOffset(...
%                 ugvStereo.time, ugvStereo.uav.yaw_ugv, ...
%                 vicon.uav.time, vicon.uav.yaw.global.degrees);
%     catch
%     end
% 

    try
        [offset.dt(i,1), offset.mindiff(i,1)] = fitViconClockOffset(...
                uavRecorder.est.time, uavRecorder.est.Position_gl(:,1), ...
                vicon.uav.time, vicon.uav.P.ugv(:,1));
            i = i + 1;
    catch
    end

%     try % uav yaw angle vicon vs estimated
%         [offset.dt(i,1), offset.mindiff(i,1)] = fitViconClockOffset(...
%                 ugvRecorder.est.time, ugvRecorder.est.Yaw, ...
%                 vicon.ugvk.time, vicon.ugvk.yaw.global*180/pi);
%             i = i + 1;
%     catch
%     end
% 
    
    % try % uav yaw angle vicon vs estimated
    %     [offset.dt(i,1), offset.mindiff(i,1)] = fitViconClockOffset(...
    %             uavRecorder.est.time, uavRecorder.est.Yaw, ...
    %             vicon.uav.time, vicon.uav.yaw.global.degrees);
    %         i = i + 1;
    % catch
    % end


    try 
        [v,i] = min(offset.mindiff);
        offset.dt_i = offset.dt(i);
    catch
        offset.dt_i = 0;
    end
    
    
    out = offset;
end
function [dt, varargout] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
%% function [dt] = fitViconClockOffset(rostime, rosdata, vicontime, vicondata)
    % find vicon clock offset
%     disp('[rostime(1) rostime(end) length(rostime)]');disp([rostime(1) rostime(end) length(rostime)])
%     disp('[vicontime(1) vicontime(end) ]');disp([vicontime(1) vicontime(end) ])

    % inflate vicon data set with zeros
%     t_length = 10+abs((vicontime(end) - rostime(end)));
% %     t_length = 60 + (vicontime(end) - rostime(end));
%     t_vicon_dt = vicontime(end) - vicontime(end-1);
%     t_pad_vicon = vicontime(end) + [t_vicon_dt:t_vicon_dt:t_length];
% 
%     d_vicon_zeros = zeros(size(t_pad_vicon));
% 
%     vicontime = [t_pad_vicon'; vicontime; t_pad_vicon'];
%     vicondata = [d_vicon_zeros'; vicondata; d_vicon_zeros'];
% 


%%

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
%     disp([' dt = ' num2str(dt) ', mindiff = ' num2str(min_diff/length(rostime))]);
    
    varargout{1} = min_diff/length(rostime);
    
    
% %%
% figure(1); clf
% % hold on
% %     plot(offset.rostime, rosdata, 'o', 'displayname', 'rosdata')
% %     plot(vicontime, vicondata, '.', 'displayname', 'rosdata')
% % hold off    
end
function [vicon] = yawfromviconRGB(vicon)
%% Simulate stereoObs.cpp function

    GmB = vicon.green_.P.vicon - vicon.blue_.P.vicon;
    RmB = vicon.red_.P.vicon - vicon.blue_.P.vicon;
    BmR = vicon.blue_.P.vicon - vicon.red_.P.vicon;
 
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
    try vicon.diff_yaw.uav_ugv = differentialYaw(vicon.ugvk.x_axis, vicon.uav.x_axis(:,1:3)); catch; end
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
function vicon = viconsplines(vicon, ugvStereo, uavRecorder, ugvRecorder, meta)
%% Spline the ugv positions of uav markers

    try
    [uavRecorder.navdata.compassSpline, ...
        uavRecorder.navdata.vicon_yaw, ...
        uavRecorder.navdata.compassError] = ... 
            spliner(vicon.uav.time, vicon.uav.yaw.degrees, ...
                    uavRecorder.navdata.time, uavRecorder.navdata.CompassYaw); 
    catch; end

    spline = csapi(vicon.ugvk.time,vicon.ugvk.yaw.global);
        vector_newtime = fnval(spline, ugvRecorder.stereo.time);
            vicon.ugvk.atStereoTime.yaw.global = vector_newtime;

    

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
%% RGB spline rms error
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
%% Spline the cam positions of uav markers

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


ugvStereo.Projection.Right(1,4) = -155;

for i = 1:length(vicon.red_.splines.P_cam)
    uvw_l = ugvStereo.Projection.Left*vicon.red_.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
    uvw_r = ugvStereo.Projection.Right*vicon.red_.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
    vicon.red_.splines.left.rawxy(i,:) =  uvw_l;
    vicon.red_.splines.right.rawxy(i,:) = uvw_r;
    vicon.red_.splines.left.xy(i,:) =  [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
    vicon.red_.splines.right.xy(i,:) = [(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
    vicon.red_.splines.err_cam = vicon.red_.splines.P_cam(:,1:3) - ugvStereo.Red.P_cam;
    vicon.red_.splines.err_ugv = vicon.red_.splines.P_ugv(:,1:3) - ugvStereo.Red.P_ugv;
    vicon.red_.splines.err_ugv_rms = sqrt([...
        vicon.red_.splines.err_ugv(:,1).*vicon.red_.splines.err_ugv(:,1) + ...
        vicon.red_.splines.err_ugv(:,2).*vicon.red_.splines.err_ugv(:,2) + ...
        vicon.red_.splines.err_ugv(:,3).*vicon.red_.splines.err_ugv(:,3)]);
    clear uvw_l uvw_r

    uvw_l = ugvStereo.Projection.Left*vicon.blue_.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
    uvw_r = ugvStereo.Projection.Right*vicon.blue_.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
    vicon.blue_.splines.left.rawxy(i,:) =  uvw_l;
    vicon.blue_.splines.right.rawxy(i,:) = uvw_r;
    vicon.blue_.splines.left.xy(i,:) = [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
    vicon.blue_.splines.right.xy(i,:) =[(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
    vicon.blue_.splines.err_cam = vicon.blue_.splines.P_cam(:,1:3) - ugvStereo.Blue.P_cam;
    vicon.blue_.splines.err_ugv = vicon.blue_.splines.P_ugv(:,1:3) - ugvStereo.Blue.P_ugv;
    vicon.blue_.splines.err_ugv_rms = sqrt([...
        vicon.blue_.splines.err_ugv(:,1).*vicon.blue_.splines.err_ugv(:,1) + ...
        vicon.blue_.splines.err_ugv(:,2).*vicon.blue_.splines.err_ugv(:,2) + ...
        vicon.blue_.splines.err_ugv(:,3).*vicon.blue_.splines.err_ugv(:,3)]);
    clear uvw_l uvw_r
    
    uvw_l = ugvStereo.Projection.Left*vicon.green_.splines.P_cam(i,:)'; uvw_l = uvw_l'/uvw_l(3);
    uvw_r = ugvStereo.Projection.Right*vicon.green_.splines.P_cam(i,:)'; uvw_r = uvw_r'/uvw_r(3);
    vicon.green_.splines.left.rawxy(i,:) =  uvw_l;
    vicon.green_.splines.right.rawxy(i,:) = uvw_r;
    vicon.green_.splines.left.xy(i,:) = [(Cx-uvw_l(1)) (uvw_l(2)-Cy) uvw_l(3)];
    vicon.green_.splines.right.xy(i,:) =[(Cx-uvw_r(1)) (uvw_r(2)-Cy) uvw_r(3)];
    vicon.green_.splines.err_cam = vicon.green_.splines.P_cam(:,1:3) - ugvStereo.Green.P_cam;
    vicon.green_.splines.err_ugv = vicon.green_.splines.P_ugv(:,1:3) - ugvStereo.Green.P_ugv;
    vicon.green_.splines.err_ugv_rms = sqrt([...
        vicon.green_.splines.err_ugv(:,1).*vicon.green_.splines.err_ugv(:,1) + ...
        vicon.green_.splines.err_ugv(:,2).*vicon.green_.splines.err_ugv(:,2) + ...
        vicon.green_.splines.err_ugv(:,3).*vicon.green_.splines.err_ugv(:,3)]);
    clear uvw_l uvw_r
    
end
%% UAV pose splines
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

    % stereotime in global  frame
    vicon.uav.splines.P.global.x_uavStereotime = fnval(vicon.uav.splines.P.global.x, ugvStereo.time);
    vicon.uav.splines.P.global.y_uavStereotime = fnval(vicon.uav.splines.P.global.y, ugvStereo.time);
    vicon.uav.splines.P.global.z_uavStereotime = fnval(vicon.uav.splines.P.global.z, ugvStereo.time);
    vicon.uav.splines.P.global.x_diff_stereotime = vicon.uav.splines.P.global.x_uavStereotime - ugvStereo.uav.P_ugv(:,1);
    vicon.uav.splines.P.global.y_diff_stereotime = vicon.uav.splines.P.global.y_uavStereotime - ugvStereo.uav.P_ugv(:,2);
    vicon.uav.splines.P.global.z_diff_stereotime = vicon.uav.splines.P.global.z_uavStereotime - ugvStereo.uav.P_ugv(:,3);
    
    vicon.uav.splines.P.global.xyz = [vicon.uav.splines.P.global.x_uavStereotime vicon.uav.splines.P.global.y_uavStereotime vicon.uav.splines.P.global.z_uavStereotime];
%% ugv position splines
if (strcmp('20170914/', meta.date))
    disp('   splining ugvc data')
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
else
    disp('   splining ugvk data')
    % spline of 
    vicon.ugvk.splines.P.global.x = csapi(vicon.ugvk.time,vicon.ugvk.P.global(:,1));
    vicon.ugvk.splines.P.global.y = csapi(vicon.ugvk.time,vicon.ugvk.P.global(:,2));
    vicon.ugvk.splines.P.global.z = csapi(vicon.ugvk.time,vicon.ugvk.P.global(:,3));
    
    %Vicon data in ugvRecorder time
    vicon.ugvk.splines.P.global.x_ugvkesttime = fnval(vicon.ugvk.splines.P.global.x, ugvRecorder.est.time);
    vicon.ugvk.splines.P.global.y_ugvkesttime = fnval(vicon.ugvk.splines.P.global.y, ugvRecorder.est.time);
    vicon.ugvk.splines.P.global.z_ugvkesttime = fnval(vicon.ugvk.splines.P.global.z, ugvRecorder.est.time);
    vicon.ugvk.splines.P.global.x_ugvkstereotime = fnval(vicon.ugvk.splines.P.global.x, ugvStereo.time);
    vicon.ugvk.splines.P.global.y_ugvkstereotime = fnval(vicon.ugvk.splines.P.global.y, ugvStereo.time);
    vicon.ugvk.splines.P.global.z_ugvkstereotime = fnval(vicon.ugvk.splines.P.global.z, ugvStereo.time);
    
    % Vicon position - est position
    vicon.ugvk.splines.P.global.x_diff = vicon.ugvk.splines.P.global.x_ugvkesttime - ugvRecorder.est.Position_gl(:,1);
    vicon.ugvk.splines.P.global.y_diff = vicon.ugvk.splines.P.global.y_ugvkesttime - ugvRecorder.est.Position_gl(:,2);
    vicon.ugvk.splines.P.global.z_diff = vicon.ugvk.splines.P.global.z_ugvkesttime - ugvRecorder.est.Position_gl(:,3);

    % yaw
    vicon.ugvk.splines.yaw.est = csapi(vicon.ugvk.time,vicon.ugvk.yaw.global); 
    vicon.ugvk.splines.yaw.yaw_ugvkesttime = fnval(vicon.ugvk.splines.yaw.est, ugvRecorder.est.time);
    vicon.ugvk.splines.yaw.est_err = vicon.ugvk.splines.yaw.yaw_ugvkesttime*180/pi - ugvRecorder.est.Yaw;
    vicon.ugvk.splines.yaw.yaw_ugvkstereotime = fnval(vicon.ugvk.splines.yaw.est, ugvStereo.time);
    
    for i = 1:length(vicon.ugvk.splines.yaw.yaw_ugvkstereotime)
        theta = vicon.ugvk.splines.yaw.yaw_ugvkstereotime(i);
        t = [vicon.ugvk.splines.P.global.x_ugvkstereotime(i) ...
             vicon.ugvk.splines.P.global.y_ugvkstereotime(i) ...
             vicon.ugvk.splines.P.global.z_ugvkstereotime(i)]';
        vicon.ugvk.splines.H_stereotime.lo2gl(:,:,i) = wrapH(Rz(theta)', -t);
    end
end
%% Compute yaw splines of vicon data

    vicon.uav.splines.yaw.global.degrees = csapi(vicon.uav.time, vicon.uav.yaw.global.degrees);
    vicon.uav.splines.yaw.global.degrees_stereotime = fnval(vicon.uav.splines.yaw.global.degrees, ugvStereo.time);
    vicon.uav.splines.yaw.global.error_stereotime =  vicon.uav.splines.yaw.global.degrees_stereotime - ugvStereo.uav.yaw_ugv;

    vicon.diff_yaw.spline = csapi(vicon.uav.time,vicon.diff_yaw.uav_ugv); 
    vicon.est_yaw.spline = csapi(vicon.uav.time, vicon.uav.yaw.global.degrees);

    vicon.diff_yaw.splineyaw = fnval(vicon.diff_yaw.spline, ugvRecorder.stereo.time);
    vicon.diff_yaw.spline_error = ugvRecorder.stereo.yaw_ugv - vicon.diff_yaw.splineyaw;
    vicon.diff_yaw.spline_mean = mean(vicon.diff_yaw.spline_error);
    vicon.diff_yaw.spline_std = std(vicon.diff_yaw.spline_error);
    vicon.diff_yaw.spline_var = var(vicon.diff_yaw.spline_error);

    vicon.est_yaw.splineyaw = fnval(vicon.est_yaw.spline, uavRecorder.est.time);
    vicon.est_yaw.spline_error = uavRecorder.est.Yaw - vicon.est_yaw.splineyaw;
    vicon.est_yaw.spline_mean = mean(vicon.est_yaw.spline_error);
    vicon.est_yaw.spline_std = std(vicon.est_yaw.spline_error);
    vicon.est_yaw.spline_var = var(vicon.est_yaw.spline_error);

    
    spline = csapi(vicon.uav.time,vicon.uav.yaw.global.degrees);
    vector_newtime = fnval(spline, uavRecorder.est.time);
    vicon.uav.splines.yaw.global.deg_atEstTime = vector_newtime;
    vicon.uav.yaw.global.degrees_error = uavRecorder.est.Yaw - vicon.uav.splines.yaw.global.deg_atEstTime;
end
function vicon = vicon_aprilMeasError_uav(vicon, vicon_tag_id, down_data, down_tag_id)
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.downtime = down_data.(matlab.lang.makeValidName(down_tag_id)).time;
    
    [x_spline, x_vector_newtime, x_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.uav(:,1), ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasPosition_uav(:,1));
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.x_uav_spline = x_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.x_uav_at_downtime = x_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.x_uav_diff = x_diff;
    
    [y_spline, y_vector_newtime, y_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.uav(:,2), ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasPosition_uav(:,2));
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.y_uav_spline = y_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.y_uav_at_downtime = y_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.y_uav_diff = y_diff;

    [z_spline, z_vector_newtime, z_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.uav(:,3), ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasPosition_uav(:,3));
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.z_uav_spline = z_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.z_uav_at_downtime = z_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.z_uav_diff = z_diff;
    
    [yaw_spline, yaw_vector_newtime, yaw_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.uav, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasYaw_uav);
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.spline.yaw_uav_spline = yaw_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.spline.yaw_uav_at_downtime = yaw_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.spline.yaw_uav_diff = yaw_diff;
    
    
end
function vicon = vicon_aprilfromSLAMspline(vicon, vicon_tag_id, slam_data, slam_tag_id)
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.time = slam_data.time;
    
    [x_spline, x_vector_newtime, x_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.global(:,1), ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.time, ...
        slam_data.(matlab.lang.makeValidName(slam_tag_id)).est.p.global(:,1));

    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.x_spline_global = x_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.x_at_slamtime = x_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.x_diff = x_diff;
    
    [y_spline, y_vector_newtime, y_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.global(:,2), ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.time, ...
        slam_data.(matlab.lang.makeValidName(slam_tag_id)).est.p.global(:,2));
    
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.y_spline_global = y_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.y_at_slamtime = y_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.y_diff = y_diff;

    [z_spline, z_vector_newtime, z_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.global(:,3), ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.time, ...
        slam_data.(matlab.lang.makeValidName(slam_tag_id)).est.p.global(:,3));
    
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.z_spline_global = z_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.z_at_slamtime = z_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.z_diff = z_diff;
    
    [yaw_spline, yaw_vector_newtime, yaw_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.global_deg, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.time, ...
        slam_data.(matlab.lang.makeValidName(slam_tag_id)).est.yaw.global);

    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.yaw_spline_global = yaw_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.yaw_at_slamtime = yaw_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).slam.spline.yaw_diff = yaw_diff;
end
function vicon = vicon_uavfromSLAMspline(vicon, slam_data)
    vicon.uav.slam.time = slam_data.time;
        
    [x_spline, x_vector_newtime, x_diff]  = spliner(...
        vicon.uav.time, vicon.uav.P.global(:,1), ...
        vicon.uav.slam.time, slam_data.uav.est.p.global(:,1));
    
    vicon.uav.slam.spline.x_spline_global = x_spline;
    vicon.uav.slam.spline.x_at_slamtime = x_vector_newtime;
    vicon.uav.slam.spline.x_diff = x_diff;
    
    [y_spline, y_vector_newtime, y_diff]  = spliner(...
        vicon.uav.time, vicon.uav.P.global(:,2), ...
        vicon.uav.slam.time, slam_data.uav.est.p.global(:,2));
    
    vicon.uav.slam.spline.y_spline_global = y_spline;
    vicon.uav.slam.spline.y_at_slamtime = y_vector_newtime;
    vicon.uav.slam.spline.y_diff = y_diff;

    [z_spline, z_vector_newtime, z_diff]  = spliner(...
        vicon.uav.time, vicon.uav.P.global(:,3), ...
        vicon.uav.slam.time, slam_data.uav.est.p.global(:,3));
    
    vicon.uav.slam.spline.z_spline_global = z_spline;
    vicon.uav.slam.spline.z_at_slamtime = z_vector_newtime;
    vicon.uav.slam.spline.z_diff = z_diff;
    
    [yaw_spline, yaw_vector_newtime, yaw_diff]  = spliner(...
        vicon.uav.time, vicon.uav.yaw.global.degrees, ...
        vicon.uav.slam.time, slam_data.uav.est.yaw.global);

    vicon.uav.slam.spline.yaw_spline_global = yaw_spline;
    vicon.uav.slam.spline.yaw_at_slamtime = yaw_vector_newtime;
    vicon.uav.slam.spline.yaw_diff = yaw_diff;
end
function vicon = vicon_aprilfromUAV(vicon, vicon_tag_id, down_data)
%%
%     switch tag_id
%         case 'april02_'
%             marker_id = 'marker_01';
%     end

    marker_id = ['tag_' vicon_tag_id(6:7)];

    for i = 1:length(vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.radians)

        R_april = Rz(vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.radians(i));
        R_uav = Rz(vicon.uav.yaw.radians(i));
        R_april_in_uav = R_uav * R_april';

        t = vicon.uav.P.vicon(i,1:3)';
        R = [...
            vicon.uav.x_axis(i,1:3);...
            vicon.uav.y_axis(i,1:3);...
            vicon.uav.z_axis(i,1:3)];
        H = wrapH(R,t);
        P = [vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.vicon(i,:) 1]';
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.uav(i,:) = H*P;
        
        
        % vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.uav(i,1) = -atan2(R_april_in_uav(1,2), R_april_in_uav(1,1));
        % This line was what I had been using up until 20180201, then I changed it to the following:
        % vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.uav(i,1) = 180*atan2(R_april_in_uav(1,2), R_april_in_uav(1,1))/pi;
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.uav(i,1) = -180*atan2(R_april_in_uav(1,2), R_april_in_uav(1,1))/pi;
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.uav_rad(i,1) = -atan2(R_april_in_uav(1,2), R_april_in_uav(1,1));
        
    end 
    
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.global_deg = ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.global*180/pi;
    

    
    
end
function [csvlist, april_csv_list] = getCSVlists(meta)
    %%
    
    files = dir([meta.dataroot meta.date 'csv']);
    clear csvlist april_csv_list
    april_csv_list = [];
    csvlist = [];
    csv_count = 1;
    april_count = 1;
    for i = 1:length(files)
        try 
            switch files(i).name(end-6:end-4)
                case meta.run
                    if strcmp(files(i).name(1:4), 'hast')
                        %skip
                    else
                        disp(files(i).name)
                        csvlist{csv_count,1} = files(i).name(1:end-7);
                        csv_count = csv_count+1;
                    end
                %keep track of april files seperately
                if strcmp(files(i).name(1:5), 'april')
    %                 disp(['april file: ' files(i).name])
                    april_csv_list{april_count} = files(i).name(1:end-7);
                    april_count = april_count+1;
                end
            end
        end
    end    
end
function vicon = vicon_aprilMeasError_gl(vicon, vicon_tag_id, down_data, down_tag_id)
%     vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.downtime = down_data.(matlab.lang.makeValidName(down_tag_id)).time;
    
    [x_spline, x_vector_newtime, x_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.global(:,1), ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasPosition_gl(:,1));
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.x_gl_spline = x_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.x_gl_at_downtime = x_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.x_gl_diff = x_diff;
    
    [y_spline, y_vector_newtime, y_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.global(:,2), ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasPosition_gl(:,2));
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.y_gl_spline = y_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.y_gl_at_downtime = y_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.y_gl_diff = y_diff;

    [z_spline, z_vector_newtime, z_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.global(:,3), ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasPosition_gl(:,3));
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.z_gl_spline = z_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.z_gl_at_downtime = z_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).P.spline.z_gl_diff = z_diff;
    
    [yaw_spline, yaw_vector_newtime, yaw_diff]  = spliner(...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).time, ...
        vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.global_deg, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).time, ...
        down_data.(matlab.lang.makeValidName(down_tag_id)).MeasYaw_gl);
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.spline.yaw_gl_spline = yaw_spline;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.spline.yaw_gl_at_downtime = yaw_vector_newtime;
    vicon.(matlab.lang.makeValidName(vicon_tag_id)).yaw.spline.yaw_gl_diff = yaw_diff;
    
    
end
function vicon = vicon_SplineGatorboard(vicon, ugvStereo, caldata)
%%
%     vicon = data.vicon;
%     ugvStereo = data.ugvStereo;
%     caldata = data.caldata
%%  UGV in vicon frame
% spline q
    spline = csapi(vicon.ugvk.time,vicon.ugvk.q_i(:,1));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.ugvk.atRawTime.q_i(:,1) = vector_newtime;
    spline = csapi(vicon.ugvk.time,vicon.ugvk.q_i(:,2));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.ugvk.atRawTime.q_i(:,2) = vector_newtime;
    spline = csapi(vicon.ugvk.time,vicon.ugvk.q_i(:,3));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.ugvk.atRawTime.q_i(:,3) = vector_newtime;
    spline = csapi(vicon.ugvk.time,vicon.ugvk.q_i(:,4));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.ugvk.atRawTime.q_i(:,4) = vector_newtime;
% spline P    
    spline = csapi(vicon.ugvk.time,vicon.ugvk.P.vicon(:,1));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.ugvk.atRawTime.P.vicon(:,1) = vector_newtime;
    spline = csapi(vicon.ugvk.time,vicon.ugvk.P.vicon(:,2));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.ugvk.atRawTime.P.vicon(:,2) = vector_newtime;
    spline = csapi(vicon.ugvk.time,vicon.ugvk.P.vicon(:,3));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.ugvk.atRawTime.P.vicon(:,3) = vector_newtime;
    
% compute DCM and H for ugv in vicon frame at stereoraw time
    for i = 1:length(vector_newtime)
        q = vicon.ugvk.atRawTime.q_i(i,:);
        RH = QwedgeDCM(q); %dcm converts the vicon orientation into the ugv orientation
        vicon.ugvk.atRawTime.dcm(:,:,i) = RH;
        t = vicon.ugvk.atRawTime.P.vicon(i,:)';
        vicon.ugvk.atRawTime.H_i(:,:,i) = [RH [RH*(-t)]; [0 0 0 1]];
    end
    
%%  gatorboard in vicon frame
% spline q
    spline = csapi(vicon.gatorboard_.time,vicon.gatorboard_.q_i(:,1));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.gatorboard_.atRawTime.q_i(:,1) = vector_newtime;
    spline = csapi(vicon.gatorboard_.time,vicon.gatorboard_.q_i(:,2));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.gatorboard_.atRawTime.q_i(:,2) = vector_newtime;
    spline = csapi(vicon.gatorboard_.time,vicon.gatorboard_.q_i(:,3));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.gatorboard_.atRawTime.q_i(:,3) = vector_newtime;
    spline = csapi(vicon.gatorboard_.time,vicon.gatorboard_.q_i(:,4));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.gatorboard_.atRawTime.q_i(:,4) = vector_newtime;
% spline P    
    spline = csapi(vicon.gatorboard_.time,vicon.gatorboard_.P.vicon(:,1));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.gatorboard_.atRawTime.P.vicon(:,1) = vector_newtime;
    spline = csapi(vicon.gatorboard_.time,vicon.gatorboard_.P.vicon(:,2));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.gatorboard_.atRawTime.P.vicon(:,2) = vector_newtime;
    spline = csapi(vicon.gatorboard_.time,vicon.gatorboard_.P.vicon(:,3));vector_newtime = fnval(spline, ugvStereo.leftraw.time);
    vicon.gatorboard_.atRawTime.P.vicon(:,3) = vector_newtime;
    
% compute DCM and H for ugv in vicon frame at stereoraw time
    for i = 1:length(vector_newtime)
        q = vicon.gatorboard_.atRawTime.q_i(i,:);
        RH = QwedgeDCM(q); %dcm converts the vicon orientation into the ugv orientation
        vicon.gatorboard_.atRawTime.dcm(:,:,i) = RH;
        t = vicon.gatorboard_.atRawTime.P.vicon(i,:)';
        vicon.gatorboard_.atRawTime.H_i(:,:,i) = [RH [RH*(-t)]; [0 0 0 1]];
    end
    
%% Compute H_cam2ugv left and right

for i = 1:length(vector_newtime)
    vicon.ugvk.atRawTime.H_Lcam2ugv(:,:,i) = ...
        vicon.ugvk.atRawTime.H_i(:,:,i)*...
        invertH(vicon.gatorboard_.atRawTime.H_i(:,:,i))*...
        invertH(eval(['caldata.left.H.H' num2str(i)]))* Rzh(pi);

    vicon.ugvk.atRawTime.H_Rcam2ugv(:,:,i) = ...
        vicon.ugvk.atRawTime.H_i(:,:,i)*...
        invertH(vicon.gatorboard_.atRawTime.H_i(:,:,i))*...
        invertH(eval(['caldata.right.H.H' num2str(i)]))* Rzh(pi);
end


    vicon.ugvk.atRawTime.H_Lcam2ugv_mean = mean(vicon.ugvk.atRawTime.H_Lcam2ugv,3);
    vicon.ugvk.atRawTime.H_Rcam2ugv_mean = mean(vicon.ugvk.atRawTime.H_Rcam2ugv,3);
    vicon.ugvk.atRawTime.H_Lcam2ugv_std = std(vicon.ugvk.atRawTime.H_Lcam2ugv,0,3);
    vicon.ugvk.atRawTime.H_Rcam2ugv_std = std(vicon.ugvk.atRawTime.H_Rcam2ugv,0,3);
    vicon.ugvk.atRawTime.H_stereoCenter = (vicon.ugvk.atRawTime.H_Lcam2ugv_mean + vicon.ugvk.atRawTime.H_Rcam2ugv_mean)./2;
    
end


