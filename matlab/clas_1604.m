disp('running clas.m')
set(0, 'DefaultFigureVisible', 'off');
% meta.date = '20170112/' through 20180226; experiments using april tags in ros message format
% meta.date = '20170112/'; % icra2018 data
% meta.run = '033'; % 
% meta.date = '20170211/'; experiments using april tags in ros message format
% meta.date = '20170211/'; % icra2018 data
% meta.run = '043'; % 
% meta.date = '20170913/'; experiments using april tags in ros message format
% meta.date = '20170913/'; % icra2018 data
% meta.run = '033'; % 
% meta.date = '20170913/'; experiments using april tags in ros message format
% meta.date = '20170914/'; % icra2018 data
% meta.run = '041'; % 
% meta.date = '20171215/'; 
% meta.date = '20171215/'; % actual experiment data
% meta.date = '20171216/'; % simulated data of 20171215
% meta.run = '001'; % overhead lights, ugvauto
% meta.run = '002'; % overhead lights, pull
% meta.run = '003'; % stage lights, pull
% meta.run = '004'; % stage lights, ugvauto
% meta.run = '005'; % stage lights, ugvauto
% meta.run = '006'; % stage lights, ugvauto
% meta.run = '007'; % stage lights, ugvauto
% meta.run = '008'; % stage lights, ugvauto
% meta.run = '009'; % stage lights, ugvauto
% meta.run = '010'; % stage lights, ugvauto
% meta.run = '011'; % stage lights, ugvauto
% meta.date = '20171218/'; experiments using april tags in ros message format
% meta.date = '20171218/'; % actual experiment data
% meta.date = '20171219/'; % simulated data of 20171218
% meta.run = '001'; % stage lights, ugvauto, bad liftoff
% meta.run = '002'; % stage lights, ugvauto, bad liftoff
% meta.run = '003'; % stage lights, ugvauto, weird flight 
% meta.run = '004'; % stage lights, ugvauto, weird flight
% meta.run = '012'; % stage lights, ugvauto
% meta.date = '20171221/'; experiments using april tags in ros message format
% meta.date = '20171221/'; % actual experiment data
% meta.date = '20171222/'; % simulated data of 20171221*************
% meta.run = '001'; % stage lights, square maneuver
% meta.run = '002'; % stage lights, octagon maneuver
% meta.run = '003'; % stage lights, short maneuver
% meta.run = '004'; % stage lights, octagon maneuver
% meta.run = '005'; % stage lights, octagon maneuver
% meta.run = '006'; % stage lights, octagon maneuver **********
% meta.date = '20180108/'; experiments using april tags in ros message format
% meta.date = '20180108/'; % actual experiment data
% meta.date = '20171222/'; % simulated data of 20171221
% meta.run = '001'; % stage lights, zigzag maneuver
% meta.run = '002'; % stage lights, zigzag maneuver
% meta.run = '003'; % stage lights, zigzag maneuver
% meta.run = '004'; % stage lights, ugvfore : fixed slam bug
% meta.date = '20180119/'; experiments using april tags in ros message format
% meta.date = '20180119/'; % actual experiment data
% meta.date = '20171222/'; % simulated data of 20171221
% meta.run = '001'; % overhead lights, no motions, testing slam
% meta.run = '002'; % overhead lights, no motions, testing slam
% meta.run = '003'; % overhead lights, no motions, testing slam
% meta.run = '005'; % overhead lights, no motions, testing slam
% meta.run = '010'; % overhead lights, ugv action server plots course around landmarks, testing slam, ugv wires disturbed lanmark locations
% meta.date = '20180126/'; experiments using april tags in ros message format
% meta.date = '20180126/'; % actual experiment data
% meta.date = '20180128/'; % simulated data of 20180126
% meta.run = '004'; % stage lights
% meta.run = '005'; % stage lights
% meta.run = '009'; % stage lights, suspended uav
% meta.date = '20180129/'; experiments using april tags in ros message format
% meta.date = '20180129/'; % actual experiment data
% meta.date = '20180130/'; % simulated data of 20180126 (mars)
% meta.date = '20180131/'; % simulated data of 20180126 (venus)
% meta.run = '004'; % stage lights, flying uav, ugv drives into landmark
% meta.run = '006'; % stage lights, flying uav, ugv is static
% meta.run = '007'; % stage lights, flying uav, ugv is static
% 007 is missing some vicon data, so only on this one will we be chopping data down to a more
% bitesize set
% meta.date = '20180201/'; experiments using april tags in ros message format
% meta.date = '20180201/'; % actual experiment data
% meta.date = '17180201/'; % simulated data of 20180201 (on kobuki01)
% meta.date = '19180201/'; % simulated data of 20180201 (on venus)
% meta.run = '021'; % stage lights, flying uav, ugv drives into landmark
% meta.date = '20180202/'; experiments using april tags in ros message format
% meta.date = '20180202/'; % actual experiment data
% meta.date = '19180202/'; % simulated data of 20180126 (venus)
% meta.run = '007'; % stage lights, flying uav, ugv drives into landmark
% meta.date = '20180205/'; experiments using april tags in ros message format
% meta.date = '20180205/'; % actual experiment data
% meta.run = '015'; % stage lights, flying uav, ugv drives into wall
% meta.date = '20180208/'; experiments using april tags in ros message format
% meta.date = '20180208/'; % actual experiment data
% meta.run = '007'; % stage lights, flying uav
% meta.run = '012'; %  
% meta.date = '20180209/'; experiments using april tags in ros message format % Good experiment
% meta.date = '20180209/'; % actual experiment data
% meta.run = '003'; % Good experiment
% meta.date = '20180212/'; experiments using april tags in ros message format % 003 is the experiment that actually worked and was turned into a video
% meta.date = '20180212/'; % actual experiment data
% meta.run = '003'; % 003 is the experiment that actually worked and was turned into a video
% meta.run = '004'; %
% meta.run = '006'; % something goes terribly wrong with plotting figure 3 in run 006
% meta.date = '20180215/'; experiments using april tags in ros message format
% meta.date = '20180215/'; % actual experiment data
% meta.run = '011'; % 
% meta.date = '20180216/'; experiments using april tags in ros message format
% meta.date = '20180216/'; % actual experiment data
% meta.run = '005'; % 
% meta.date = '20180220/'; experiments using april tags in ros message format
% meta.date = '20180220/'; % actual experiment data
% meta.run = '001'; % calibration by steps
% meta.run = '002'; % calibration by steps
% meta.run = '004'; % 
% meta.run = '005'; % 
% meta.date = '20180222/'; experiments using april tags in ros message format
% meta.date = '20180222/'; % actual experiment data
% meta.run = '001'; % 
% meta.date = '20180223/'; experiments using april tags in ros message format
% meta.date = '20180223/'; % actual experiment data
% meta.run = '006'; % 006 had the best performance of the UGV following a trajectory while the UAV was suspended from the tripod
% meta.run = '015'; % /
% meta.date = '20180224/'; experiments using april tags in ros message format
% meta.date = '20180224/'; % actual experiment data
% meta.run = '004'; % abcde experiment (linear trajecotries)
% meta.run = '006'; % abcde experiment (step trajecotries)
% meta.run = '011'; % teleop calibration data
% meta.date = '20180226/'; experiments using april tags in ros message format
% meta.date = '20180226/'; % actual experiment data
% meta.run = '003'; % 
% meta.date = '20180316/'; experiments using april tags in ros message format
% meta.date = '20180316/'; % actual experiment data
% meta.run = '001'; % testing calibration and configurations, overhead lights on, uav on tripod
% meta.run = '002'; % testing calibration, stage lights on, uav on tripod
% meta.run = '003'; % testing calibration, stage lights on, uav on tripod
% meta.run = '004'; % new calibration images
% meta.date = '20180319/'; experiments using april tags in ros message format
% meta.date = '20180319/'; % actual experiment data
% meta.run = '001'; % testing calibration, stage lights on, uav on tripod
% meta.date = '20180323/'; experiments using april tags in ros message format
% meta.date = '20180323/'; % actual experiment data
% meta.run = '004'; % testing calibration
% meta.run = '005'; % testing offsets: wrong sign
% meta.run = '006'; % abcde experiment (linear trajecotries)
% meta.run = '007'; % abcde experiment (step trajecotries)
% meta.date = '20180326/'; experiments using april tags in ros message format
% meta.date = '20180326/'; % actual experiment data
% meta.run = '001'; % abcde experiment (linear trajecotries), uav spun in circles, testing python calibration after weekend.
% meta.run = '002'; % abcde experiment (linear trajecotries), testing python calibration after weekend.
% meta.run = '003'; % abcde experiment (linear trajecotries), testing python calibration after weekend.
% meta.run = '004'; % action experiment, ugv drove into obstruction
% meta.run = '005'; % abcde experiment (step trajecotries)
% meta.run = '006'; % action experiment, ugv drove into obstruction
% meta.date = '20180427/'; experiments using tbrl
% meta.date = '20180427/'; % actual experiment data
% meta.run = '001'; % static test, overhead lights
% meta.run = '002'; % stage lights, abdce:=true, step:=true
% meta.date = '20180430/'; experiments using tbrl
% meta.date = '20180430/'; % actual experiment data
% meta.run = '001'; % stage lights, action:=true, wait:=45 :: good UGV path
% meta.run = '002'; % stage lights, action:=true, wait:=45 :: good UGV path
% meta.run = '003'; % stage lights, action:=true, wait:=45 :: UAV flew out of FOV
% meta.run = '004'; % stage lights, action:=true, wait:=45:: UAV flew out of FOV
% meta.run = '005'; % stage lights, action:=true, wait:=45:: UGV drove into obstacle
% meta.run = '006'; % stage lights, action:=true, wait:=45:: UAV flew out of FOV
% meta.date = '20180504/'; experiments using gazebo
% meta.date = '20180504/'; % experiments using gazebo
% meta.run = '001'; % simulated in gazebo!
% meta.date = '20180506/'; experiments using gazebo
% meta.date = '20180506/'; % experiments using gazebo
% meta.run = '001'; % simulated in gazebo! abcde step
% meta.run = '002'; % simulated in gazebo! abcde step, fixed ugv-stereo transform
% meta.run = '003'; % simulated in gazebo! abcde step, fixed ugv-stereo transform
% meta.run = '006'; % simulated in gazebo! action 
% meta.date = '20180508/'; experiments using gazebo configuring ardrone pid gains
% meta.date = '20180508/'; % experiments using gazebo
% meta.run = '001'; % simulated in gazebo!
% meta.run = '002'; % simulated in gazebo! rollpitchD 5.0 -> 2.0
% meta.run = '003'; % simulated in gazebo! yawP 2.0-5.0
% meta.run = '004'; % simulated in gazebo! velxy and velz 1.0 -> 0.5
% meta.run = '005'; % simulated in gazebo! testing gazebo vicon base
% meta.date = '20180510/'; experiments using gazebo configuring ardrone pid gains
% meta.date = '20180510/'; % experiments using gazebo
% meta.run = '001'; % simulated in gazebo!
% meta.run = '004'; % simulated in gazebo! 
% meta.run = '005'; % simulated in gazebo! 
% meta.run = '007'; % simulated in gazebo! 
% meta.run = '020'; % simulated in gazebo! 
% meta.date = '20180514/'; experiments using gazebo configuring ardrone pid gains
% meta.date = '20180514/'; % experiments using gazebo
% meta.run = '004'; % simulated in gazebo! 
% meta.date = '20180515/'; experiments using gazebo configuring ardrone pid gains
% meta.date = '20180515/'; % experiments using gazebo
% meta.run = '002'; % simulated in gazebo!  added pid recorder for gazebo
% meta.run = '004'; % simulated in gazebo! added state to controller msg and recorder
% meta.run = '005'; % simulated in gazebo! reduced noise factors on uav urdf
% meta.date = '20180516/'; experiments using gazebo configuring ardrone pid gains
% meta.date = '20180516/'; % experiments using gazebo
% meta.run = '008'; % simulated in gazebo!  removed most of the noise on the uav
% meta.date = '20180605/'; experiments using gazebo configuring ardrone pid gains
% meta.date = '20180605/'; % testing ugv local planner recorder
% meta.run = '001'; % testing ugv local planner recorder
% meta.run = '002'; % turned off UAV noise in urdf
% meta.run = '003'; % trying experiment with all floor markers
% meta.run = '004'; % trying experiment with all floor markers
% meta.run = '017'; % trying experiment with all floor markers
%% meta.date = '20180619/'; % abcde step in lab
%     meta.date = '20180619/'; % testing ugv local planner recorder
%     meta.run = '021'; %021 is abcde for proposal / icra2019
%% meta.date = '20180701/'; % abcde step in sim
%    meta.date = '20180701/'; % testing ugv local planner recorder
%    meta.run = '005'; 
%     meta.date = '20180702/'; % testing ugv local planner recorder
%     meta.run = '001'; 
%% meta.date = '20180711/'; % abcde step in lab
%     meta.date = '20180711/'; % testing ugv local planner recorder
%     meta.run = '001'; % testing action of UGV around block, ICs from 20180430 001
% meta.run = '003'; % local:=true
%% meta.date = '20180711/'; % abcde step in lab
%     meta.date = '20180718/'; % testing ugv local planner recorder
% meta.run = '001'; % local:=true

% meta.date = '20180730/'; % debugging diff between ckf and gazbo
% meta.run = '001'; % teleop
% meta.run = '002'; % testing
% meta.run = '003'; % abcde : may have been stopped early
% meta.run = '004'; % abcde

% meta.date = '20180731/'; % debugging diff between ckf and gazbo
% meta.run = '001'; % action plan, ugv stopped randomly
% meta.run = '002'; % action plan, tings look better
% meta.run = '003'; % action plan, added noise to wheel, things went badly
% meta.run = '004'; % action plan, fixed parenthesis typo, things looked better
% meta.run = '005'; % action plan, zeroed wheel noise when cmd vel is zero
%% meta.date = '20180814/'; % gazebo sim for icra paper, abcde errors
% meta.date = '20180814/'; 
% meta.run = '001'; % yaw errors jump when uav starts to move
% meta.run = '002'; % 
%% meta.date = '20180826/'; % testing ugv local planner recorder
%     meta.date = '20180826/'; % experiments @ tbrl
%     meta.run = '030'; % sucessful maneuver
%% meta.date = '20180828/'; % testing ugv local planner recorder
%     meta.date = '20180828/'; % gazebo on neptune
%     meta.run = '001'; % using 2018-08-21 code branch
%     meta.run = '002'; % merged oneCKF from 0826 into 0821 code branch
%     meta.run = '003'; % merged oneCKF from 0826 into 0821 code branch
%     meta.run = '007'; % gazebo sim of 20180826_030 experiment
%% meta.date = '20180905/'; % gazebo testing different picket lengths for uav
%     meta.date = '20180905/'; % 
%     meta.run = '003'; % added   yaw_goal_tolerance:  1.5 and   xy_goal_tolerance:   0.3129  to several move_base config files
%     meta.run = '006'; 
%     meta.run = '010'; 
%% meta.date = '20180905/'; % gazebo testing different picket lengths for uav
%     meta.date = '20180907/'; % 
%     meta.run = '023'; 

%% meta.date = '20190305/'; % gazebo testing uavSLAM
    meta.date = '20190305/'; % 
    meta.run = '001'; 

    
    %% define meta data
% meta.dataroot = '/media/benjamin/devsdb/hast/data/';
% meta.dataroot = '/Users/benjamin/hast/data/';
meta.dataroot = '/home/benjamin/ros/data/';
meta.saveplots = false;
meta.saveirosplots = false;
meta.saveplotroot = ['/home/benjamin/ros/data/' [meta.date meta.run] '/'];

%% data = loadData(meta);
data = loadData(meta);

% [dx, error.x] = fitDataY(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,1), data.vicon.uav.time, data.vicon.uav.P.global(:,1));
% [dy, error.y] = fitDataY(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,2), data.vicon.uav.time, data.vicon.uav.P.global(:,2));
% [dz, error.z] = fitDataY(data.ugvStereo.time, data.ugvStereo.uav.P_ugv(:,3), data.vicon.uav.time, data.vicon.uav.P.global(:,3));
%% plotclas
[data, meta] = plotclas_func(data, meta);
% ploticra2018
% figure(7)
%% if meta.saveplots
if meta.saveplots
    %%
    meta.figpath = [meta.dataroot meta.date meta.run '/figs/'];
    meta.savenotable = ['/media/benjamin/devsdb/hast/data/notable/' meta.date ];
    try mkdir(meta.figpath); end
    try mkdir(meta.saveplotroot); end
    try mkdir([meta.saveplotroot '/eps/']); end
    try mkdir([meta.saveplotroot '/figs/']); end
    
    meta.saveWhichFigs = {... % stereo vs vicon rgb marker positionss
    };

    for fig = 1:size(meta.saveWhichFigs,1)
        current_fig = figure(meta.saveWhichFigs{fig});
        meta.savefilename = ['figure(' num2str(current_fig.Number) ')_' meta.run];
        saveas(gcf, [meta.savenotable meta.savefilename '.png']); 
%         saveas(gcf, [meta.savenotable meta.savefilename '.fig']); 
%         print('-depsc', [meta.savenotable meta.savefilename '.eps']); 
        clear current_fig;
    end
    clear fig
end
%% 20170212-011  
% matlabStereo.elevation = {-1.6];
% matlabStereo.azimuth = {-1.15};        
%% 20170210-023  
% data.matlabStereo.elevation = {0.25};
% data.matlabStereo.azimuth = {-1.25};  
%%
% data.matlabStereo = matlabStereo(data.ugvStereo, data.vicon, data.matlabStereo);
% plot_matlabStereo(meta, data);

%% test vgg_gui_f
% PL = data.ugvStereo.Projection.Left;
% PR = data.ugvStereo.Projection.Right;
% F = vgg_F_from_P(PL,PR);
% F = F/norm(F)
% imdir = ['/home/benjamin/git/hast/data/' meta.date meta.run '/original/'];
% IL = imread([imdir 'left_image_00704.png']);
% IR = imread([imdir 'right_image_00704.png']);
% vgg_gui_F(IL,IR,F')


%%

figure(125)