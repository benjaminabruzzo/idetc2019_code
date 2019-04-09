%% meta.date = '20161209/'; 
% meta.date = '20161209/'; % 021-022
% meta.date = '20161204/'; % 021-022
% meta.date = '20161216/'; % 021-022
% meta.date = '20161218/'; 
% meta.date = '20161219/'; 
% meta.date = '20161228/'; 
% '012'; '014'; '016'; '018'; '020'; ...
% '022'; '024'; '026'; '028'; '030'; ...
% '032'; '036'; '038'; '042';  ...
% meta.run = '002';
% 
%% meta.date = '20170106/'; 
% meta.date = '20170106/'; 
% jstr = {...
%     '020'; '022'; '024'; '025'; '026';...
%     '027'; '031'; '028'; '032'; '029';...
%     '030'; '033'; '034'; '035';...
%     '036'; '037'; '038';...
%     }; %037 is mostly okay, except for what looks like a single data point
% meta.run = jstr{1};
%% meta.date = '20170112/'; Data use for IROS yaw plots
% meta.date = '20170112/'; 
% meta.run = '020';
% meta.run = '022';
% meta.run = '024';
% meta.run = '033';
% meta.run = '020';
% meta.run = '035'; % uav spin
% meta.dataroot = '/Users/benjamin/Documents/Academic/SIT/hast/ROS/Data Archive/2015/';
% meta.dataroot = '~/benjamin/git/hast/data/';

% meta.date = '20151230/'; 
% meta.run = '017'; % Figure 3 : X positions shows the trajectory shaping of drone.

% meta.date = '20170120/'; % actual data collection with images
% meta.date = '20170123/'; % sim of 20january data
% 001-003 008-011 015 017 019 are no good 
% runs before 006 were poorly calibrated
% meta.run = '018'; % 
% meta.run = '046'; % drone suspended from tripod
%% meta.date = '20170126/'; % actual data collection with images
% meta.date = '20170126/'; % actual data collection with images
% meta.date = '20170129/'; % simulating data on mars from 20170126
% meta.date = '20170130/'; % simulating data on create/mars from 20170126
% meta.date = '20170131/'; % simulating data on neptune from 20170126
% meta.run = '008'; 
%% meta.date = '20170203/'; % actual data collection with images
%% meta.date = '20170205/'; % actual data collection with imagesmeta.date = '20170205/'; % actual data collection with images
% meta.date = '20170205/';
% meta.run = '008'; % linear ugv experiment
%% meta.date = '20170210/'; % actual data collection with images
% meta.date = '20170210/'; % actual data collection with images
% meta.date = '20170211iros/'; % sim data for 20170210
% meta.run = '006'; % 006 - 018 are for the plots vs range

% Flights
% meta.run = '022'; 
% meta.run = '029'; 
% meta.run = '030';
% meta.run = '031'; 
% meta.run = '032'; 
% meta.run = '033'; 
% meta.run = '034'; 
% meta.run = '035';
% meta.run = '036';
% meta.run = '037'; 
% meta.run = '038'; 
% meta.run = '039'; 
% meta.run = '040';
% meta.run = '041'; 
% meta.run = '042'; 
% meta.run = '043'; %flight test for plotting
% meta.run = '044'; 
% meta.run = '045';
% meta.run = '046'; 
% meta.run = '047'; 
% meta.run = '048';

% flights that are worth considering:
% 023, 029, 038, 042, 043, 045
% meta.run = '043'; 
%% meta.date = '20170212/'; % actual data collection with images
% meta.date = '20170212/'; % 
% meta.run = '024'; %1 m
%% Iros2017 flight data for statistics
% meta.date = '20170205/'; 
% meta.run = '019';
% meta.run = '047';
% meta.date = '20170210/'; 
% meta.run = '023';
% meta.run = '038';
% meta.run = '042';
% meta.run = '043';
% meta.date = '20170212/'; 
% meta.run = '011'; 
% meta.run = '019'; 
% meta.run = '020'; 
% meta.date = '20170211/'; 
% meta.run = '043';
%% meta.date = '20170317/';
% meta.date = '20170317/'; % actual data collection with images
% meta.run = '034'; 
%% dates for re-creating iros2017 paper
% meta.date = '20170112/'; meta.run = '022';
% meta.date = '20170211/'; meta.run = '043';

% for ugv motion:
%     meta.date = '20170205/'; % for figure 14 
    % 006-010 linear ugv motion
    % 026-030 angular ugv motion
%     meta.run = '008'; % '008' for figure 14
%     meta.run = '028'; % angular turn for paper?
%% meta.date = '20170331/';
% meta.date = '20170331/'; % actual data collection with images
% meta.run = '016'; 
% meta.run = '017'; 
% meta.run = '018'; 
% meta.run = '019'; 
%% 20170732 : first data collected after changin cameras to chameleon 3
% meta.date = '20170731/';
% meta.run = '009'; 
%% 20170825 : trying uav flights with move_base mapping as well
% meta.date = '20170825/';
% meta.run = '005'; 
% meta.run = '015'; 
% meta.run = '016'; 
% meta.run = '017'; 
% meta.run = '019'; 
% meta.run = '020'; 
% meta.run = '022'; 
%% 20170905 : trying uav picket flights and basic testing
% meta.date = '20170905/';
% meta.run = '010'; 
%% 20170907 : trying uav picket flights and basic testing
% meta.date = '20170907/';
% meta.run = '001'; % cord pull : stereo time issue
% meta.run = '041'; 
%% 20170908 : added roi testing to stereo obs
% meta.date = '20170908/';
% meta.run = '025'; % picket test
%% 20170911 : testing with VICON!
% meta.date = '20170911/';
% meta.run = '001'; % pull test 001-006, 032 (mostly debugging argumnetalizing the experiment node)
% meta.run = '050'; % abcde test 007-033
%% 20170912 : testing with VICON!
% meta.date = '20170912/';
% meta.run = '026'; % pull test 001-007
%% 20170913 : testing with VICON!
% meta.date = '20170913/';
% meta.run = '033'; % pull test used for icra paper plot 
% meta.run = '999'; % pull test 999 is actually 001 from 20170914
%% 20170914 : testing with VICON!
% meta.date = '20170914/';
% meta.run = '011'; % pull test 001-011
% meta.run = '012'; % picket experiment
% meta.run = '013'; % picket fore experiment % UAV always updates ckf with new stereo data (good looking flight)
% meta.run = '015'; % picket turn experiment % UAV always updates ckf with new stereo data
% meta.run = '017'; % picket turn experiment % not bad, yaw doesn't track ugv
% meta.run = '021'; % picket complex maneuver experiment % UAV always updates ckf with new stereo data 
% meta.run = '023'; % picket complex maneuver experiment % not a terrible run, battery died at the end
% meta.run = '026'; % picket complex maneuver experiment % linear works, angular does not (UAV spins
% in circles because the way dt was being calculated in the ugv recorder (fixed by thresholding dt)
% meta.run = '028'; % picket turn experiment, decent flight
% meta.run = '029'; % picket F-T-F-T experiment % UAV spins in circles (dt issue)
% meta.run = '030'; % picket complex maneuver experiment % it worked!!
% meta.run = '031'; % picket complex maneuver experiment % ugv did not move
% meta.run = '033'; % picket complex maneuver experiment % ugv did not move
% meta.run = '034'; % picket complex maneuver experiment % ugv did not move
% meta.run = '035'; % picket complex maneuver experiment % not bad
% meta.run = '036'; % picket complex maneuver experiment % decent run (has 2x april tags)
% meta.run = '037'; % picket arc forward experiment % 
% meta.run = '038'; % picket arc forward experiment % longer arc with larger radius
% meta.run = '039'; % picket 2x arc forward experiment % uav yaw now tracks ugv
% meta.run = '040'; % picket complex maneuver experiment % 
% meta.run = '041'; % picket more complex maneuver experiment : F Arcleft Arcright F % no issues
% meta.run = '042'; % picket more complex maneuver experiment : F Al F Ar F 
%% 20170922 : switching ugv cmd vel with odom vel
% meta.date = '20170922/';
% meta.run = '001'; % ugvforeward
% meta.run = '002'; % ugvforeward
% meta.run = '003'; % ugvforeward wheel odom based ckf may now work
% meta.run = '004'; % ugvforeward wheel odom based ckf may now work
% meta.run = '005'; % ugvforeward wheel odom based ckf now works, but is not very good
% meta.run = '006'; % ugvforeward wheel odom based ckf now works, and has okay data
% meta.run = '008'; % arcleft then arc right
%% 20171002 : switching ugv cmd vel with odom vel
% meta.date = '20171002/';
% meta.run = '003'; % pull
% meta.run = '004'; % pull
% meta.run = '006'; % pull
% meta.run = '009'; % ugvfore
% meta.run = '010'; % ugvfore kobuki (no experiment.m file, forgot to change to kobuki in launch file)
% meta.run = '011'; % Forgot to turn on kobuki base
% meta.run = '012'; % did not call experiment launch file with tb_base:=kobuki
% meta.run = '013'; % and I quote "WTF was that?!"
% meta.run = '014'; % I have no notes for 014 
% meta.run = '015'; % I have no notes for 015
%% 20171006 : switching ugv cmd vel with odom vel
% meta.date = '20171006/';
% meta.date = '20171010/'; %simulation data of 10/06
% meta.run = '001'; % ugvfore
% meta.run = '002'; % ugvfore : weird quick turn
% meta.run = '003'; % ugvfore : weird quick turn
% meta.run = '004'; % ugvfore
% meta.run = '005'; % ugvfore : weird quick turn
% meta.run = '006'; % ugvfore
% meta.run = '007'; % ugvfore
%% 20171013 : switching ugv cmd vel with odom vel
% meta.date = '20171013/';
% meta.run = '001'; % ugvfore (no wheel odom data)
% meta.run = '002'; % ugvfore (with wheel odom data)
% meta.run = '003'; % picket : ugv turn is still wacky
% meta.run = '004'; % picket : uav sudden altitude loss, may have been a battery issue
% meta.run = '005'; % picket : wacky ugv turn
% meta.run = '006'; % picket : jerky ugv turn, but the uav kept up..
% meta.run = '007'; % picket
%% 20171016 : switching ugv cmd vel with odom vel
% meta.date = '20171016/'; %stereo settings are off did not observe any uav markers

% meta.run = '001'; % picket, wonky behavior, testing velocity ramps
% meta.run = '004'; % picket, super slow turn, maybe the wrong way
% meta.run = '005'; % picket, stereo settings are off did not observe any uav markers
% meta.run = '006'; % picket, stereo settings are off did not observe any uav markers
%% 20171020 : switching ugv cmd vel with odom vel
% meta.date = '20171020/'; %stereo settings are off did not observe any uav markers
% meta.run = '001'; % pull: (didn't pull) testing stereo obs configuration on kobuki
% meta.run = '002'; % picket, ugvAutopilot, suspended uav
% meta.run = '005'; % picket, stereo settings are off did not observe any uav markers
% meta.run = '007'; % huge velocity command
% meta.run = '008'; % weird delayed turn command
% meta.run = '009'; % weird delayed turn command
% meta.run = '012'; % weird delayed turn command
%% 20171023 : switching ugv cmd vel with odom vel
% meta.date = '20171023/'; %stereo settings are off did not observe any uav markers
% meta.run = '001'; % pull: saved raw images to tyest calibration
% meta.run = '002'; % pull: no raw images 
% meta.run = '003'; % pull until TF NaN error - no error
% meta.run = '004'; % pull until TF NaN error - no error
% meta.run = '005'; % pull until TF NaN error - no error
% meta.run = '006'; % pull until TF NaN error - no error
%% 20171027 : switching ugv cmd vel with odom vel
% meta.date = '20171027/'; %stereo settings are off did not observe any uav markers
% meta.run = '001'; % pull
% meta.run = '002'; % pull
% meta.run = '003'; % ugvAuto picket
%% 20171030 : gathering april tag data for characterizing deviations
% meta.date = '20171030/'; %stereo settings are off did not observe any uav markers
% meta.run = '009'; % stage lights, ta size 152.4mm, raw image
% meta.run = '010'; % stage lights, ta size 152.4mm, raw image
% meta.run = '011'; % stage lights, ta size 152.4mm, rectified image
% meta.run = '012'; % stage lights, ta size 152.4mm, rectified image
% meta.run = '013'; % stage lights, ta size 152.0mm, rectified image
% meta.run = '014'; % stage lights, ta size 152.0mm, rectified image
%% 20171030 : gathering april tag data for characterizing deviations
% meta.date = '20171103/'; %stereo settings are off did not observe any uav markers
% meta.run = '001'; % stage lights, tag size 152mm, image_rect
% meta.run = '002'; % stage lights, tag size 152mm, image_rect
% meta.run = '003'; % stage lights, tag size 152mm, image_rect
% meta.run = '004'; % stage lights, tag size 152mm, image_rect
% meta.run = '005'; % stage lights, tag size 152mm, image_rect
% meta.run = '006'; % stage lights, tag size 152mm, image_rect
% meta.run = '007'; % stage lights, tag size 152mm, image_rect
% meta.run = '008'; % stage lights, tag size 152mm, image_rect
% meta.run = '009'; % stage lights, tag size 152mm, image_rect
% meta.run = '010'; % stage lights, tag size 152mm, image_rect
% meta.run = '011'; % stage lights, tag size 152mm, image_rect
% meta.run = '012'; % stage lights, tag size 152mm, image_rect seems broken for some reason, y and z are also very close here
% meta.run = '013'; % stage lights, tag size 152mm, image_rect
% meta.run = '014'; % stage lights, tag size 152mm, image_rect y and z are nearly perfect, x is off.  it might just be another adjustment on the tf

% meta.run = '016'; % stage lights, tag size 152mm, image_rect tf = "0.075 0 0 pi/2 pi 0"
% meta.run = '017'; % NaN error tf = "0.1 0 0 pi/2 pi 0"
% meta.run = '018'; % stage lights, tag size 152mm, image_rect, tf = "0.1 0 0 pi/2 pi 0"
% meta.run = '019'; % stage lights, tag size 152mm, image_rect, tf = "0.125 0 0 pi/2 pi 0"
% meta.run = '020'; % stage lights, tag size 152mm, image_rect, tf = "0.825 0 0 pi/2 pi 0"
%% meta.date = '20171106/'; %stereo settings are off did not observe any uav markers
% meta.date = '20171106/'; %stereo settings are off did not observe any uav markers
% meta.run = '001'; % stage lights, tf = "0.0875 0 0 pi/2 pi 0"
% meta.run = '002'; % stage lights, tf = "0.0875 0 0 pi/2 pi 0"
% meta.run = '003'; % stage lights, tf = "0.0875 0 0 pi/2 pi 0"
% meta.run = '004'; % stage lights, tf = "0.0875 0 0 pi/2 pi 0"
% meta.run = '005'; % stage lights, tf = "0.0875 0 0 pi/2 pi 0"
% meta.run = '009'; % stage lights, tf = "0.0875 0 0 pi/2 pi 0"
% meta.run = '010'; % stage lights, tf = "0.0825 0 0 pi/2 pi 0"
%% meta.date = '20171117/'; 
% meta.date = '20171117/'; 
% meta.run = '001'; % overhead lights, create base drive
% meta.run = '002'; % overhead lights, create base drive
% meta.run = '003'; % overhead lights, create base drive
% meta.run = '004'; % overhead lights, create base drive
% meta.run = '005'; % overhead lights, create base drive
% meta.run = '006'; % overhead lights, create base drive
% meta.run = '007'; % overhead lights, create base drive
% meta.run = '008'; % overhead lights, create base drive
% meta.run = '009'; % overhead lights, create base drive
% meta.run = '010'; % overhead lights, create base drive
%% meta.date = '20171201/'; %stereo settings are off did not observe any uav markers
% meta.date = '20171201/'; %stereo settings are off did not observe any uav markers
% meta.run = '001'; % ceiling lights, uav on ground facing north
% meta.run = '002'; % ceiling lights, uav on ground facing north east
% meta.run = '003'; % ceiling lights, uav on ground facing east
% meta.run = '004'; % ceiling lights, uav on ground facing south east
% meta.run = '005'; % ceiling lights, uav on ground facing south
% meta.run = '006'; % ceiling lights, uav on ground facing south west

% meta.run = '013'; % 
%% meta.date = '20171204/'; %stereo settings are off did not observe any uav markers
% meta.date = '20171204/'; %stereo settings are off did not observe any uav markers
% meta.date = '20171206/'; %simulation of 1204 data
% meta.run = '005'; % stage lights, arcarc
% meta.run = '014'; % stage lights, autopilot