%% Load calibration data
clc
clear all

path_to_cal = '/Users/benjamin/git/hast/ros/turtlebot/cam_info/';
cal_filename = 'kobukistereo20161123b';

savepath = [path_to_cal];

cd(savepath)
load([cal_filename '.mat'])

clear savepath cal_filename

%%
PL = Calibration.Left.ProjectionMatrix;
PR = Calibration.Right.ProjectionMatrix;

% clc
% F = vgg_F_from_P(PL,PR);
% F = F/norm(F)
% F = F/F(3,3)
% %%
F = zeros(3);
F(2,3) = 1;
F(3,2) = -1;

%%
imdir = '/Users/benjamin/git/hast/data/20161123/024/good/';

IL = imread([imdir 'left.png']);
IR = imread([imdir 'right.png']);

vgg_gui_F(IL,IR,F')

%%
imdir = '/Users/benjamin/ros/data/20180224/011/rectified/';

IL = imread([imdir 'left_rect_00053.png']);
IR = imread([imdir 'right_rect_00053.png']);

% F = zeros(3); F(2,3) = 1; F(3,2) = -1;
F = data.caldata.stereo.F;
vgg_gui_F(IL,IR,F')
