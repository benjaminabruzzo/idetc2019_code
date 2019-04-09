clc
clear all
close all

warning('off','MATLAB:legend:PlotEmpty')

% meta.root = '/media/benjamin/devsdb/hast/matlab/';
% meta.root = '/home/benjamin/git/hast/matlab/';
meta.root = '/Users/benjamin/hast/matlab/';
meta.datapath = '/Users/benjamin/ros/data/';
cd([meta.root 'exp-scripts'])
addpath(genpath([meta.root 'exp-scripts'])); rmpath(genpath([meta.root 'exp-scripts/.git']))

load([meta.datapath 'loopout_20170112.mat'])
load([meta.datapath 'loopout_20170913.mat'])
loopout1 = loopout_20170112;
loopout2 = loopout_20170913;
overlay_yaw_uncert(loopout_20170112, loopout_20170913);