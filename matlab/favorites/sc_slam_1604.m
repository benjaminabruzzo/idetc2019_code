clc
clear all
close all

warning('off','MATLAB:legend:PlotEmpty')

meta.root = '/home/benjamin/ros/src/metahast/matlab/';
cd([meta.root])
addpath(genpath([meta.root])); 
slam_1604

set(0, 'DefaultFigureVisible', 'on');
default_colors = getpref('display','default_colors');