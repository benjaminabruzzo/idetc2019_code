clc
clear all
close all

warning('off','MATLAB:legend:PlotEmpty')


meta.root = '/Users/benjamin/hast/ros/metahast/matlab';
% meta.root = '/home/benjamin/ros/src/metahast/matlab/';
cd([meta.root])
addpath(genpath([meta.root])); 
clas