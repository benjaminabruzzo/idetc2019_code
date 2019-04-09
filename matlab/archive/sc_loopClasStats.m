clc
clear all
close all

warning('off','MATLAB:legend:PlotEmpty')

meta.root = '~/git/hast/matlab/';
cd(meta.root)
addpath([meta.root 'exp-scripts'])
addpath([meta.root 'rotationprimitives'])
loopClasStats
cd([meta.root 'exp-scripts'])
disp(pwd)
playsound()
