clc
clear all
close all

warning('off','MATLAB:legend:PlotEmpty')

meta.root = '~/git/hast/matlab/';
cd(meta.root)
addpath([meta.root 'exp-scripts'])
cd([meta.root 'exp-scripts'])
loopclas
rmpath([meta.root 'exp-scripts'])
set(0, 'DefaultFigureVisible', 'on');
figHandles = findall(0, 'Type', 'figure');
set(figHandles(:), 'visible', 'on')
playsound()
