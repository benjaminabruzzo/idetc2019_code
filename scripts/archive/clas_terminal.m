%% setup
meta.root = '~/ros/matlab/';
cd(meta.root)
addpath([meta.root 'exp-scripts'])
addpath([meta.root 'rotationprimitives'])

%% clas

meta.showdisp = true;
meta.dataroot = '~/ros/data/';
meta.date = '20170130/'; % simulating data on mars from 20170126
meta.run = '014'; % 001 had overhead lights illuminated

LoadRunData
syncHastTimers
loadViconData

quit;
%% closeout
% cd([meta.root 'exp-scripts'])
% disp(pwd)
% playsound()
