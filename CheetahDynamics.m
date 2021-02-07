% Reset 
clear; matlabrc; restoredefaultpath; clc;
cur = pwd;
EXPORT_PATH = fullfile(cur, 'gen');
if ~exist(EXPORT_PATH,'dir')
    mkdir(EXPORT_PATH);
    addpath(EXPORT_PATH);
end

% Add FROST
FROST_PATH = 'C:\Users\tylin\Documents\MATLAB\frost-dev-master';
addpath(FROST_PATH)
frost_addpath;

addpath(genpath('urdf'));
genpath('urdf');

% order of legs are different in v1 and v2
% v2 aligns with the convention we used on mini cheetah
% v1: front left, front right, hind left, hind right
% v2: front right, front left, hind right, hind left
urdf = fullfile('urdf','mini_cheetah_v2.urdf');

%%
mc = MiniCheetah(urdf);
%%
if ~exist(fullfile(EXPORT_PATH,'dyn'),'dir')
    mkdir(fullfile(EXPORT_PATH,'dyn'));
    mkdir(fullfile(EXPORT_PATH,'dyn','slrt'));
    mkdir(fullfile(EXPORT_PATH,'dyn','eigen'));
end
mc.configureDynamics('DelayCoriolisSet',false);
% define B matrix here;
B = [zeros(6,12); eye(12)];

mc.ExportDynamics(@Export.export_slrt, fullfile(EXPORT_PATH,'dyn','slrt'));    
mc.ExportDynamics(@Export.export_eigen, fullfile(EXPORT_PATH,'dyn','eigen'));    
