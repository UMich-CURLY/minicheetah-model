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
urdf = fullfile('urdf','mini_cheetah_mesh.urdf');
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
