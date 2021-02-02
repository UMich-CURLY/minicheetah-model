% Reset 
clear; matlabrc; clc;
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

% Add model paths
addpath(genpath('urdf'));

%% Load model
miniCheetah = MC_model(fullfile('urdf','mini_cheetah_v2.urdf'));

%% Export Kinematics
if ~exist(fullfile(EXPORT_PATH,'kin'),'dir')
    mkdir(fullfile(EXPORT_PATH,'kin'));
    mkdir(fullfile(EXPORT_PATH,'kin','slrt'));
    mkdir(fullfile(EXPORT_PATH,'kin','eigen'));
end

%% Compute Manipulator Jacobians
miniCheetah.ExportKinematics_IMU(@Export.export_slrt, fullfile(EXPORT_PATH,'kin','slrt'));
miniCheetah.ExportKinematics_IMU(@Export.export_eigen, fullfile(EXPORT_PATH,'kin','eigen'));
%     
miniCheetah.ExportJacobians_IMU(@Export.export_slrt, fullfile(EXPORT_PATH,'kin','slrt'));
miniCheetah.ExportJacobians_IMU(@Export.export_eigen, fullfile(EXPORT_PATH,'kin','eigen'));

