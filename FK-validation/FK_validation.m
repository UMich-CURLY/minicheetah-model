%  Last edited in 01.08.2020
%% Script to run mex and do visualization

% Reset 
clear; restoredefaultpath; clc;

% Add paths
addpath('mex');
addpath('lcm-log');
addpath('src');

%% Load lcm log
load('lcmlog-2019-12-26-trot.mat');

%% Perform FK, encoder readings: leg_control_data.q
% init
n = length(leg_control_data.lcm_timestamp);
p_Body_to_FrontRightFoot = zeros(n,3);
p_Body_to_FrontLeftFoot = zeros(n,3);
p_Body_to_HindRightFoot = zeros(n,3);
p_Body_to_HindLeftFoot = zeros(n,3);

for i = 1:n
    encoder = leg_control_data.q(i,:)';
    p_Body_to_FrontRightFoot(i,:) = p_Body_to_FrontRightFoot_mex(encoder)';
    p_Body_to_FrontLeftFoot(i,:) = p_Body_to_FrontLeftFoot_mex(encoder)';
    p_Body_to_HindRightFoot(i,:) = p_Body_to_HindRightFoot_mex(encoder)';
    p_Body_to_HindLeftFoot(i,:) = p_Body_to_HindLeftFoot_mex(encoder)';
    if mod(i,1000)==0
        fprintf('%2.2f percentage completed\n', i/n*100);
    end 
end
%% Compare with leg_control_data.p
% FYI
% cheetah._bodyLength = 0.19 * 2;
% cheetah._bodyWidth = 0.049 * 2;
% cheetah._bodyHeight = 0.05 * 2;
% 
% FRONT
% 1 0   RIGHT
% 3 2
% BACK
%     x
%     ^
%     |
% y<- z
% 
% leg_control_data.p:
% [T_Hip_to_FrontRightFoot' T_Hip_to_FrontLeftFoot' ...
%  T_Hip_to_HindRightFoot' T_Hip_to_HindLeftFoot']
FK_Body_to_Foot = [ p_Body_to_FrontRightFoot p_Body_to_FrontLeftFoot ...
                    p_Body_to_HindRightFoot p_Body_to_HindLeftFoot];
diff = FK_Body_to_Foot - leg_control_data.p;
diff_mean = mean(diff, 1)
diff_std = std(diff)