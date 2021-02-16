%% LOAD DATA

clear all
mocap_path = 'C:\Users\tylin\Documents\MATLAB\data\08282020_trial1_data';
mocap_struct = who(matfile(mocap_path));
mocap_struct = mocap_struct{1};
mocap = load(mocap_path, mocap_struct);
mocap = mocap.(mocap_struct);

% LCM DATA BEING USED:
% contact_data.tau_feed_back = joint tau
% leg_control_data.q, leg_control_data.p = joint position, foot position
% microstrain.acc, microstrain.omega, microstrain.rpy, micrstrain.quat =
% IMU accelerometer, gyro, orientation
lcm_path = 'C:\Users\tylin\Documents\MATLAB\data\lcmlog-2020-08-29-00.mat';
lcm = load(lcm_path, 'contact_data', 'leg_control_data', 'microstrain');


%% TRIM MOCAP DATA

% indicate indices corresponding to body and leg(s)
% read mocap.RigidBodies.Name to figure out which is which, and order
% leg_idx as follows: [right front, left front, right back, left back]
mocap_body_idx = 2;
mocap_leg_idx = [3, 4, 5, 6];

% build transformation matrix to find leg coordinates relative to body
p_body = mocap.RigidBodies.Positions(mocap_body_idx, :, :);
p_body = reshape(p_body, 3, 1, []);
R_body = mocap.RigidBodies.Rotations(mocap_body_idx, :, :);
R_body = reshape(R_body, 3, 3, []);

g_body_inv = repmat(eye(4, 4), [1, 1, size(p_body, 3)]);           % build inverse manually
g_body_inv(1:3, 1:3, :) = pagetranspose(R_body);                    % R'
g_body_inv(1:3, 4, :) = pagemtimes(-pagetranspose(R_body), p_body); % -R'p

mocap_leg = pagetranspose(cat(2, mocap.RigidBodies.Positions(mocap_leg_idx, :, :), ...
                                    ones(size(mocap_leg_idx, 2), 1, size(p_body, 3))));
mocap_leg = pagemtimes(g_body_inv, mocap_leg);
mocap_leg_z = squeeze(mocap_leg(3,:,:))' .* (-1 / 1000);

% plot rough estimate
T = 1 / mocap.FrameRate;
mocap_t = 0:T:((size(mocap_leg_z, 1) - 1) * T);
plot(mocap_t, mocap_leg_z(:,1));

% identify mocap start and end times
mocap_start_t = input('Judging from the graph, please provide the time that the measurement starts: ');
mocap_end_t = input('Judging from the graph, please provide the time that the measurement ends: ');

% adjust mocap values to match given start/end time
[~, ~, mocap_t, mocap_leg_z] = crop_data(mocap_t, mocap_leg_z, mocap_start_t, mocap_end_t);

% replot shifted mocap data, identify first peak time
plot(mocap_t, mocap_leg_z(:,1));
mocap_pulse_t = input('Judging from the shifted graph, please provide the time at the first pulse: ');


%% ALIGN LCM FOOT POSITION WITH MOCAP
% pick leg index that corresponds to the first leg in mocap
% 0 = right front, 1 = left front, 2 = right back, 3 = left back
lcm_leg_idx = 0;

% extract z coordinates of foot from lcm
lcm_foot_pos = lcm.leg_control_data.p(:, (3*lcm_leg_idx + 3));
foot_t = lcm.leg_control_data.lcm_timestamp;

% identify first peak time in order to align with mocap
plot(foot_t, lcm_foot_pos);
lcm_pulse_t = input('Judging from the graph, please provide the time at the first pulse: ');

lcm_start_t = lcm_pulse_t - mocap_pulse_t;
lcm_end_t = lcm_start_t + (mocap_end_t - mocap_start_t);
[~, ~, foot_t, lcm_foot_pos] = crop_data(foot_t, lcm_foot_pos, lcm_start_t, lcm_end_t);

% plot aligned mocap, lcm data
plot(mocap_t, mocap_leg_z(:,1) - mean(mocap_leg_z(:,1), 'omitnan'), foot_t, lcm_foot_pos - mean(lcm_foot_pos));
title('mocap vs. lcm')
legend('mocap', 'lcm')


%% ALIGN ALL LEG DATA
% lcm_start_t and lcm_end t mark the start and end points we will align to

% contact_data
contact_t = lcm.contact_data.lcm_timestamp;
lcm_tau_fb = lcm.contact_data.tau_feed_back;
[start_idx, end_idx, contact_t, lcm_tau_fb] = crop_data(contact_t, lcm_tau_fb, lcm_start_t, lcm_end_t);
lcm_tau_ff = lcm.contact_data.tau_feed_forward(start_idx:end_idx, :);

% leg_control_data
legcontrol_t = lcm.leg_control_data.lcm_timestamp;
lcm_q = lcm.leg_control_data.q;
[start_idx, end_idx, legcontrol_t, lcm_q] = crop_data(legcontrol_t, lcm_q, lcm_start_t, lcm_end_t);
lcm_p = lcm.leg_control_data.p(start_idx:end_idx, :);
lcm_qd = lcm.leg_control_data.qd(start_idx:end_idx, :);
lcm_v = lcm.leg_control_data.v(start_idx:end_idx, :);
lcm_tau_est = lcm.leg_control_data.tau_est(start_idx:end_idx, :);

% microstrain
imu_t = lcm.microstrain.lcm_timestamp;
lcm_acc = lcm.microstrain.acc;
[start_idx, end_idx, imu_t, lcm_acc] = crop_data(imu_t, lcm_acc, lcm_start_t, lcm_end_t);
lcm_omega = lcm.microstrain.omega(start_idx:end_idx, :);
lcm_rpy = lcm.microstrain.rpy(start_idx:end_idx, :);
lcm_quat = lcm.microstrain.quat(start_idx:end_idx, :);


%% IDENTIFY CONTACT INSTANCES
% mocap and lcm data are now aligned wrt to time, as well as their time stamps
% input data:   [n x d]
% time stamps:  [1 x n]

contact_labels = islocalmin(mocap_leg_z) & (mocap_leg_z < mean(mocap_leg_z, 1, 'omitnan'));
peak = islocalmax(mocap_leg_z) & (mocap_leg_z > mean(mocap_leg_z, 1, 'omitnan'));

for l = 1:size(mocap_leg_z, 2)
    contact_singleleg = contact_labels(:,l);
    peak_singleleg = peak(:,l);

    % indexes at which local mins and maxes occurred
    contact_idxs = find(contact_singleleg);
    peak_idxs = find(peak_singleleg);

    % for clarity/safety, we start searching for contact only after 1st peak
    contact_idxs(contact_idxs < peak_idxs(1)) = 0;
    i = find(contact_idxs, 1);
    j = 2;

    while i <= size(contact_idxs, 1) && j <= size(peak_idxs, 1)
        contact_start = contact_idxs(i);
        next_peak = peak_idxs(j);

        while i <= size(contact_idxs, 1) && contact_idxs(i) < next_peak 
            contact_end = contact_idxs(i);
            i = i + 1;
        end

        contact_singleleg(contact_start:contact_end) = 1;
        j = j + 1;
    end
    
    contact_labels(:,l) = contact_singleleg;
end

for i = 1:size(contact_labels, 2)
    figure
    plot(mocap_t, mocap_leg_z(:,i), mocap_t(contact_labels(:,i)), mocap_leg_z(contact_labels(:,i), i), 'r*')
    title(i)
    
end

save('sync_data.mat', ...
     'mocap_t', 'contact_labels', ...
     'contact_t', 'lcm_tau_fb', 'lcm_tau_ff', ...
     'legcontrol_t', 'lcm_q', 'lcm_p', 'lcm_qd', 'lcm_v', 'lcm_tau_est', ...
     'imu_t', 'lcm_acc', 'lcm_omega', 'lcm_rpy', 'lcm_quat')

function [start_idx, end_idx, t, x] = crop_data(t_init, x_init, start_t, end_t)
start_idx = knnsearch(t_init', start_t);
end_idx = knnsearch(t_init', end_t);
x = x_init(start_idx:end_idx, :);
t = t_init(start_idx:end_idx) - t_init(start_idx);

end