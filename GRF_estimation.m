% clear; clc;

%% add path
cur_pth = pwd;
gen_pth = append(cur_pth, '/gen/dyn/eigen/mex');
addpath(gen_pth);
thirdparty_pth = append(cur_pth, '/thirdparty');
addpath(genpath(thirdparty_pth));

%% load data
data_pth = "C:\Users\tylin\Documents\MATLAB\data";
addpath(data_pth);
data_name = "lcmlog-2020-08-29-00";
load(append(data_name,".mat"));

% plot(1:size(leg_control_data.tau_est(:,3)),leg_control_data.tau_est(:,3));

% find start index
% start_idx = find(leg_control_data.tau_est(:,3)>0,1);
% start_idx = 551947;
% end_idx = 601899;
% control_time = leg_control_data.lcm_timestamp(start_idx:end_idx);
lcm_start_time = 1115.22000000000;
lcm_end_time = 1151.50000000000;
control_time = leg_control_data.lcm_timestamp;
% lcm_start_time = control_time(1);
% lcm_end_time = control_time(end);
% plot(leg_control_data.lcm_timestamp(start_idx:end),leg_control_data.tau_est(start_idx:end,3));

% extract leg_control data
[start_idx, end_idx, control_time, p] = crop_data(control_time, leg_control_data.p, lcm_start_time, lcm_end_time);
% control_time = control_time - control_time(1); % shift control time to start from 0
% p = leg_control_data.p(start_idx:end_idx,:);
q = leg_control_data.q(start_idx:end_idx,:);
qd = leg_control_data.qd(start_idx:end_idx,:);
v = leg_control_data.v(start_idx:end_idx,:);
tau_est = leg_control_data.tau_est(start_idx:end_idx,:);

% imu is in 1000 Hz, encoder signal is in 500 Hz
% downsample imu to 500 Hz and match the control timestamp
imu_time = microstrain.lcm_timestamp;

% extract imu data for converting to lcm, in it's original frequency 1000Hz
[imu_start_idx, imu_end_idx, imu_time, imu_acc] = crop_data(imu_time, microstrain.acc(:,:), lcm_start_time, lcm_end_time);
imu_omega = microstrain.omega(imu_start_idx:imu_end_idx,:);
imu_rpy = microstrain.rpy(imu_start_idx:imu_end_idx,:);
imu_quat = microstrain.quat(imu_start_idx:imu_end_idx,:);

% use flip to turn rpy to ypr
imu_time_ds_idx = knnsearch(imu_time',control_time');
imu_ypr_ds = flip(microstrain.rpy(imu_time_ds_idx,:),2);
imu_dypr_ds = flip(microstrain.omega(imu_time_ds_idx,:),2);

% append body orientation from imu in q. q becomes a n*18 array.
% body_x body_y body_z body_y body_p body_r encoder_1 ... encoder_12
num_data = length(q(:,1));
ctest = zeros(num_data,3);
q = [zeros(num_data,3)'; imu_ypr_ds'; q']';
qd = [zeros(num_data,3)'; imu_dypr_ds'; qd']';


%% compute angular acceleration from angular velocity
% we apply a median filter here
% dt is the time step caculate from lcm_timestamp. 

tic
diff_qd = qd(1:num_data-1,:) - qd(2:num_data,:);
dt = control_time(1,1:num_data-1) - control_time(1,2:num_data);
diff_qd_fil = medfilt1(diff_qd,11,[],1);
qdd = diff_qd_fil ./ dt';
% qdd = diff_qd_fil ./ 0.002;
qdd = medfilt1(qdd,11,[],1);
toc

% figure(1);
% plot(1:num_data-1,qdd(1:num_data-1,3));
% hold on;
% plot(1:num_data-1, qd(1:num_data-1,3));
% hold on;
% plot(1:num_data-1, diff_qd(1:num_data-1,3));
% legend("qdd","qd","diff\_qd")
% 
% figure(2);
% plot(1:num_data-1, qd(1:num_data-1,3));
% hold on;
% plot(1:num_data-1, diff_qd(1:num_data-1,3));
% legend("qd","diff\_qd")

%% Compute Ground Reaction Force
B = [zeros(6,12); eye(12)];
F = zeros(num_data-1,12);
tic
% De*qdd + Ce*qd + Ge = B*tau + J'*F
for i = 1:num_data-1
    % Get matrices from the dynamic model
    De = De_mini_cheetah_mex(q(i,:)');
    Ce = Ce_mini_cheetah_mex(q(i,:)',qd(i,:)');
    Ge = Ge_mini_cheetah_mex(q(i,:)');
    
    % construct the Jacobian matrix
    J_fr = Jp_fr_foot_mex(q(i,:)');
    J_fl = Jp_fl_foot_mex(q(i,:)');
    J_hr = Jp_hr_foot_mex(q(i,:)');
    J_hl = Jp_hl_foot_mex(q(i,:)');
    J = [J_fr; J_fl; J_hr; J_hl];
    
    F(i,:) = (J' \ (De*qdd(i,:)'+Ce*qd(i,:)'+Ge-B*tau_est(i,:)'))';
end
toc


%% compute force magnitude
F_magnitude = zeros(num_data-1,4);
for i=1:4
    F_magnitude(:,i) = sqrt(F(:,3*i-2).^2+F(:,3*i-1).^2+F(:,3*i).^2);
end

%% visualize the forces
% figure(3)
% plot(1:length(F(:,3)),F(:,3));
% hold on
% plot(1:length(F(:,6)),F(:,6));
% hold on
% plot(1:length(F(:,9)),F(:,9));
% hold on
% plot(1:length(F(:,12)),F(:,12));
% hold on
% plot(1:length(tau_est(:,3)),tau_est(:,3));

% figure(3)
% plot(2:length(F_magnitude(:,1)),medfilt1(diff(F_magnitude(:,1)),3,[],1));
% hold on
% plot(2:length(F_magnitude(:,1)),diff(F_magnitude(:,1)));

% hold on
% plot(1:length(F_magnitude(:,2)),F_magnitude(:,2));
% hold on
% plot(1:length(F_magnitude(:,3)),F_magnitude(:,3));
% hold on
% plot(1:length(F_magnitude(:,4)),F_magnitude(:,4));
% 
% legend("GRF\_fr","GRF\_fl","GRF\_hr","GRF\_hl");

% plot(1:length(F_mag_smooth(:,1)),F_mag_smooth(:,1));

%% detect contact
period_thres = 0.1;

local_max = islocalmax(F_magnitude) & (F_magnitude > mean(F_magnitude, 1, 'omitnan'));
local_min = islocalmin(F_magnitude) & (F_magnitude < mean(F_magnitude, 1, 'omitnan'));

contacts = false(num_data,4);
tic
% loop through number of legs
for i = 1:4
    
    max_idx = find(local_max(:,i));
    min_idx = find(local_min(:,i));
    % index for time
    contact_start = max_idx(1);
    j = 1;
    k = 1;
    while contact_start<length(F_magnitude(:,1))
        j = find(min_idx(1:end)>contact_start,1,'first');
        contact_end = min_idx(j);
        if(isempty(contact_end))
            break;
        end
%         contact_time = control_time(contact_end) - control_time(contact_start)
        if control_time(contact_end) - control_time(contact_start) > period_thres
           contacts(contact_start:contact_end,i) = 1; 
        end
        
        k = find(max_idx(1:end)>contact_end,1,'first');
        contact_start = max_idx(k);
        
    end
    contacts(F_magnitude(:,i) < mean(F_magnitude(:,i), 1, 'omitnan'),i)=0;
end
toc

%% plot to visualize
figure(5)
plot(control_time(1,1:end-1),F_magnitude(:,2));
hold on
% plot(control_time(1,local_max(:,2)),F_magnitude(local_max(:,2),2), "r*");
% hold on
% 
% plot(control_time(1,local_min(:,2)),F_magnitude(local_min(:,2),2), "b*");
% hold on
plot(control_time(1,contacts(:,2)),F_magnitude(contacts(:,2),2), "g*");
% hold on
legend("GRF","contacts");



%% load mocap data to verify
mocap_pth = "C:\Users\tylin\Documents\MATLAB\data\08282020_trial1_data";
mocap_struct = who(matfile(mocap_pth));
mocap_struct = mocap_struct{1};
mocap = load(mocap_pth, mocap_struct);
mocap = mocap.(mocap_struct);

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
% plot(mocap_t, mocap_leg_z(:,1));

% mocap start and end time
mocap_start_t = 4.34;
mocap_end_t = 40.62;

[~, ~, mocap_t, mocap_leg_z] = crop_data(mocap_t, mocap_leg_z, mocap_start_t, mocap_end_t);

% plot contact with mocap data
temp = -0.28*ones(num_data,1);
temp(contacts(:,1),1) = -0.325;
non_contacts = ~contacts;
figure(6);
plot(mocap_t, mocap_leg_z(:,1));
hold on
plot(control_time(1,contacts(:,1)),temp(contacts(:,1),1), "g*");
hold on
plot(control_time(1,non_contacts(:,1)),temp(non_contacts(:,1),1), "r*");

legend("mocap\_leg\_position","contacts","non\_contacts");

% %% to run this session, run utils/mocap_lcm_sync.m first.
% figure(7)
% plot(control_time,contacts(:,1),"g*");
% hold on
% plot(mocap_t,contact_labels(:,1), "b*");
% ylim([-0.5 1.5]);
% legend("GRF","mocap");
% 
% %% to run this session, run utils/mocap_lcm_sync.m first.
% figure(8)
% plot(control_time(1,1:end-1),F_magnitude(:,1));
% hold on
% plot(mocap_t, 500*mocap_leg_z(:,1)+170);
% hold on
% plot(control_time(1,contacts(:,1)),F_magnitude(contacts(:,1),1), "b*");
% hold on
% plot(mocap_t(1,contact_labels(:,1)),500*mocap_leg_z(contact_labels(:,1),1)+170, "r*");
% legend("GRF","mocap\_leg\_position","contact\_GRF","contact\_mocap");

%%
save('sync_data.mat', ...
     'control_time', 'q', 'p', 'qd', 'v', 'tau_est', 'contacts', ...
     'imu_time', 'imu_acc', 'imu_omega', 'imu_rpy', 'imu_quat');
 
 
 function [start_idx, end_idx, t, x] = crop_data(t_init, x_init, start_t, end_t)
    start_idx = knnsearch(t_init', start_t);
    end_idx = knnsearch(t_init', end_t);
    x = x_init(start_idx:end_idx, :);
    t = t_init(start_idx:end_idx) - t_init(start_idx);
end