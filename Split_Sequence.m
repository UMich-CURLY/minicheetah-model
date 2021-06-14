%% add path
cur_pth = pwd;
gen_pth = append(cur_pth, '/gen/dyn/eigen/mex');
addpath(gen_pth);
thirdparty_pth = append(cur_pth, '/thirdparty');
addpath(genpath(thirdparty_pth));
data_saved_pth = append(cur_pth,'/data');
addpath(data_saved_pth);

%% load data
data_pth = "D:\data\contact_data\2021-06-05_forset_data\mat";
data_saved_pth = data_pth;
addpath(data_pth);
data_name = "lcmlog-2021-06-05_06_asphalt_road_and_rock_road_network_data";
load(append(data_name,".mat"));

%% split info
split_time = 32;
save_data = true;

%%
control_time_raw = control_time;
q_raw = q;
p_raw = p;
qd_raw = qd;
v_raw = v;
tau_est_raw = tau_est;
contacts_raw = contacts;
F_raw = F;
imu_time_raw = imu_time;
imu_acc_raw = imu_acc;
imu_omega_raw = imu_omega;
imu_rpy_raw = imu_rpy;
imu_quat_raw = imu_quat;

control_time = [];
q = [];
p = [];
qd = [];
v = [];
tau_est = [];
contacts = [];
F = [];
imu_time = [];
imu_acc = [];
imu_omega = [];
imu_rpy = [];
imu_quat = [];
%% create first half

split_idx = find(imu_time_raw>split_time+0.02,1,'first');

control_time = control_time_raw(:,1:split_idx-1);
q = q_raw(1:split_idx-1,:);
p = p_raw(1:split_idx-1,:);
qd = qd_raw(1:split_idx-1,:);
v = v_raw(1:split_idx-1,:);
tau_est = tau_est_raw(1:split_idx-1,:);
contacts = contacts_raw(1:split_idx-1,:);
F = F_raw(1:split_idx-1,:);
imu_time = imu_time_raw(:,1:split_idx-1);
imu_acc = imu_acc_raw(1:split_idx-1,:);
imu_omega = imu_omega_raw(1:split_idx-1,:);
imu_rpy = imu_rpy_raw(1:split_idx-1,:);
imu_quat = imu_quat_raw(1:split_idx-1,:);

if save_data
    save(append(data_saved_pth,'/',data_name,'-1.mat'), ...
     'control_time', 'q', 'p', 'qd', 'v', 'tau_est', 'contacts', 'F', ...
     'imu_time', 'imu_acc', 'imu_omega', 'imu_rpy', 'imu_quat');
end
%% create second half
control_time = [];
q = [];
p = [];
qd = [];
v = [];
tau_est = [];
contacts = [];
F = [];
imu_time = [];
imu_acc = [];
imu_omega = [];
imu_rpy = [];
imu_quat = [];

control_time = control_time_raw(:,split_idx:end)-control_time_raw(1,1);
q = q_raw(split_idx:end,:);
p = p_raw(split_idx:end,:);
qd = qd_raw(split_idx:end,:);
v = v_raw(split_idx:end,:);
tau_est = tau_est_raw(split_idx:end,:);
contacts = contacts_raw(split_idx:end,:);
F = F_raw(split_idx:end,:);
imu_time = imu_time_raw(:,split_idx:end)-imu_time_raw(1,1);
imu_acc = imu_acc_raw(split_idx:end,:);
imu_omega = imu_omega_raw(split_idx:end,:);
imu_rpy = imu_rpy_raw(split_idx:end,:);
imu_quat = imu_quat_raw(split_idx:end,:);

if save_data
    save(append(data_saved_pth,'/',data_name,'-2.mat'), ...
     'control_time', 'q', 'p', 'qd', 'v', 'tau_est', 'contacts', 'F', ...
     'imu_time', 'imu_acc', 'imu_omega', 'imu_rpy', 'imu_quat');
end