clear; clc;

%% add gen path
cur_pth = pwd;
gen_pth = append(cur_pth, '/gen/dyn/eigen/mex');
addpath(gen_pth);

%% add thirdparty path

thirdparty_pth = append(cur_pth, '/thirdparty');
addpath(genpath(thirdparty_pth));
%% load data
data_pth = "C:\Users\tylin\Documents\MATLAB\data";
addpath(data_pth);
load("lcmlog-2020-08-29-00.mat");

% plot(1:size(leg_control_data.tau_est(:,3)),leg_control_data.tau_est(:,3));

% find start index
start_idx = find(leg_control_data.tau_est(:,3)>0,1);
% start_idx = 551047;
control_time = leg_control_data.lcm_timestamp(start_idx:end);
% plot(leg_control_data.lcm_timestamp(start_idx:end),leg_control_data.tau_est(start_idx:end,3));

% extract data
q = leg_control_data.q(start_idx:end,:);
qd = leg_control_data.qd(start_idx:end,:);
tau_est = leg_control_data.tau_est(start_idx:end,:);
control_timestamp = leg_control_data.lcm_timestamp(start_idx:end);

% imu is in 1000 Hz, encoder signal is in 500 Hz
% downsample imu to 500 Hz and match the control timestamp
imu_time_1000 = microstrain.lcm_timestamp;
imu_time_ds_idx = knnsearch(imu_time_1000',control_time');
% use flip to turn rpy to ypr
imu_rpy = flip(microstrain.rpy(imu_time_ds_idx,:),2);
imu_drpy = flip(microstrain.omega(imu_time_ds_idx,:),2);

% append body orientation from imu in q. q becomes a n*18 array.
% body_x body_y body_z body_y body_p body_r encoder_1 ... encoder_12
num_data = length(q(:,1));
ctest = zeros(num_data,3);
q = [zeros(num_data,3)'; imu_rpy'; q']';
qd = [zeros(num_data,3)'; imu_drpy'; qd']';


%% compute angular acceleration from angular velocity
% we apply a median filter here
% dt is the time step caculate from lcm_timestamp. However, it might be
% noisy. The frequency of the signal is 500 Hz, which means the time step
% is around 0.002 sec. The effective digit in the lcm_timestamp is only 3rd
% decimal place, which indicates the 4th decimal place in lcm_timestamp is
% just a guessing number. 

tic
diff_qd = qd(1:num_data-1,:) - qd(2:num_data,:);
dt = control_timestamp(1,1:num_data-1) - control_timestamp(1,2:num_data);
diff_qd_fil = medfilt1(diff_qd,15,[],1);
qdd = diff_qd_fil ./ dt';
% qdd = diff_qd_fil ./ 0.002;
qdd = medfilt1(qdd,15,[],1);
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
% De = De_mini_cheetah_mex(q(1,:)');
% Ce = Ce_mini_cheetah_mex(q(1,:)',qd(1,:)');
% Ge = Ge_mini_cheetah_mex(q(1,:)');
B = [zeros(6,12); eye(12)];
% J_fr = Jp_fr_foot_mex(q(1,:)');
% J_fl = Jp_fl_foot_mex(q(1,:)');
% J_hr = Jp_hr_foot_mex(q(1,:)');
% J_hl = Jp_hl_foot_mex(q(1,:)');

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

% F = zeros(num_data-10,12);
% tic
% filter_size = 10;
% % De*qdd + Ce*qd + Ge = B*tau + J'*F
% for i = filter_size:num_data
%     % Get matrices from the dynamic model
%     De = De_mini_cheetah_mex(q(i,:)');
%     Ce = Ce_mini_cheetah_mex(q(i,:)',qd(i,:)');
%     Ge = Ge_mini_cheetah_mex(q(i,:)');
%     
%     % construct the Jacobian matrix
%     J_fr = Jp_fr_foot_mex(q(i,:)');
%     J_fl = Jp_fl_foot_mex(q(i,:)');
%     J_hr = Jp_hr_foot_mex(q(i,:)');
%     J_hl = Jp_hl_foot_mex(q(i,:)');
%     J = [J_fr; J_fl; J_hr; J_hl];
%     
%     qdd = zeros(1,length(qd(1,:)));
%     for j = 1:length(qd(1,:))
%         p = polyfit(i-filter_size+1:i,qd(i-filter_size+1:i,j),2);
%         dp = polyder(p);
%         qdd(j) = polyval(dp,i); 
%     end
%     F(i,:) = (J' \ (De*qdd'+Ce*qd(i,:)'+Ge-B*tau_est(i,:)'))';
% 
% %     F(i,:) = (J' \ (De*qdd(i,:)'+Ce*qd(i,:)'+Ge-B*tau_est(i,:)'))';
% 
% end
% toc

%%
figure(3)
plot(1:length(F(:,3)),F(:,3));
hold on
plot(1:length(F(:,6)),F(:,6));
hold on
plot(1:length(F(:,9)),F(:,9));
hold on
plot(1:length(F(:,12)),F(:,12));
hold on
plot(1:length(tau_est(:,3)),tau_est(:,3));
legend("GRF\_fr","GRF\_fl","GRF\_hr","GRF\_hl");

