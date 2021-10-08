% clear; clc;

%% add path
cur_pth = pwd;
gen_pth = append(cur_pth, '/gen/dyn/eigen/mex');
addpath(gen_pth);
thirdparty_pth = append(cur_pth, '/thirdparty');
addpath(genpath(thirdparty_pth));
data_saved_pth = append(cur_pth,'/data');
addpath(data_saved_pth);

%% load data
data_pth = "D:\data\contact_data\2021-09-07_terrain_estimation\raw_mat";
data_saved_pth = data_pth;
addpath(data_pth);
data_name = "lcmlog-2021-09-06_00";
load(append(data_name,".mat"));

%%
% gt_data_pth = "D:\data\contact_data\train\mat";
% gt_data_name = "forest";
% gt = load(append("D:\data\contact_data\train\mat\",data_name,"_network_data.mat"));
% gt = load("D:\data\contact_data\train\mat\lcmlog-2020-08-29-00_MAir_network_data.mat");
%%
% figure(2147483646);
% plot(leg_control_data.lcm_timestamp,leg_control_data.p(1:end,3));
% hold on
% plot(leg_control_data.lcm_timestamp,leg_control_data.p(1:end-1,3));
% plot(microstrain.lcm_timestamp,microstrain.acc(1:end,3));
%%
% find start index

% 2021 09 06
% 00
lcm_start_time = 0;
lcm_end_time = 55.6;

% 01
% lcm_start_time = 0;
% lcm_end_time = 90.1342;

% 02
% lcm_start_time = 0;
% lcm_end_time = 33.3665;

% 03
% lcm_start_time = 0;
% lcm_end_time = 161.046;

% 04
% lcm_start_time = 0;
% lcm_end_time = 47.9795;

% 05 - 07
% lcm_start_time = 0;
% lcm_end_time = microstrain.lcm_timestamp(1,end);

% 08
% lcm_start_time = 0;
% lcm_end_time = 105;

% 09
% lcm_start_time = 0;
% lcm_end_time = 84;

% 05-29

% 06-02

% lcm_start_time = 0;
% lcm_end_time = 51;

% 06-05

% 00
% lcm_start_time = 0;
% lcm_end_time = 57.68;

% 01
% lcm_start_time = 1.368;
% lcm_end_time = 61;

% 02
% lcm_start_time = 18.33;
% lcm_end_time = 72.9;

% 03
% lcm_start_time = 25.83;
% lcm_end_time = 94.6;

% 04
% lcm_start_time = 18.45;
% lcm_end_time = 82.72;

% 05
% lcm_start_time = 47.96;
% lcm_end_time = 119.71;

% 06
% lcm_start_time = 39.55;
% lcm_end_time = 107.65;

% 07
% lcm_start_time = 33.3;
% lcm_end_time = 117.53;

% 08
% lcm_start_time = 26;
% lcm_end_time = 111.56;

% % 09
% lcm_start_time = 26;
% lcm_end_time = 106;

% side walk
% lcm_start_time = 8.139;
% lcm_end_time = 159.46;

% forest
% lcm_start_time = 7.918;
% lcm_end_time = 129.5;

% forest 2
% lcm_start_time = 24.93;
% lcm_end_time = 117.614;

% mocap 08 29 00
% lcm_start_time = 1115.4;
% lcm_end_time = 1232.23;

% 3
% lcm_start_time = 21.119;
% lcm_end_time = 137.552;

% 4
% lcm_start_time = 9.44041;
% lcm_end_time = 131.881;

% 5 short
% lcm_start_time = 10.5162;
% lcm_end_time = 57.1235;

% 6
% lcm_start_time = 6.73;
% lcm_end_time = 147.888;

% 7
% lcm_start_time = 15.9883;
% lcm_end_time = 75.0456;

% 8 not usable
% lcm_start_time = 15.9883;
% lcm_end_time = 75.0456;

% 10 jumping
% lcm_start_time = 7.92166;
% lcm_end_time = 131.127;

control_time = leg_control_data.lcm_timestamp;
wbc_time = wbc_lcm_data.lcm_timestamp;

% extract leg_control data
[start_idx, end_idx, control_time, p] = crop_data(control_time, leg_control_data.p, lcm_start_time, lcm_end_time);
q = leg_control_data.q(start_idx:end_idx,:);
qd = leg_control_data.qd(start_idx:end_idx,:);
v = leg_control_data.v(start_idx:end_idx,:);
tau_est = leg_control_data.tau_est(start_idx:end_idx,:);

[start_idx, end_idx, wbc_time, gait_cycle_contacts] = crop_data(wbc_time, wbc_lcm_data.contact_est, lcm_start_time, lcm_end_time);
wbc_F = wbc_lcm_data.Fr(start_idx:end_idx,:);
gait_cycle_contacts = logical(gait_cycle_contacts);

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
q = [zeros(num_data,3)'; imu_ypr_ds'; q']';
qd = [zeros(num_data,3)'; imu_dypr_ds'; qd']';


%% compute angular acceleration from angular velocity
% we apply a median filter here
% dt is the time step caculate from lcm_timestamp. 

tic
diff_qd = qd(1:num_data-1,:) - qd(2:num_data,:);
dt = control_time(1,1:num_data-1) - control_time(1,2:num_data);
% diff_qd_fil = medfilt1(diff_qd,filter_size,[],1);
% diff_qd_fil = lowpass(diff_qd,10,500);
% diff_qd_fil = diff_qd;

d1 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.001,'DesignMethod','butter');
diff_qd_fil = filtfilt(d1, diff_qd);

% lab 07 lab 10 (bounding gait)
% cutoff_freq = 0.08;

% rest of the dataset
cutoff_freq = 0.04;

d2 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',cutoff_freq,'DesignMethod','butter');
p_fil = filtfilt(d2, p);


d3 = designfilt('lowpassiir','FilterOrder',12, ...
    'HalfPowerFrequency',0.04,'DesignMethod','butter');
v_fil = filtfilt(d3, v);
% p_fil = lowpass(p,5,500);

qdd = diff_qd_fil ./ dt';
% qdd = diff_qd_fil ./ 0.002;
% qdd = medfilt1(qdd,filter_size,[],1);

% tau_est = lowpass(tau_est,1,500);
toc

% figure(1);
% plot(1:num_data-1,p_fil(1:num_data-1,9));
% hold on
% plot(1:num_data-1,p(1:num_data-1,9));

% figure(1);
% plot(1:num_data-1,qdd(1:num_data-1,9));
% hold on;
% plot(1:num_data-1, qd(1:num_data-1,3));
% hold on;
% plot(1:num_data-1, diff_qd(1:num_data-1,3));
% legend("qdd","qd","diff\_qd",'FontSize', 18)

% title('$\ddot{q}$ via Numerical Differentiation','Interpreter','latex','FontSize', 18);
% legend('$\ddot{q}$','Interpreter','latex','FontSize', 18)

% figure(2);
% plot(1:num_data-1, qd(1:num_data-1,3));
% hold on;
% plot(1:num_data-1, diff_qd(1:num_data-1,3));
% legend("qd","diff\_qd",'FontSize', 18)

%% Compute Ground Reaction Force
B = [zeros(6,12); eye(12)];

F = zeros(num_data-1,12);
% svd_J = zeros(size(F,1),12);
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
%     svd_J(i,:) = svd(J);
    F(i,:) = (pinv(J') * (De*qdd(i,:)'+Ce*qd(i,:)'+Ge-B*tau_est(i,:)'))';
end
toc
F = lowpass(F,1,500);

%% remove first data, because we take diff for qdd
control_time = control_time(2:end);
q = q(2:end,:);
qd = qd(2:end,:);
v = v(2:end,:);
p = p(2:end,:);
tau_est = tau_est(2:end,:);
F = F(1:end,:);
p_fil = p_fil(2:end,:);

new_num_data = num_data-1;

%% compute force magnitude
F_magnitude = zeros(new_num_data,4);
for i=1:4
    F_magnitude(:,i) = sqrt(F(:,3*i-2).^2+F(:,3*i-1).^2+F(:,3*i).^2);
end

%% segment force contact
F_threshold = 30;
F_contacts = zeros(size(F,1),4);
for i = 1:4
    F_contacts(:,i) = F(:,3*i)>=F_threshold;
end
F_contacts = logical(F_contacts);
%% visualize the forces
% figure(3)
% plot(1:length(F(:,3)),F(:,3));
% hold on
% yline(30, 'LineWidth',3);
% hold on
% yline(0, 'LineWidth', 3);
% 
% %%
% F_contacts = logical(F_contacts);
% for i = 1 : 4
% figure(4+i)
% plot(1:length(F(:,3*i)),F(:,3*i));
% hold on
% plot(find(F_contacts(:,i)),F(F_contacts(:,i),3*i),'b*');
% hold on
% yline(30, 'LineWidth',3);
% hold on
% yline(0, 'LineWidth', 3);
% end

% hold on
% plot(1:length(wbc_F),wbc_F(:,3));
% hold on
% plot(1:length(F(:,6)),F(:,6));
% hold on
% plot(1:length(F(:,9)),F(:,9));
% hold on
% plot(1:length(F(:,12)),F(:,12));
% hold on
% plot(1:length(tau_est(:,3)),tau_est(:,3));
% title("Estimated GRF");
% legend("right front leg z")
% figure(3)
% % plot(2:length(F_magnitude(:,1)),medfilt1(diff(F_magnitude(:,1)),3,[],1));
% % hold on
% % plot(2:length(F_magnitude(:,1)),diff(F_magnitude(:,1)));
% 
% plot(1:length(F_magnitude(:,1)),F_magnitude(:,1));
% hold on
% plot(1:length(F_magnitude(:,2)),F_magnitude(:,2));
% hold on
% plot(1:length(F_magnitude(:,3)),F_magnitude(:,3));
% hold on
% plot(1:length(F_magnitude(:,4)),F_magnitude(:,4));
% 
% legend("GRF\_fr","GRF\_fl","GRF\_hr","GRF\_hl");
% title("Estimated GRF");
% % legend("right front leg")
% ylabel("Force Magnitude (N)");
% xlabel("index");


%%
%% lab 07
est_period = 0.19;

est_period_idx = floor(est_period / 0.002);
est_contact_idx = 30;
foot_z_pos = [p_fil(:,3), p_fil(:,6), p_fil(:,9), p_fil(:,12)];
% foot_local_max = islocalmax(foot_z_pos,'MinSeparation',est_period_idx) & (foot_z_pos > mean(foot_z_pos, 1, 'omitnan'));
foot_local_max = islocalmax(foot_z_pos,'MinSeparation',est_period_idx);
% contact_labels = islocalmin(foot_z_pos) & (foot_z_pos < mean(foot_z_pos, 1, 'omitnan'));
% foot_local_min = islocalmin(foot_z_pos) & (foot_z_pos < mean(foot_z_pos, 1, 'omitnan'));
% contact_labels = islocalmin(foot_z_pos);
contact_labels = false(num_data-1,4);
foot_local_min = islocalmin(foot_z_pos); 


for l = 1:size(foot_z_pos, 2)
    contact_singleleg = foot_local_min(:,l);
    peak_singleleg = foot_local_max(:,l);

    % indexes at which local mins and maxes occurred
    contact_idxs = find(contact_singleleg);
    peak_idxs = find(peak_singleleg);

    % for clarity/safety, we start searching for contact only after 1st peak
%     contact_idxs(contact_idxs < peak_idxs(1)) = 0;
    i = find(contact_idxs, 1);
    j = 1;
    contact_end = 1;
    
    while i <= size(contact_idxs, 1) && j <= size(peak_idxs, 1)
        contact_singleleg(contact_idxs(i)) = 0;
        contact_start = contact_idxs(i);
        next_peak = peak_idxs(j);
        
        count = 0;
        while i <= size(contact_idxs, 1) && contact_idxs(i) < next_peak 
            contact_end = contact_idxs(i);
            i = i + 1;
            count = count + 1;
        end
        
        
        if count == 1
           contact_start = contact_end - est_contact_idx;
           if contact_start<1
               contact_start=1;
           end
        end
        contact_singleleg(contact_start:contact_end) = 1;
        j = j + 1;
    end
    
    contact_labels(:,l) = contact_singleleg;
end

contacts = contact_labels;

%%
i = 1;
figure(24+i)
idx_list = 1:size(foot_z_pos(:,i));
plot(idx_list,foot_z_pos(:,i));
hold on

plot(idx_list(1,contacts(:,i)),foot_z_pos(contacts(:,i),i), "g*");
hold on
plot(idx_list(1,foot_local_max(:,i)),foot_z_pos(foot_local_max(:,i),i), "r*");
hold on

plot(idx_list(1,foot_local_min(:,i)),foot_z_pos(foot_local_min(:,i),i), "b*");
% hold on
% plot(idx_list,p(:,3*i),'black');
% hold on
% plot(idx_list(1,contacts(:,i)),p(contacts(:,i),3*i), "m*");
% plot(1:size(p_fil,1),p_fil(foot_local_max))

%%
% d4 = designfilt('lowpassiir','FilterOrder',12, ...
%     'HalfPowerFrequency',0.05,'DesignMethod','butter');
% v_fil = filtfilt(d4, v);
% 
% plot(1:length(v_fil(:,3)),v_fil(:,3));



%% save manual data
% save(append('contacts_manual_fix_',append(data_name,".mat")), ...
%          'manual_label_fix','lcm_start_time','lcm_end_time');


%% print foot_z_pos with respect to control time
% for i = 1:4
%     figure(20+i)
%     plot(control_time(1,1:end),foot_z_pos(:,i));
%     hold on
%     plot(control_time(1,contact_labels(:,i)),foot_z_pos(contact_labels(:,i),i), "g*");
%     hold on
%     plot(control_time(1,foot_local_max(:,i)),foot_z_pos(foot_local_max(:,i),i), "r*");
%     hold on
% 
%     plot(control_time(1,foot_local_min(:,i)),foot_z_pos(foot_local_min(:,i),i), "b*");
%     hold on
% 
%     legend("foot_pos","contacts","local\_max","local\_min");
%     title("foot\_position"+i)
% end

%% visualize force versus local max/ min
% figure(5)
% plot(control_time(1,1:end),F_magnitude(:,1));
% hold on
% plot(control_time(1,contacts(:,1)),F_magnitude(contacts(:,1),1), "g*");
% hold on
% plot(control_time(1,local_max(:,1)),F_magnitude(local_max(:,1),1), "r*");
% hold on
% 
% plot(control_time(1,local_min(:,1)),F_magnitude(local_min(:,1),1), "b*");
% hold on
% 
% legend("GRF","contacts","local\_max","local\_min");

%% visualization GRF with foot position
% for i =1:4
%     figure(13+i)
%     plot(control_time(1,1:end),F_magnitude(:,i));
%     hold on
%     plot(control_time(1,contacts(:,i)),F_magnitude(contacts(:,i),i), "b*");
%     hold on
%     plot(control_time(1,1:end),500*p(1:end,3*i)+180);
%     hold on
%     plot(control_time(1,contacts(:,i)),500*p(contacts(:,i),3*i)+180, "g*");
%     hold on
%     legend("GRF","contacts","leg\_pos\_z","contacts");
%     title(i);
% end


%% map control data (500 Hz) to imu frequency (1000 Hz)

% find the highest frequency set of data and make it univ_t
t_vars = ["control_time", "wbc_time" , "imu_time"];
t_sizes = [size(control_time, 2), size(wbc_time,2), size(imu_time, 2)];
[~, max_t_idx] = max(t_sizes);
% univ_t will be the set of timestamps that we change all inputs to correspond to
univ_t = eval(t_vars(max_t_idx));

univ_idx = zeros(size(univ_t, 2), size(t_vars, 2));
for i = 1:size(t_vars, 2)
    t_i = eval(t_vars(i));
    univ_idx(:,i) = knnsearch(t_i', univ_t');
end

control_time = imu_time;
contacts = contacts(univ_idx(:,1),:);
F_contacts = F_contacts(univ_idx(:,1),:);
p = p(univ_idx(:,1),:);
q = q(univ_idx(:,1),:);
qd = qd(univ_idx(:,1),:);
v = v(univ_idx(:,1),:);
tau_est = tau_est(univ_idx(:,1),:);
F = F(univ_idx(:,1),:);
gait_cycle_contacts = gait_cycle_contacts(univ_idx(:,2),:);

% remove body states in q and qd
q = q(:,7:end);
qd = qd(:,7:end);

%% visualize after mapping to the highest frequency
for i = 1:4
    figure(28+i)
    plot(imu_time(1,1:end),p(:,3*i));
    hold on
    plot(imu_time(1,contacts(:,i)),p(contacts(:,i),3*i), "g*");

    
    legend("foot_pos","contacts");
    title("foot\_position"+i)
end

% for i = 1:4
%     figure(32+i)
%     plot(imu_time(1,1:end),v(:,3*i));
%     hold on
%     plot(imu_time(1,contacts(:,i)),v(contacts(:,i),3*i), "g*");
%     hold on
%     plot(imu_time(1,1:end),v(:,3*i-1));
%     hold on
%     plot(imu_time(1,1:end),v(:,3*i-2));
%     hold on
%     
%     legend("foot_v_z","contacts");
%     title("foot\_velocity"+i)
% end


% figure(33)
% plot(imu_time(1,1:end),F(:,3*i));

% figure(34)
% plot(imu_time(1,1:end),p(:,3*3));
% hold on
% plot(imu_time(1,contacts(:,3)),p(contacts(:,3),3*3), "g*");
% 
% 
% legend('foot position (Z)','contacts','FontSize', 18);
% title("Foot Position in Body Frame",'FontSize', 18)

%%
% figure(136)
% plot(imu_time(1,1:end),tau_est(:,1));
% hold on
% plot(imu_time(1,1:end),tau_est(:,2));
% hold on
% plot(imu_time(1,1:end),tau_est(:,3));

% gait_cycle_contacts = logical(gait_cycle_contacts);
% figure(3524)
% plot(control_time(1,1:end),p(:,3*3));
% hold on
% plot(control_time(1,gait_cycle_contacts(:,3)),p(gait_cycle_contacts(:,3),3*3), "m*");
% figure(3524)
% plot(1:length(F(:,3)),F(:,3));
% hold on
% yline(30, 'LineWidth',3);
% hold on
% plot(1:length(wbc_F),wbc_F(:,3));
% hold on
% plot(find(gait_cycle_contacts(:,1)),wbc_F(gait_cycle_contacts(:,1),3), "m*");

%%
% for i = 1:4
%     figure(32+i)
%     idx_list = 1:size(foot_z_pos(:,i));
%     plot(imu_time,p(:,3*i));
%     hold on
% 
%     plot(imu_time(1,contacts(:,i)),p(contacts(:,i),3*i), "g*");
%     hold on
% 
%     plot(gt.imu_time,gt.p(:,3*i)+0.02,'black');
%     hold on
% 
%     plot(gt.imu_time(1,gt.contacts(:,i)),gt.p(gt.contacts(:,i),3*i)+0.02, "m*");
%     hold on
% end
% legend('foot position','new method contacts','foot position','gt contacts');
% plot(idx_list(1,foot_local_max(:,i)),foot_z_pos(foot_local_max(:,i),i), "r*");
% hold on
% 
% plot(idx_list(1,foot_local_min(:,i)),foot_z_pos(foot_local_min(:,i),i), "b*");
% hold on
% plot(idx_list,p(:,3*i),'black');
% hold on
% plot(idx_list(1,contacts(:,i)),p(contacts(:,i),3*i), "m*");

%% compute new contact acc

% correct_contact = contacts == gt.contacts;
% num_correct_contact = sum(sum(correct_contact));
% acc_contact = num_correct_contact/(size(gt.contacts,1)*4)


%% save data

% save the data
save(append(data_saved_pth,'/',data_name,'_network_data.mat'), ...
     'control_time', 'q', 'p', 'qd', 'v', 'tau_est', 'F_contacts', 'F', ...
     'imu_time', 'imu_acc', 'imu_omega', 'imu_rpy', 'imu_quat', 'gait_cycle_contacts', 'contacts');
%  
 
 function [start_idx, end_idx, t, x] = crop_data(t_init, x_init, start_t, end_t)
    start_idx = knnsearch(t_init', start_t);
    end_idx = knnsearch(t_init', end_t);
    x = x_init(start_idx:end_idx, :);
    t = t_init(start_idx:end_idx) - t_init(start_idx);
end