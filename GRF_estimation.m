% clear; clc;

%% add path
cur_pth = pwd;
gen_pth = append(cur_pth, '/gen/dyn/eigen/mex');
addpath(gen_pth);
thirdparty_pth = append(cur_pth, '/thirdparty');
addpath(genpath(thirdparty_pth));
data_saved_pth = append(cur_pth,'/data');
addpath(data_saved_pth);
addpath(append(cur_pth,'/data/manual_fix_labels'));

%% load data
data_pth = "D:\data\contact_data\2021-06-05_forset_data\raw_mat";
data_saved_pth = data_pth;
addpath(data_pth);
data_name = "lcmlog-2021-06-05_04_small_pebble";
load(append(data_name,".mat"));

manual_fix = false; 
%%
figure(2147483646);
plot(leg_control_data.lcm_timestamp,leg_control_data.p(1:end,3));
% hold on
% plot(leg_control_data.lcm_timestamp,leg_control_data.p(1:end-1,3));
% plot(microstrain.lcm_timestamp,microstrain.acc(1:end,3));
%%
% find start index
filter_size = 11;
est_period = 0.15;

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
lcm_start_time = 18.45;
lcm_end_time = 82.72;

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

% load manual data from file.

if manual_fix
    load(append('contacts_manual_fix_',append(data_name,".mat")));
end
control_time = leg_control_data.lcm_timestamp;

% extract leg_control data
[start_idx, end_idx, control_time, p] = crop_data(control_time, leg_control_data.p, lcm_start_time, lcm_end_time);
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
diff_qd_fil = medfilt1(diff_qd,filter_size,[],1);
qdd = diff_qd_fil ./ dt';
% qdd = diff_qd_fil ./ 0.002;
% qdd = medfilt1(qdd,filter_size,[],1);
toc

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

%% remove first data, because we take diff for qdd
control_time = control_time(2:end);
q = q(2:end,:);
qd = qd(2:end,:);
v = v(2:end,:);
p = p(2:end,:);
tau_est = tau_est(2:end,:);
F = F(1:end,:);

new_num_data = num_data-1;

%% compute force magnitude
F_magnitude = zeros(new_num_data,4);
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

%% detect contact from GRF
period_thres = 0.04;

local_max = islocalmax(F_magnitude) & (F_magnitude > mean(F_magnitude, 1, 'omitnan'));
local_min = islocalmin(F_magnitude) & (F_magnitude < mean(F_magnitude(islocalmin(F_magnitude)), 1, 'omitnan'));

contacts = false(new_num_data,4);
remove_first_couple = 10;
contact_shift = 9;
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
           [~,actual_peak_idx] = max(F_magnitude(contact_start:contact_start+20,i)); 
%            
           contacts(contact_start+actual_peak_idx-1+remove_first_couple:contact_end,i) = 1; 
%            contact_idx = floor((contact_start+actual_peak_idx-1+remove_first_couple+contact_end)/2);
%            contacts(contact_idx-contact_shift:contact_idx+contact_shift,i) = 1;
        end
        
        k = find(max_idx(1:end)>contact_end,1,'first');
        contact_start = max_idx(k);
        
    end
    contacts(F_magnitude(:,i) < mean(F_magnitude(:,i), 1, 'omitnan'),i)=0;
end
toc

%% detect contact using foot position from FK
est_period_idx = floor(est_period / 0.002);
foot_z_pos = [p(:,3), p(:,6), p(:,9), p(:,12)];
foot_local_max = islocalmax(foot_z_pos,'MinSeparation',est_period_idx) & (foot_z_pos > mean(foot_z_pos, 1, 'omitnan'));
% contact_labels = islocalmin(foot_z_pos) & (foot_z_pos < mean(foot_z_pos, 1, 'omitnan'));
% foot_local_min = islocalmin(foot_z_pos) & (foot_z_pos < mean(foot_z_pos, 1, 'omitnan'));
contact_labels = islocalmin(foot_z_pos);
foot_local_min = islocalmin(foot_z_pos); 

for l = 1:size(foot_z_pos, 2)
    contact_singleleg = contact_labels(:,l);
    peak_singleleg = foot_local_max(:,l);

    % indexes at which local mins and maxes occurred
    contact_idxs = find(contact_singleleg);
    peak_idxs = find(peak_singleleg);

    % for clarity/safety, we start searching for contact only after 1st peak
    contact_idxs(contact_idxs < peak_idxs(1)) = 0;
    i = find(contact_idxs, 1);
    j = 2;

    while i <= size(contact_idxs, 1) && j <= size(peak_idxs, 1)
        contact_singleleg(contact_idxs(i)) = 0;
        contact_start = contact_idxs(i+1);
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

contacts = contact_labels;


%% load manual contact labels and fix contact points

%     manual_label_fix = [];
%     load(append('contacts_manual_fix_',append(data_name,".mat")));


manual_label_fix_backup = manual_label_fix;
manual_fix = true;
if manual_fix
    for i=1:size(manual_label_fix,1)
        contacts(manual_label_fix(i,3):manual_label_fix(i,4),manual_label_fix(i,1))...
         = manual_label_fix(i,2);
    end
end


%% save manual data% save(append('contacts_manual_fix_',append(data_name,".mat")), ...
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

%% visualize f_z_pos with respect to index

for i = 1:4
    
    figure(24+i)
    idx_list = 1:size(foot_z_pos(:,i));
    plot(idx_list,foot_z_pos(:,i));
    hold on
    plot(idx_list(1,contacts(:,i)),foot_z_pos(contacts(:,i),i), "g*");
    hold on
    plot(idx_list(1,foot_local_max(:,i)),foot_z_pos(foot_local_max(:,i),i), "r*");
    hold on

    plot(idx_list(1,foot_local_min(:,i)),foot_z_pos(foot_local_min(:,i),i), "b*");
    hold on

    legend("foot_pos","contacts","local\_max","local\_min");
    title("foot\_position"+i)
end

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

%% load mocap data to verify
% mocap_pth = "C:\Users\tylin\Documents\MATLAB\data\08282020_trial1_data";
% mocap_struct = who(matfile(mocap_pth));
% mocap_struct = mocap_struct{1};
% mocap = load(mocap_pth, mocap_struct);
% mocap = mocap.(mocap_struct);
% 
% mocap_body_idx = 2;
% mocap_leg_idx = [3, 6, 5, 4];
% 
% % build transformation matrix to find leg coordinates relative to body
% p_body = mocap.RigidBodies.Positions(mocap_body_idx, :, :);
% p_body = reshape(p_body, 3, 1, []);
% R_body = mocap.RigidBodies.Rotations(mocap_body_idx, :, :);
% R_body = reshape(R_body, 3, 3, []);
% 
% g_body_inv = repmat(eye(4, 4), [1, 1, size(p_body, 3)]);           % build inverse manually
% g_body_inv(1:3, 1:3, :) = pagetranspose(R_body);                    % R'
% g_body_inv(1:3, 4, :) = pagemtimes(-pagetranspose(R_body), p_body); % -R'p
% 
% mocap_leg = pagetranspose(cat(2, mocap.RigidBodies.Positions(mocap_leg_idx, :, :), ...
%                                     ones(size(mocap_leg_idx, 2), 1, size(p_body, 3))));
% mocap_leg = pagemtimes(g_body_inv, mocap_leg);
% mocap_leg_z = squeeze(mocap_leg(3,:,:))' .* (-1 / 1000);
% 
% % plot rough estimate
% T = 1 / mocap.FrameRate;
% mocap_t = 0:T:((size(mocap_leg_z, 1) - 1) * T);
% % plot(mocap_t, mocap_leg_z(:,1));
% 
% % mocap start and end time
% mocap_start_t = 4.34;
% mocap_end_t = 40.62;
% 
% [~, ~, mocap_t, mocap_leg_z] = crop_data(mocap_t, mocap_leg_z, mocap_start_t, mocap_end_t);

% plot contact with mocap data
% temp = -0.28*ones(new_num_data,1);
% temp(contacts(:,1),1) = -0.325;
% non_contacts = ~contacts;
% figure(6);
% plot(mocap_t, mocap_leg_z(:,1));
% hold on
% plot(control_time(1,contacts(:,1)),temp(contacts(:,1),1), "g*");
% hold on
% plot(control_time(1,non_contacts(:,1)),temp(non_contacts(:,1),1), "r*");
% 
% legend("mocap\_leg\_position","contacts","non\_contacts");

%% to run this session, run utils/mocap_lcm_sync.m first.
% for i=1:4
%     figure(6+i)
%     
%     plot(control_time,contacts(:,i),"g*");
%     hold on
%     plot(mocap_t,contact_labels(:,i), "b*");
%     ylim([-0.5 1.5]);
%     title(i);
%     legend("GRF","mocap");
% end

%% to run this session, run utils/mocap_lcm_sync.m first.
% figure(11)
% plot(control_time(1,1:end),F_magnitude(:,1));
% hold on
% plot(mocap_t, 500*mocap_leg_z(:,1)+170);
% hold on
% plot(control_time(1,contacts(:,1)),F_magnitude(contacts(:,1),1), "b*");
% hold on
% plot(mocap_t(1,contact_labels(:,1)),500*mocap_leg_z(contact_labels(:,1),1)+170, "r*");
% legend("GRF","mocap\_leg\_position","contact\_GRF","contact\_mocap");

%% map control data (500 Hz) to imu frequency (1000 Hz)

% find the highest frequency set of data and make it univ_t
t_vars = ["control_time", "imu_time"];
t_sizes = [size(control_time, 2), size(imu_time, 2)];
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
p = p(univ_idx(:,1),:);
q = q(univ_idx(:,1),:);
qd = qd(univ_idx(:,1),:);
v = v(univ_idx(:,1),:);
tau_est = tau_est(univ_idx(:,1),:);
F = F(univ_idx(:,1),:);

% remove body states in q and qd
q = q(:,7:end);
qd = qd(:,7:end);

%% visualize after mapping to the highest frequency
% for i = 1:4
%     figure(28+i)
%     plot(imu_time(1,1:end),p(:,3*i));
%     hold on
%     plot(imu_time(1,contacts(:,i)),p(contacts(:,i),3*i), "g*");
% 
%     
%     legend("foot_pos","contacts");
%     title("foot\_position"+i)
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


%% save data

% save the data
% save(append(data_saved_pth,'/',data_name,'_network_data.mat'), ...
%      'control_time', 'q', 'p', 'qd', 'v', 'tau_est', 'contacts', 'F', ...
%      'imu_time', 'imu_acc', 'imu_omega', 'imu_rpy', 'imu_quat');
%  
 
 function [start_idx, end_idx, t, x] = crop_data(t_init, x_init, start_t, end_t)
    start_idx = knnsearch(t_init', start_t);
    end_idx = knnsearch(t_init', end_t);
    x = x_init(start_idx:end_idx, :);
    t = t_init(start_idx:end_idx) - t_init(start_idx);
end