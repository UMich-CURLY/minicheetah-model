# ----------------------------------------------------------------------------
#  Copyright 2021, Harrison Chen <hlfchen@umich.edu>, Tzu-yuan Lin <tzuyuan@umich.edu>
#  All Rights Reserved
#  See LICENSE for the license information
#  -------------------------------------------------------------------------- */

#
# @file   mat2lcm.py
# @author Harrison Chen, Tzu-yuan Lin
# @brief  
# @date   September 20, 2019


import lcm
import numpy as np
import scipy.io as sio
import time
from lcm_type import contact_t, groundtruth_t, imu_t, legcontrol_t

# NOTE: one potential source of confusion is that the loaded dictionary
# variables share similar names to the message types with "_t" endings: these
# hold the timestamps for each of the different sets of messages

# 3 CHANNELS:
#
# CHANNEL 1: 'contact' (** previously we used mocap to detect contact. now we don't. need to change the naming in the future **)
# int8_t  num_legs;
# double  mocap_timestamp;
# int8_t  contact[num_legs];
#
# CHANNEL 2: 'leg_control_data'
# int8_t  num_joints;
# double  lcm_timestamp;
# double  q[num_joints];
# double  p[num_joints];
# double  qd[num_joints];
# double  v[num_joints];
# double  tau_est[num_joints];
#
# CHANNEL 3: 'microstrain'
# double  lcm_timestamp;
# double  acc[3];
# double  omega[3];
# double  rpy[3];
# double  quat[4];

def main():
    PATH = '/media/curly_ssd_justin/data/sync_data_small_threshold_remove_f10.mat'
    sync_data = sio.loadmat(PATH)

    log = lcm.EventLog('sync_data_small_threshold_remove_f10.log', mode='w', overwrite=True)
    utime = int(time.time() * 10**6)
    num_legs = 4
    num_joints = 3 * num_legs

    # pull variables from mat file
    m_t = sync_data['control_time'].flatten().tolist()
    labels = sync_data['contacts'].tolist()
    
    # c_t = sync_data['contact_t'].flatten().tolist()
    # tau_fb = sync_data['lcm_tau_fb'].tolist()
    # tau_ff = sync_data['lcm_tau_ff'].tolist()

    l_t = sync_data['control_time'].flatten().tolist()
    q = sync_data['q'].tolist()
    p = sync_data['p'].tolist()
    qd = sync_data['qd'].tolist()
    v = sync_data['v'].tolist()
    tau_est = sync_data['tau_est'].tolist()

    i_t = sync_data['imu_time'].flatten().tolist()
    acc = sync_data['imu_acc'].tolist()
    omega = sync_data['imu_omega'].tolist()
    rpy = sync_data['imu_rpy'].tolist()
    quat = sync_data['imu_quat'].tolist()

    # combine all timestamps, remove duplicates, and sort
    # m_t is equal to l_t now. so we only add up l_t and i_t here.
    all_t = list(set(l_t + i_t))
    all_t.sort()
    
    # indices to move iterate through different messages
    m, l, i = 0, 0, 0
    complete_flag = [False] * 3

    for current_timestamp in all_t:
        if not complete_flag[0] and m_t[m] == current_timestamp:
            msg = groundtruth_t()
            msg.num_legs = num_legs
            msg.mocap_timestamp = m_t[m]
            msg.contact = labels[m]

            log.write_event(utime + int(10**6 * m_t[m]), 'ground_truth', msg.encode())
            m += 1
            if m == len(m_t):
                complete_flag[0] = True
                print('ground_truth complete')
        
        # if not complete_flag[1] and c_t[c] == current_timestamp:
        #     msg = contact_t()
        #     msg.num_joints = num_joints
        #     msg.lcm_timestamp = c_t[c]
        #     msg.tau_feed_back = tau_fb[c]
        #     msg.tau_feed_forward = tau_ff[c]

        #     log.write_event(utime + int(10**6 * c_t[c]), 'contact_data', msg.encode())
        #     c += 1
        #     if c == len(c_t):
        #         complete_flag[1] = True
        #         print('contact_data complete')
        
        if not complete_flag[1] and l_t[l] == current_timestamp:
            msg = legcontrol_t()
            msg.num_joints = num_joints
            msg.lcm_timestamp = l_t[l]
            msg.q = q[l]
            msg.p = p[l]
            msg.qd = qd[l]
            msg.v = v[l]
            msg.tau_est = tau_est[l]
            
            log.write_event(utime + int(10**6 * l_t[l]), 'leg_control_data', msg.encode())
            l += 1
            if l == len(l_t):
                complete_flag[1] = True
                print('leg_control_data complete')
        
        if not complete_flag[2] and i_t[i] == current_timestamp:
            msg = imu_t()
            msg.lcm_timestamp = i_t[i]
            msg.acc = acc[i]
            msg.omega = omega[i]
            msg.rpy = rpy[i]
            msg.quat = quat[i]

            log.write_event(utime + int(10**6 * i_t[i]), 'microstrain', msg.encode())
            i += 1
            if i == len(i_t):
                complete_flag[2] = True
                print('microstrain complete')
    
    print('\nDONE!')

if __name__ == '__main__':
    main()