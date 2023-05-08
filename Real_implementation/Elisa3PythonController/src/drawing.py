import numpy as np
import functions
import time
import json
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
import copy
import pickle

import pandas as pd

P = 3

robot_N = 4
offset = 5
T = 1500 #3100 2487

if __name__ == "__main__":

    robot_history = dict.fromkeys(['0', '1', '2'], dict())

    key_set = ['orien', 'pos_x', 'pos_y', 'estimation_phi', 'estimation_x', 'estimation_y', 'cam_phi', 'cam_x', 'cam_y',
               'x', 'y', 'phi', 'P_k_odo', 'P_k_cam', 'OWA_w1', 'OWA_w2'] # 'P_k_odo', 'P_k_cam' na phi

    with open('./data/good_results_1_5/phase_line_4_line_video.p', 'rb') as fp: #6
    #with open('./data/saved_data_t0_RUN6.p', 'rb') as fp:  # 6
        if P == 3:
            b = pickle.load(fp)
        elif P == 2:
            b = pickle.load(fp, encoding='iso-8859-1')
    # print(b)
    for i in range(T):
        if (i == 0):
            for j in range(robot_N):
                robot_history[str(j)] = copy.deepcopy(b[i][str(j)])
                robot_history[str(j)][0] = dict.fromkeys(key_set, dict())

                for k in range(len(key_set)):
                    robot_history[str(j)][0][key_set[k]] = copy.deepcopy(b[i][str(j)][key_set[k]])
                    del robot_history[str(j)][key_set[k]]
                # robot_history[str(j)][0]['pos_x'] = copy.deepcopy(b[i][str(j)]['pos_x'])
                # robot_history[str(j)][0]['pos_y'] = copy.deepcopy(b[i][str(j)]['pos_y'])
                # robot_history[str(j)][0]['orien'] = copy.deepcopy(b[i][str(j)]['orien'])

                # del robot_history[str(j)]['pos_x']
                # del robot_history[str(j)]['pos_y']
                # del robot_history[str(j)]['orien']

        else:
            for j in range(robot_N):
                robot_history[str(j)][i] = copy.deepcopy(b[i][str(j)])

    # print(b[0])
    # print(b[0][str(1)])

    # print(robot_history['0'])
    # print(robot_history['2'][1])

    t = np.arange(T)*0.05

    # a = pd.DataFrame(robot_history['2']).T
    # print('2')
    # print(a)
    a = pd.DataFrame(robot_history['0']).T
    b = pd.DataFrame(robot_history['1']).T
    c = pd.DataFrame(robot_history['2']).T
    d = pd.DataFrame(robot_history['3']).T
    # e = pd.DataFrame(robot_history['4']).T
    # f = pd.DataFrame(robot_history['5']).T
    # g = pd.DataFrame(robot_history['6']).T
    # h = pd.DataFrame(robot_history['7']).T
    print(a)
    # a = pd.DataFrame(robot_history['0']).T
    # print('0')
    # print(a)
    # print(a['pos_x'])

    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '.', color='#0072BD')
    # plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '.', color='#7E2F8E')
    # plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '.', color='#77AC30')
    # plt.plot(d['pos_x'][offset:], d['pos_y'][offset:], '.', color='#EDB120')
    # plt.plot(e['pos_x'][offset:], e['pos_y'][offset:], '.', color='#4DBEEE')
    # plt.plot(f['pos_x'][offset:], f['pos_y'][offset:], '.', color='#D95319')
    # plt.plot(a['pos_x'][offset], a['pos_y'][offset], '*', color='g', markersize=15)
    # plt.plot(a['pos_x'][T - 1], a['pos_y'][T - 1], 'X', color='r', markersize=10)
    # plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '-', color='#0072BD', alpha=0.2)
    # plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '-', color='#7E2F8E', alpha=0.2)
    # plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '-', color='#77AC30', alpha=0.2)
    # plt.plot(d['pos_x'][offset:], d['pos_y'][offset:], '-', color='#EDB120', alpha=0.2)
    # plt.plot(e['pos_x'][offset:], e['pos_y'][offset:], '-', color='#4DBEEE', alpha=0.2)
    # plt.plot(f['pos_x'][offset:], f['pos_y'][offset:], '-', color='#D95319', alpha=0.2)
    # plt.plot(b['pos_x'][offset], b['pos_y'][offset], '*', color='g', markersize=15)
    # plt.plot(c['pos_x'][offset], c['pos_y'][offset], '*', color='g', markersize=15)
    # plt.plot(d['pos_x'][offset], d['pos_y'][offset], '*', color='g', markersize=15)
    # plt.plot(e['pos_x'][offset], e['pos_y'][offset], '*', color='g', markersize=15)
    # plt.plot(f['pos_x'][offset], f['pos_y'][offset], '*', color='g', markersize=15)
    # plt.plot(b['pos_x'][T - 1], b['pos_y'][T - 1], 'X', color='r', markersize=10)
    # plt.plot(c['pos_x'][T - 1], c['pos_y'][T - 1], 'X', color='r', markersize=10)
    # plt.plot(d['pos_x'][T - 1], d['pos_y'][T - 1], 'X', color='r', markersize=10)
    # plt.plot(e['pos_x'][T - 1], e['pos_y'][T - 1], 'X', color='r', markersize=10)
    # plt.plot(f['pos_x'][T - 1], f['pos_y'][T - 1], 'X', color='r', markersize=10)
    # plt.legend(["Robot 0", "Robot 1", "Robot 2", "Robot 3", "Robot 4", "Robot 5", "Start", "Finish"])
    # plt.xlabel("x-coordinate [m]")
    # plt.ylabel("y-coordinate [m]")
    # # plt.xlim([0.5,1.5])
    # # plt.ylim([0.6,1.5])
    # plt.title('Odometry')
    #
    # # plt.figure(figsize = (8, 6), dpi = 80)
    # # plt.plot(a['x'], a['y'], '.')
    #
    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '.', color='#0072BD')
    # plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '.', color='#7E2F8E')
    # plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '.', color='#77AC30')
    # plt.plot(d['cam_x'][offset:], d['cam_y'][offset:], '.', color='#EDB120')
    # plt.plot(e['cam_x'][offset:], e['cam_y'][offset:], '.', color='#4DBEEE')
    # plt.plot(f['cam_x'][offset:], f['cam_y'][offset:], '.', color='#D95319')
    # plt.plot(g['cam_x'][offset:], g['cam_y'][offset:], '.', color='#FF00FF')
    # plt.plot(h['cam_x'][offset:], h['cam_y'][offset:], '.', color='#00FF00')
    # plt.plot(a['cam_x'][offset], a['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(a['cam_x'][T-1], a['cam_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '-', color='#0072BD', alpha=0.2)
    # plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '-', color='#7E2F8E', alpha=0.2)
    # plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '-', color='#77AC30', alpha=0.2)
    # plt.plot(d['cam_x'][offset:], d['cam_y'][offset:], '-', color='#EDB120', alpha=0.2)
    # plt.plot(e['cam_x'][offset:], e['cam_y'][offset:], '-', color='#4DBEEE', alpha=0.2)
    # plt.plot(f['cam_x'][offset:], f['cam_y'][offset:], '-', color='#D95319', alpha=0.2)
    # plt.plot(g['cam_x'][offset:], g['cam_y'][offset:], '-', color='#FF00FF', alpha=0.2)
    # plt.plot(h['cam_x'][offset:], h['cam_y'][offset:], '-', color='#00FF00', alpha=0.2)
    # plt.plot(b['cam_x'][offset], b['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(c['cam_x'][offset], c['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(d['cam_x'][offset], d['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(e['cam_x'][offset], e['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(f['cam_x'][offset], f['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(g['cam_x'][offset], g['cam_y'][offset], '*', color='g', markersize=15)
    # plt.plot(h['cam_x'][offset], h['cam_y'][offset], '*', color='g', markersize=15)
    # # plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], '*', color='r', markersize=10)
    # plt.plot(b['cam_x'][T-1], b['cam_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(c['cam_x'][T-1], c['cam_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(d['cam_x'][T-1], d['cam_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(e['cam_x'][T-1], e['cam_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(f['cam_x'][T-1], f['cam_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(g['cam_x'][T - 1], g['cam_y'][T - 1], 'X', color='r', markersize=10)
    # plt.plot(h['cam_x'][T - 1], h['cam_y'][T - 1], 'X', color='r', markersize=10)
    # plt.legend(["Robot 0", "Robot 1", "Robot 2",  "Robot 3", "Robot 4", "Robot 5", "Robot 6", "Robot 7", "Start", "Finish"], loc='upper right')
    # plt.xlabel("x-coordinate [m]")
    # plt.ylabel("y-coordinate [m]")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('OptiTrack measurements')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '.', color='#0072BD')
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '.', color='#7E2F8E')
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '.', color='#77AC30')
    plt.plot(d['estimation_x'][offset:], d['estimation_y'][offset:], '.', color='#EDB120')
    # plt.plot(e['estimation_x'][offset:], e['estimation_y'][offset:], '.', color='#4DBEEE')
    # plt.plot(f['estimation_x'][offset:], f['estimation_y'][offset:], '.', color='#D95319')
    # plt.plot(g['estimation_x'][offset:], g['estimation_y'][offset:], '.', color='#FF00FF')
    # plt.plot(h['estimation_x'][offset:], h['estimation_y'][offset:], '.', color='#00FF00')
    plt.plot(a['estimation_x'][offset], a['estimation_y'][offset], '*', color='g', markersize=15)
    plt.plot(a['estimation_x'][T-1], a['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '-', color='#0072BD', alpha=0.2)
    plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '-', color='#7E2F8E', alpha=0.2)
    plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '-', color='#77AC30', alpha=0.2)
    plt.plot(d['estimation_x'][offset:], d['estimation_y'][offset:], '-', color='#EDB120', alpha=0.2)
    # plt.plot(e['estimation_x'][offset:], e['estimation_y'][offset:], '-', color='#4DBEEE', alpha=0.2)
    # plt.plot(f['estimation_x'][offset:], f['estimation_y'][offset:], '-', color='#D95319', alpha=0.2)
    # plt.plot(g['estimation_x'][offset:], g['estimation_y'][offset:], '-', color='#FF00FF', alpha=0.2)
    # plt.plot(h['estimation_x'][offset:], h['estimation_y'][offset:], '-', color='#00FF00', alpha=0.2)
    plt.plot(b['estimation_x'][offset], b['estimation_y'][offset], '*', color='g', markersize=15)
    plt.plot(c['estimation_x'][offset], c['estimation_y'][offset], '*', color='g', markersize=15)
    plt.plot(d['estimation_x'][offset], d['estimation_y'][offset], '*', color='g', markersize=15)
    # plt.plot(e['estimation_x'][offset], e['estimation_y'][offset], '*', color='g', markersize=15)
    # plt.plot(f['estimation_x'][offset], f['estimation_y'][offset], '*', color='g', markersize=15)
    # plt.plot(g['estimation_x'][offset], g['estimation_y'][offset], '*', color='g', markersize=15)
    # plt.plot(h['estimation_x'][offset], h['estimation_y'][offset], '*', color='g', markersize=15)
    # plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], '*', color='r', markersize=10)
    plt.plot(b['estimation_x'][T-1], b['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(c['estimation_x'][T-1], c['estimation_y'][T-1], 'X', color='r', markersize=10)
    plt.plot(d['estimation_x'][T-1], d['estimation_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(e['estimation_x'][T-1], e['estimation_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(f['estimation_x'][T-1], f['estimation_y'][T-1], 'X', color='r', markersize=10)
    # plt.plot(g['estimation_x'][T - 1], g['estimation_y'][T - 1], 'X', color='r', markersize=10)
    # plt.plot(h['estimation_x'][T - 1], h['estimation_y'][T - 1], 'X', color='r', markersize=10)
    plt.legend(["Robot 0", "Robot 1", "Robot 2",  "Robot 3", "Start", "Finish"], loc='upper right')
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    #plt.xlim([0,2])
    #plt.ylim([0,2])
    #plt.title('Position estimation')
    plt.title('Trajectory of phase algorithm')

    plt.figure(figsize=(8, 6), dpi=80)
    plt.plot(1.226, 0.839, 'X', color='#0072BD', markersize=10)
    plt.plot(b['estimation_x'][T-1], b['estimation_y'][T-1], 'X', color='#7E2F8E', markersize=10)
    plt.plot(c['estimation_x'][T-1], c['estimation_y'][T-1], 'X', color='#77AC30', markersize=10)
    plt.plot(d['estimation_x'][T-1], d['estimation_y'][T-1], 'X', color='#EDB120', markersize=10)
    plt.plot(1.2, 0.55, 'X', color='#4DBEEE', markersize=10)
    #plt.plot(f['estimation_x'][T-1], f['estimation_y'][T-1], 'X', color='#D95319', markersize=10)
    plt.legend(["Robot 0", "Robot 1", "Robot 2",  "Robot 3", "Robot 4", "Robot 5"])
    plt.xlabel("x-coordinate [m]")
    plt.ylabel("y-coordinate [m]")
    plt.xlim([0.3,1.44])
    plt.ylim([0.3,1.44])
    plt.title('Actual final position')

    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '.')
    # plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(a['pos_x'][offset], a['pos_y'][offset], 'x', color='r', markersize=10)
    # plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], 'x', color='g', markersize=10)
    # plt.xlabel("time")
    # plt.ylabel("speed")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('Estimation')
    #
    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '.')
    # plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '.')
    # plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '.')
    # plt.plot(b['pos_x'][offset:], b['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(b['estimation_x'][offset:], b['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(b['cam_x'][offset:], b['cam_y'][offset:], '-k', alpha=0.2)
    # plt.plot(b['pos_x'][offset], b['pos_y'][offset], 'x', color='r', markersize=10)
    # plt.plot(b['pos_x'][T-1], b['pos_y'][T-1], 'x', color='g', markersize=10)
    # plt.legend(["odom", "cam", "estimation"])
    # plt.xlabel("x-coordinate [m]")
    # plt.ylabel("y-coordinate [m]")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('Merged robot 1')
    #
    #
    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '.')
    # plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '.')
    # plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '.')
    # plt.plot(a['pos_x'][offset:], a['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(a['estimation_x'][offset:], a['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(a['cam_x'][offset:], a['cam_y'][offset:], '-k', alpha=0.2)
    # plt.plot(a['pos_x'][offset], a['pos_y'][offset], 'x', color='r', markersize=10)
    # plt.plot(a['pos_x'][T-1], a['pos_y'][T-1], 'x', color='g', markersize=10)
    # plt.legend(["odom", "cam", "estimation"])
    # plt.xlabel("x-coordinate [m]")
    # plt.ylabel("y-coordinate [m]")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('Merged robot 0')
    #
    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '.')
    # plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '.')
    # plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '.')
    # plt.plot(c['pos_x'][offset:], c['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(c['estimation_x'][offset:], c['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(c['cam_x'][offset:], c['cam_y'][offset:], '-k', alpha=0.2)
    # plt.plot(c['pos_x'][offset], c['pos_y'][offset], 'x', color='r', markersize=10)
    # plt.plot(c['pos_x'][T-1], c['pos_y'][T-1], 'x', color='g', markersize=10)
    # plt.legend(["odom", "cam", "estimation"])
    # plt.xlabel("x-coordinate [m]")
    # plt.ylabel("y-coordinate [m]")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('Merged robot 2')
    #
    # plt.figure(figsize=(8, 6), dpi=80)
    # plt.plot(t[offset-2:], b['pos_y'][offset:], '.')
    # plt.plot(t[offset-2:], b['cam_y'][offset:], '.')
    # plt.plot(t[offset-2:], b['estimation_y'][offset:], '.')
    # plt.plot(t[offset-2:], b['pos_y'][offset:], '-k', alpha=0.2)
    # plt.plot(t[offset-2:], b['estimation_y'][offset:], '-k', alpha=0.2)
    # plt.plot(t[offset-2:], b['cam_y'][offset:], '-k', alpha=0.2)
    # plt.legend(["odom", "cam", "estimation"])
    # plt.xlabel("time")
    # plt.ylabel("y-coordinate [m]")
    # #plt.xlim([0,2])
    # #plt.ylim([0,2])
    # plt.title('Time plot robot 1')
    #
    #
    # # plt.figure(figsize=(8, 6), dpi=80)
    # # plt.plot(t, c['cam_x'][2:], '.')
    # # plt.plot(t, c['cam_x'][2:], '-k', alpha=0.2)
    # # plt.xlabel("time")
    # # plt.ylabel("x-coordinate [m]")
    # # #plt.xlim([0,2])
    # # #plt.ylim([0,2])
    # # plt.title('Camera - x position')
    # #
    # # plt.figure(figsize=(8, 6), dpi=80)
    # # plt.plot(t, c['cam_y'][2:], '.')
    # # plt.plot(t, c['cam_y'][2:], '-k', alpha=0.2)
    # # plt.xlabel("time")
    # # plt.ylabel("y-coordinate [m]")
    # # #plt.xlim([0,2])
    # # #plt.ylim([0,2])
    # # plt.title('Camera - y position')
    #
    plt.show()
    #
    # # plt.figure(figsize = (8, 6), dpi = 80)
    # # plt.plot(a['estimation_x'], a['estimation_y'], '.')
    # # plt.title('estimation')
    #
    #
    #
    #
    #
