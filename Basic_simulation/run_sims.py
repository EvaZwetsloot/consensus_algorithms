#Author: Eva Zwetsloot
#Python script for running all algorithms and saving the data

import matplotlib.pyplot as plt
import numpy as np
import random
import math
import time
import PySimpleGUI as sg
import networkx as nx

import ETC_seyboth.seyboth_delay
import settings
import draw_sims
import os.path
from os import path

import DA.DA_phase as phase
import ETC_seyboth.seyboth_delay as ETC
import PETC.petc as PETC
import DA.DA_Tarry as tarry

def ASSERT_DRV(err):
    if isinstance(err, cuda.CUresult):
        if err != cuda.CUresult.CUDA_SUCCESS:
            raise RuntimeError("Cuda Error: {}".format(err))
    elif isinstance(err, nvrtc.nvrtcResult):
        if err != nvrtc.nvrtcResult.NVRTC_SUCCESS:
            raise RuntimeError("Nvrtc Error: {}".format(err))
    else:
        raise RuntimeError("Unknown error type: {}".format(err))

def initialize():
    global N, Ts, area, D, dis, K, final_time
    Ts = settings.Ts
    area = settings.area
    D = settings.D
    K = settings.K
    final_time = settings.final_time

def save_data(N, dis, delay, error, stop_time, nb_com, net, filename, conv_rate, max_delay, u_max, L, radius, delta, alpha, c0, c1, h, sigma, u_petc):

    file = open(filename, "a+") #w+ is to create and overwrite
    file.write('%i' % N + ', ' + '%i' % dis + ', ' + '%f' % alpha + ', ' + '%f' % c0 + ', ' + '%f' % c1 + ', ' + '%f' % h + ', ' + '%f' % sigma + ', ' + '%f' % delay + ', ' + '%f' % error + ', ' + '%i' % radius + ', ' + '%f' % np.mean(u_max) + ', ' + '%f' % np.std(u_max) + ', '+ '%f' % np.mean(u_petc) + ', ' + '%f' % np.std(u_petc) + ', ')
    for i in np.arange(4):
        file.write('%f' % np.mean(stop_time[:,i]) + ', ' + '%f' % np.std(stop_time[:,i]) +', ' )
    for i in np.arange(4):
        file.write('%f' % np.mean(nb_com[:,i]) + ', ' + '%f' % np.std(nb_com[:,i]) +', ' )
    file.write('%f' % conv_rate + ', ' + '%f' % max_delay + ', ')
    for i in np.arange(N):
        file.write('%f' % np.mean(delta[:, i]) + ', ' + '%f' % np.std(delta[:, i]) + ', ')
    file.write('\n')
    file.close()


def collect_data(network, delay, error, N, imagename, filename, conv_rate, max_delay, radius, alpha, c0, c1, h, sigma):
    L, N, dis = settings.select_network(network, N)
    nb_positions = 40
    u_max = np.zeros(nb_positions)
    u_petc = np.zeros(nb_positions)
    stop_time = np.zeros([nb_positions, 4])
    nb_com = np.zeros([nb_positions, 4])
    delta = np.zeros([nb_positions, N])

    #Run ETC algorithm
    for n in np.arange(0,nb_positions, dtype=int):
        _, _, _,_,_ = settings.create_network(N, L, network, imagename, False, radius) #laplacian based positions
        #x0 = settings.create_initial_pos(N, area, D, L, radius) #random based positions

        x_etc, t_events, stop_time_etc, t_hat_time, events_etc, x_hat_next, u_max2, events_etc2 = ETC.run_simulation(N, area, D, final_time, Ts, delay, L, x0, error, alpha, c0, c1)
        #Run PETC algorithm
        x_petc, t_petc, stop_time_petc, events_petc, x0, u_petc2, events_petc2 = PETC.run_simulation(N, area, D, final_time, h, sigma, delay, L, x0, error)

        #Run phase algorithm
        x_phase, t_phase, nb_com_phase, init_time_phase, events_phase, stop_time_phase = phase.run_simulation(N, area, D, final_time, Ts, delay, L, x0, dis, error, u_max2)

        #Run tarry's algorithm
        x_tarry, t_tarry, nb_com_tarry, init_time_tarry, init_agent, events_tarry, stop_time_tarry = tarry.run_simulation(N, area, D, final_time, Ts, delay, L, x0, dis, error, u_max2)

        #save data to file
        u_max[n] = u_max2
        u_petc[n] = u_petc2
        stop_time[n,:] = [np.amax(t_tarry), np.amax(t_phase), stop_time_etc, stop_time_petc]
        nb_com[n,:] = [np.sum(np.count_nonzero(events_tarry[:,0,:], axis=1)), np.sum(np.count_nonzero(events_phase[:,0,:], axis=1)), np.sum(events_etc), np.sum(events_petc)]
        delta[n,:] = np.sort(np.linalg.norm(x0-np.mean(x0,0), axis=1))

    #draw_sims.draw_etc(x_etc, t_events, events_etc, events_etc2, t_hat_time, stop_time_etc, N, "ETC", delay, imagename, x0)
    #draw_sims.draw_petc(x_petc[:,:, :int(stop_time_petc/h)], np.arange(0, stop_time_petc, h),events_petc, events_petc2, t_hat_time, stop_time_petc, N, "PETC", delay, imagename, x0)
    #draw_sims.draw_phase(x_phase, t_phase, dis, stop_time_phase, N, "phase", imagename, x0, events_phase)
    save_data(N, dis, delay, error, stop_time, nb_com, network, filename, conv_rate, max_delay, u_max, L, radius, delta, alpha, c0, c1, h, sigma, u_petc)


def main():
    plt.close("all")
    network = ["Cube"]
    delay_range = [0.02, 0.04, 0.06, 0.08, 0.1, 0.12, 0.14, 0.16, 0.18]
    h = 0.002
    error = 0.1
    radius = 1

    #Select number of nodes [n=0 selects HC, and n=1 selects line]
    stop = [7, 13]
    start = [2, 4]
    step = [1, 2]
    n = 0

    # Convergence rate of HC topology
    w = [2.08517042,  2.18244302,  2.29530433,  2.42896241,  2.59171102,  2.79800971, 1.70641076,  0.90637328, 0.34303899]
    p = 0

    initialize()
    d = 0

    #Triggering parameters
    c1 = 0.25
    c0 = 0.0003
    alpha_per = 0.5
    sigma = 0.04

    for delay in delay_range:

        for net in network:
            nodes = np.arange(start[n], stop[n], step[n], dtype=int)
            imagepath = "Results/IMAGE_data/comm_delay_" + delay_name[d]
            filename = "Results/RAW_data/network_" + net + ".txt"
            imagename = "Results/IMAGE_data/comm_delay_" + delay_name[d] + "/network_" + net

            if not os.path.exists(imagepath):
                os.makedirs(imagepath)

            if not path.exists(filename):
                file = open(filename, "w+")
                file.write("# nodes, diameter, alpha, c0, c1, h, sigma, delay, error, r, u_max, T stop, P stop, E stop, PE stop, T events, P events, E events, Conv_rate, Max_delay, The rest \n")
                file.close()

            for i in nodes:
                N = i
                L, N_test, dis = settings.select_network(net, N)

                eig, _ = np.linalg.eig(L)

                max_delay = math.pi/(2*sorted(eig)[-1])
                conv_rate = sorted(eig)[1]
                alpha = w[p]*alpha_per
                if delay < max_delay and N_test < 257:
                    collect_data(net, delay, error, N, imagename, filename, conv_rate, max_delay, radius, alpha, c0, c1, h, sigma)
                else:
                    print("This is too large of a delay for ETC: " + net + " nodes =  %i" %N + "\n")
                    print("Max delay = %f" % max_delay)

                # Plot network graph
                settings.create_network(N_test, L, net, imagename, True, radius)

                #p += 1
        print("Finished " + net)
        d+= 1

    print("Finished sim")


if __name__ == "__main__":
    main()