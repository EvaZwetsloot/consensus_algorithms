#Author: Eva Zwetsloot
#Visualize simulations

import matplotlib.pyplot as plt
import numpy as np
import math


def draw(x, t, triggers, Ts, final_time, N, name, imagename):

    M = math.ceil(max(final_time))
    F = final_time.astype(int)
    events = np.count_nonzero(triggers[:,0,:], axis=1)
    final = 1 / N * np.sum(x[:, :, 0], axis=0)

    fig, axs = plt.subplots(3)
    fig.suptitle('Simulation of %s' % name)

    for n in range(N):
        axs[0].plot(t[n, 0:F[n]], x[n, 0, 0:F[n]], label="Agent %i" % n + ": %i" % events[n])
    axs[0].axhline(final[0])
    axs[0].set(xlabel='Time [sec]', ylabel='Trajectory of p1')
    axs[0].set_title('Convergence')
    axs[0].legend(bbox_to_anchor=(1.04, 1), borderaxespad=0)


    #fig2 = plt.figure()
    for n in range(N):
        axs[1].plot(t[n, 0:F[n]], x[n, 0, 0:F[n]])
    axs[1].axhline(final[0])
    axs[1].set(xlabel='Time [sec]', ylabel='Trajectory of p2')
    axs[1].set_title('Convergence')

    #fig3 = plt.figure()
    for n in range(N):
        axs[2].plot(x[n, 0, 0:F[n]], x[n, 1, 0:F[n]])
    axs[2].plot(final[0], final[1], 'x')
    axs[2].set(xlabel='x coordinate', ylabel='y coordinate')
    axs[2].set_title('Trajectory')
    axs[2].set_ylim([0, 10])
    axs[2].set_xlim([0, 10])

    plt.subplots_adjust(hspace=1)
    plt.subplots_adjust(right=0.75)

    num = str(N)
    plt.savefig(imagename + "_" + num + "_" + name + '.png')


def draw_etc(x, t_events, count_events, count_events2, t_hat_time, end_time, N, name, delay, imagename, x0):
    final = 1 / N * np.sum(x0, axis=0)

    for i in range(N):
        count_events2[i,0,0] = i+1

    fig, axs = plt.subplots(4)
    fig.suptitle('Simulation of %s' % name)

    for n in range(N):
        axs[0].plot(t_events, x[n, 0, :], label="Agent %i: " % n + "%i" % int(count_events[n]+1))
    axs[0].axhline(final[0])
    axs[0].scatter(end_time, final[0], marker='x')
    axs[0].set(xlabel='Time [sec]', ylabel='x')
    axs[0].set_title('Convergence')
    axs[0].legend(bbox_to_anchor=(1.04, 1), borderaxespad=0)


    for n in range(N, N+N): # change to range(N, N+N) for ETC and x[n,0,:]
        axs[1].plot(t_events, x[n, 0, :])
    axs[1].axhline(final[1])
    axs[1].scatter(end_time, final[1], marker='x')
    axs[1].set(xlabel='Time [sec]', ylabel='y')
    axs[1].set_title('Convergence')

    for n in range(N):  # change for ETC to x[n+N,0,:] instead of x[n,1,:]
        axs[3].scatter(count_events2[n,1,:], count_events2[n,0,:])
    axs[3].set(xlabel='Time [sec]', ylabel='Agent ID')
    axs[3].set_title('#Events')
    axs[3].set_ylim([0.5,N+1])
    #axs[3].set_xlim([0, N])

    for n in range(N):  # change for ETC to x[n+N,0,:] instead of x[n,1,:]
        axs[2].plot(x[n, 0, :], x[n+N,0,:])
    axs[2].plot(final[0], final[1], 'x')
    axs[2].set(xlabel='x coordinate', ylabel='y coordinate')
    axs[2].set_title('Trajectory')
    axs[2].set_ylim([0,2])
    axs[2].set_xlim([0, 2])



    plt.subplots_adjust(hspace=1)
    plt.subplots_adjust(right=0.75)

    num = str(N)
    plt.savefig(imagename + "_" + num + "_" + "ETC" + '.png')

    #plt.close("all")




def draw_petc(x, t_events, count_events, count_events2, t_hat_time, end_time, N, name, delay, imagename, x0):
    final = 1 / N * np.sum(x0, axis=0)
    end_number = t_events.shape[0]

    fig, axs = plt.subplots(4)
    fig.suptitle('Simulation of %s' % name)

    for n in range(N):
        axs[0].plot(t_events, x[n, 0, :], label="Agent %i: " % n + "%i" % int(count_events[n]))
    axs[0].axhline(final[0])
    axs[0].scatter(end_time, final[0], marker='x')
    axs[0].set(xlabel='Time [sec]', ylabel='x')
    axs[0].set_title('Convergence')
    axs[0].legend(bbox_to_anchor=(1.04, 1), borderaxespad=0)


    for n in range(N): # change to range(N, N+N) for ETC and x[n,0,:]
        axs[1].plot(t_events, x[n, 1, :])
    axs[1].axhline(final[1])
    axs[1].scatter(end_time, final[1], marker='x')
    axs[1].set(xlabel='Time [sec]', ylabel='y')
    axs[1].set_title('Convergence')

    for n in range(N):  # change for ETC to x[n+N,0,:] instead of x[n,1,:]
        axs[3].scatter(t_events, count_events2[n,:end_number])
    axs[3].set(xlabel='Time [sec]', ylabel='Agent ID')
    axs[3].set_title('#Events')
    axs[3].set_ylim([0.5,N+1])
    #axs[3].set_xlim([0, N])

    for n in range(N):  # change for ETC to x[n+N,0,:] instead of x[n,1,:]
        axs[2].plot(x[n, 0, :], x[n,1,:])
    axs[2].plot(final[0], final[1], 'x')
    axs[2].set(xlabel='x coordinate', ylabel='y coordinate')
    axs[2].set_title('Trajectory')
    axs[2].set_ylim([0,2])
    axs[2].set_xlim([0, 2])


    plt.subplots_adjust(hspace=1)
    plt.subplots_adjust(right=0.75)

    num = str(N)
    plt.savefig(imagename + "_" + num + "_" + "PETC" + '.png')


def draw_phase(x, t_events, dis, end_time, N, name, imagename, x0, triggers):
    final = 1 / N * np.sum(x0, axis=0)

    end_number = int(np.amax(end_time)-1)
    end_id = np.where(t_events[:,end_number] > 0)
    for i in range(N):
        x[i,0,int(end_time[i]):]  = np.ones(int(len(x[i,0,:])-end_time[i]))*x[i,0,int(end_time[i]-1)]
        x[i, 1, int(end_time[i]):] = np.ones(int(len(x[i, 0, :]) - end_time[i])) * x[i, 1, int(end_time[i] - 1)]

    fig, axs = plt.subplots(4)
    fig.suptitle('Simulation of %s' % name)

    for n in range(N):
        axs[0].plot(np.squeeze(t_events[end_id,:end_number]), x[n, 0, :end_number], label="Agent %i: " % n + "%i" % int(dis))
    axs[0].axhline(final[0])
    axs[0].scatter(t_events[end_id[0][0], int(end_number)], final[0], marker='x')
    axs[0].set(xlabel='Time [sec]', ylabel='x')
    axs[0].set_title('Convergence')
    axs[0].legend(bbox_to_anchor=(1.04, 1), borderaxespad=0)


    for n in range(N):
        axs[1].plot(np.squeeze(t_events[end_id,:end_number]), x[n, 1, :end_number])
    axs[1].axhline(final[1])
    axs[1].scatter(t_events[end_id[0][0], int(end_number)], final[1], marker='x')
    axs[1].set(xlabel='Time [sec]', ylabel='y')
    axs[1].set_title('Convergence')

    # Note the x-axis is tailored to a HC with 4 robots -- change in future
    for n in range(N):
        axs[3].scatter(np.array([0.0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35]), triggers[n,0,:8])
    axs[3].set(xlabel='Time [sec]', ylabel='Agent ID')
    axs[3].set_title('#Events')
    axs[3].set_ylim([0.5,N+1])
    axs[3].set_xlim([0, t_events[end_id,end_number]])
    #axs[3].set_xlim([0, N])

    for n in range(N):  # change for ETC to x[n+N,0,:] instead of x[n,1,:]
        axs[2].plot(x[n, 0, :end_number], x[n,1,:end_number])
    axs[2].plot(final[0], final[1], 'x')
    axs[2].set(xlabel='x coordinate', ylabel='y coordinate')
    axs[2].set_title('Trajectory')
    axs[2].set_ylim([0,2])
    axs[2].set_xlim([0, 2])


    plt.subplots_adjust(hspace=1)
    plt.subplots_adjust(right=0.75)

    num = str(N)
    plt.savefig(imagename + "_" + num + "_" + "phase" + '.png')