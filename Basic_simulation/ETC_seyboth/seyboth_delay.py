#Author: Eva Zwetsloot
#Python script for running the etc algorithm to solve consensus

import matplotlib.pyplot as plt
import numpy as np
import random
import math
import time
import run_sims
import settings
import types
import functools

from scipy import integrate


Ts = settings.Ts
area = settings.area
D = 2
K = settings.K
final_time = settings.final_time
u_max = 0


def create_initial_pos(N, area, D, L):
    random.seed(24) #10
    p = np.zeros((N, D))
    for x in np.arange(N):
        for n in np.arange(D):
            p[x, n] = round(random.uniform(0, area[n]), 2)
            n = n + 1
        x = x + 1

    return p


def trigger(t, x, x_hat, x_hat2, L, N, id, alpha, c0, c1, n):
    # additional error to ensure f > 0
    e = 0.00000000000001
    f = 0
    f = np.linalg.norm(x_hat2[id[n, :]] - x[id[n, :]].T) - (c0 + c1 * math.exp(-alpha * t)) - e

    return f


def f(t, x, x_hat_test, x_hat2, L, N, id, alpha, c0, c1):  # the flow, x’ = f(x)
    #global u_max
    i = 0
    j = 0
    beta = 1
    out = np.zeros(D*N)
    for i in range(D):
        out[j:j+N] = np.dot(-L, (x_hat_test[j:j+N]))
        j += N

    return out


def copy_trigger(f, i):
    g = functools.partial(trigger, n=i)
    g = functools.update_wrapper(g, f)
    g.__kwdefaults__ = f.__kwdefaults__
    return g

def find_id(data, x, x_hat, t, N, id, alpha, c0, c1):
    e = 0.000000000001

    f = -1*np.ones(N)
    for i in range(N):
        if len(data.t_events[i]) > 0:
            x_test = data.y_events[i].reshape(-1)

            for i in range(N):
                f[i] = max([np.linalg.norm(x_hat[id[i,:]] - x_test[id[i,:]].T) - (c0 + c1 * math.exp(-alpha * t)), np.linalg.norm(x_hat[id[i,:]] - x_test[id[i,:]].T) - (c0 + c1 * math.exp(-alpha * t)), f[i]])

    return np.where(f > -e)[0].tolist()


def run_simulation(N, area, D, end_time, Ts, delay, L, x0, error, alpha, c0, c1):
    # Set trigger functions
    trigger.terminal = True
    trigger.direction = 1.0

    #Set x variables
    x_hat = x0
    x_hat2 = x0
    x_last = x0
    u_max = 0
    x_hat_next= x0.reshape((D*N,1), order='F')
    x = np.zeros((N*D,1,1))
    x[:,:,0] = x0.reshape((N*D,1),  order='F')

    t_last = final_time
    t_now = delay
    t_events = np.zeros(1)
    t_trigger = np.asarray([[final_time],[0]]).reshape([2,1])
    triggers = []
    triggers2 = np.zeros((N,2)).reshape((N,2,1), order='F')
    k = 0

    t_hat_time = [0.0]

    #Register the number of events
    count_events = np.zeros(N)

    #Easy fix to make function terminate
    final = 1/N*np.sum(x0, axis=0)
    final = final.reshape(-1, order='F')

    ind = np.zeros((N,D), dtype=int)
    #Get all id's
    for i in range(N):
        for n in range(D):
            ind[i, n] = int(i+n*N)
        triggers = np.append(triggers, copy_trigger(trigger,i))

    while True:
        x_hat = x_hat.reshape(-1, order='F')
        x_hat2 = x_hat2.reshape(-1, order='F')
        x_last = x_last.reshape(-1, order='F')
        data = integrate.solve_ivp(f, [t_now, t_last], x_last, events=triggers, args=[x_hat, x_hat2, L, N, ind, alpha, c0, c1])
        t_events = np.append(t_events, data.t[1:])
        x = np.append(x, np.swapaxes(data.y[:,1:, np.newaxis], 1, 2), axis=2)
        t_now = data.t[-1]
        x_last = x[:,:,-1]

        # Compute maximum control input used
        out = np.zeros(D * N)
        j =0
        for i in range(D):
            out[j:j + N] = np.dot(-L, (x_hat2[j:j + N]))
            j += N

        if max(abs(out)) > u_max:
            u_max = max(abs(out))

        #Find which one triggered
        trigger_id = find_id(data, x_last, x_hat2, t_now, N, ind, alpha, c0, c1)

        #Assign values to x_hat
        if len(trigger_id) > 0:
            k += 1
            t_trigger = np.append(t_trigger, np.zeros(2).reshape((2,1)), axis=1)
            x_hat_next = np.append(x_hat_next, np.zeros(D*N).reshape((D*N,1)), axis=1)
            t_hat_time = np.append(t_hat_time, [0.0])
            t_trigger[0, -1] = t_now + delay
            t_trigger[1, -1] = k
            triggers2 = np.append(triggers2, np.zeros((N,2)).reshape((N,2,1)), axis=2)
            t_hat_time[k] = t_now

            #sort array based on time
            id_sort = np.argsort(t_trigger, axis=1)
            t_trigger = t_trigger[:,id_sort[0,:]]

            for i in np.arange(N):
                if i not in trigger_id:
                    x_hat_next[ind[i,:], k] = x_hat_next[ind[i,:], k-1].reshape(-1)

                else:
                    count_events[i] += 1
                    triggers2[i, 0, k] = i+1
                    triggers2[i, 1, k] = t_now
                    x_hat_next[ind[i,:], k] = x_last[ind[i,:]].reshape(-1)

        if t_now >= t_trigger[0,0] :
            x_hat = x_hat_next[:,int(t_trigger[1,0])]
            t_trigger = t_trigger[:,1:]
            t_last = t_trigger[0,0]
        else:
            t_last = t_trigger[0,0]

        x_hat2 = x_hat_next[:,-1]

        if not (np.absolute(np.subtract(x_last.T, np.repeat(final, N))) > error).any():
            end_time = t_now
            break

        if t_now > 40 or k > 100000:
            end_time = t_now
            print("ETC simulation too long, N = %i" % N + " t_now = %f" % t_now)
            break

    return x, t_events, end_time, t_hat_time, count_events, x_hat_next, u_max, triggers2