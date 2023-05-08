#Author: Eva Zwetsloot
#Python script for running the petc algorithm to solve consensus

import matplotlib.pyplot as plt
import numpy as np
import random
import math
import time
import types
import functools

from scipy import integrate

# Two-dimensional plane
D = 2

def create_initial_pos(N, area, D, L):
    random.seed(24) #10
    p = np.zeros((N, D))
    for x in np.arange(N):
        for n in np.arange(D):
            p[x, n] = round(random.uniform(0, area[n]), 2)
            n = n + 1
        x = x + 1

    return p


def trigger(x, x_hat, x_hat2, L, N, id, sigma, n):
    #additional small error to ensure f > 0
    e = 0.00000000000001
    out = np.dot(L[id[n,0]], x_hat)

    f = np.linalg.norm(x[n,:] - x_hat2[n, :].T) - sigma*np.amax(np.absolute(out))

    return f > 0


def f(t, x, x_hat, x_hat2, L, N, id):
    global u_max
    i = 0
    j = 0
    beta = 1
    out = np.zeros(D*N)
    for i in range(D):
        out[j:j+N] = np.dot(-beta*L, (x_hat[j:j+N]))
        j += N

    if max(abs(out)) > u_max:
        u_max = max(abs(out))

    return out

def run_simulation(N, area, D, end_time, Ts, sigma, delay, L, x0, error):
    triggers = np.zeros([N, end_time + 100])
    triggers[:,0] = np.arange(1,N+1)
    x = np.zeros((N,D,end_time))
    x_hat = x0
    x_hat2 = np.zeros((N,D,end_time))
    t_triggers = np.zeros(end_time)
    n_delay = math.ceil(delay/Ts)
    x[:,:,0:n_delay+1] = np.repeat(x0[:,:, np.newaxis], n_delay+1, axis=2)
    x_hat2[:, :, 0:n_delay + 1] = np.repeat(x0[:, :, np.newaxis], n_delay + 1, axis=2)
    final = 1/N*np.sum(x0, axis=0)
    t = n_delay
    u_max = 0

    ind = np.zeros((N,D), dtype=int)
    #Get all id's
    for i in range(N):
        for n in range(D):
            ind[i, n] = int(i+n*N)

    while (t<(end_time-n_delay)):

        for i in np.arange(N):
            if trigger(x[:, :, t], x_hat[:, :], x_hat2[:, :, t], L, N, ind, sigma, i):
                x_hat2[i, :, t+1] = x[i, :, t]
                triggers[i, t] = i+1
                t_triggers[t+n_delay] = 1
            else:
                x_hat2[i, :, t+1] = x_hat2[i, :, t]

        if (t_triggers[t]):
            x_hat[:,:] = x_hat2[:,:,t-n_delay]

        x[:,:,t+1] = x[:,:,t] + Ts*np.matmul(-L, x_hat[:,:])

        if np.amax(abs(np.matmul(-L, x_hat[:,:]))) > u_max:
            u_max = np.amax(abs(np.matmul(-L, x_hat[:,:])))

        if(not (np.linalg.norm(final-x[:,:,t+1]) > error).any()):
            end_time = t
            # terminate if close enough to final state
            break

        if (t*Ts > 40):
            end_time = t
            print("PETC simulation too long")
            break

        t= t+1

    return x, triggers, end_time*Ts, np.count_nonzero(triggers, axis=1), x[:,:,0], u_max, triggers
