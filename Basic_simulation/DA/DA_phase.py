#Author: Eva Zwetsloot
#Python script for running the phase algorithm for consensus

import matplotlib.pyplot as plt
import numpy as np
import random
import math
from dataclasses import dataclass
import threading
import queue
import concurrent.futures
from pyvis.network import Network
import networkx as nx
import settings
import functools

from scipy import integrate

# Global parameters
Ts = settings.Ts
area = settings.area
D = settings.D
K = settings.K
final_time = settings.final_time


class Queue:

    # To initialize the object.
    def __init__(self):

        self.queue = []
        self.data_id = []
        self.front = self.rear = 0

    # Function to insert an element
    # at the rear of the queue
    def Enqueue(self, data, agent):
        # Insert element at the rear
            self.queue.append(data)
            self.data_id.append([agent])
            self.rear += 1

    # Function to delete an element
    # from the front of the queue
    def Dequeue(self, agent):

        # Pop the front element from list
        idx = self.data_id.index(agent)
        x = self.queue.pop(idx)
        self.data_id.pop(idx)
        self.rear -= 1

        return x

    def check_queue(self, agent):
        if len(self.queue) > 0:
            if agent in self.data_id:
                return True
            else:
                return False

    # Function to print queue elements
    def Display(self):

        if (self.front == self.rear):
            print("\nQueue is Empty")

        # Traverse front to rear to
        # print elements
        for i in self.queue:
            print(i, "<--", end='')

    # Print front of queue
    def queueFront(self):

        if (self.front == self.rear):
            print("\nQueue is Empty")

        print("\nFront Element is:",
              self.queue[self.front])


def control(t, x, K, u_max, x_final, error):
    u = -K * (x - x_final)
    test = False
    if np.any(u > u_max):
        u[np.where(u > u_max)] = u_max
        test = True

    if np.any(u < -u_max):
        u[np.where(u < -u_max)] = -u_max
        test = True

    if test == False:
        test = False

    dx = u

    return dx

def zero_crossing(t, x, K, u_max, x_final, error):
    return max(np.absolute(np.subtract(x, x_final))) - error

class Agent:

    def __init__(self, i, x0, N):
        self.id = i
        self.x0 = x0
        self.data = np.zeros([D,N])
        self.data[:,i] = x0
        self.sum = np.zeros([D,1])
        self.rec = []
        self.com = 0
        self.neigh = []

    def send_data(self, queues, k, triggers, m):
        if (min(self.rec) >= self.com and max(self.rec) > 0):

            for i in self.neigh:
                queues[i].Enqueue(self.data, self.id)

            self.com += 1
            triggers[self.id, 0, k] = self.id+1

            m += 1

        return triggers, m

    def poll_data(self, queues, k, N, triggers):
        for i in self.neigh:
            if queues[self.id].check_queue(i):
                data = queues[self.id].Dequeue(i)
                self.save_data(data, N)

                self.rec[np.where(self.neigh == i)] += 1
                triggers[self.id, 1, k] = self.id+1

        return triggers

    def check_received(self, dis):
        out = True
        for i in range(len(self.neigh)):
            if (self.rec[i] < dis):
                out = False

        return out

    def controller(self, error, u_max, t_start, x, t):

        x_final = np.mean(self.data, axis=1)

        zero_crossing.terminal = True
        zero_crossing.direction = -1
        if max(np.absolute(np.subtract(self.x0, x_final))) - error > 0:
            data = integrate.solve_ivp(control, [t_start, final_time], self.x0, events=zero_crossing, args=[K, u_max, x_final, error], max_step=Ts)
            t_time = np.append(t, data.t[:])
            x = np.append(x, data.y[:,:], axis= 1)
        else:
            t_time = t


        return x, t_time

    def save_data(self, data, N):
        for i in range(N):
            if not self.data[:,i].any():
                self.data[:,i] = data[:,i]


plt.close('all')


def create_network(N, L):
    E = np.asarray(np.where(np.tril(L)==-1))
    num_E = E.shape[1]

    E_full = np.asarray(np.where(L==-1))

    #visualize network
    G = nx.Graph()
    G.add_nodes_from(np.arange(N))
    G.add_edges_from(list(map(tuple, E.T)))
    # nx.draw(G, with_labels=True)

    return num_E, E, E_full


def find_neighbours(i,E):

    id = np.asarray(np.where(E[0,:]==i))
    neighbours = E[1,id].reshape(id.shape[1])

    return neighbours

def compute_nb_comm(robots, N):
    sum = 0
    for i in range(N):
        sum += robots[i].com

    return sum


def phase(queues, robots, delay, N, triggers, dis):
    init_agents = 1
    start = random.sample(range(0,N), init_agents)
    nb_robots_finished = []
    dia = np.zeros(N)
    m = 0
    k = 0

    for i in start:
        robots[i].rec = np.zeros(len(robots[i].neigh)) + 1
        robots[i].com += 1
        triggers, _ = robots[i].send_data(queues, k, triggers, m)


    time = max(delay, Ts)

    for i in range(N):
        if i in start:
            dia[i] = dis + 1
        else:
            dia[i] = dis

    while len(nb_robots_finished) < N:
        for i in range(N):
            if k % 2 == 0:
                triggers, m = robots[i].send_data(queues, k, triggers, m)
            else:
                triggers = robots[i].poll_data(queues, k, N, triggers)

            if (robots[i].check_received(dia[i])):
                if i not in nb_robots_finished:
                    nb_robots_finished.append([i])

        if m > 0:
            time += max(delay, Ts)
            m = 0
        k += 1

    for i in start:
        robots[i].com -= 1

    return time, triggers

def run_simulation(N, area, D, end_time, Ts, delay, L, x0, diameter, error, u_max):
    # create network
    triggers = np.zeros([N, 3, end_time + 100])
    x = np.zeros((N, D, end_time))
    x0_save = x0
    e, E, E_full = create_network(N, L)
    stop_time = np.zeros(N)

    # create agents
    robots = [Agent(i, x0[i,:], N) for i in range(N)]

    #create queues
    queues = [Queue() for j in range(N)]

    # assign neighbours/queues to each robot
    for i in range(N):
        robots[i].neigh = find_neighbours(i, E_full)
        robots[i].rec = np.zeros(robots[i].neigh.shape)

    #first round of communications
    time_start, triggers = phase(queues, robots, delay, N, triggers, diameter)
    start_time = np.repeat(time_start/Ts, N)
    stop_time = np.zeros(N)

    x = np.zeros((N, D, end_time + int(start_time[0])))
    t_final = np.zeros((N, end_time + int(start_time[0])))

    # run trajectory
    for i in range(N):
        x_final, time_final = robots[i].controller(error, u_max, time_start, np.repeat(x0[i,:, np.newaxis], int(start_time[0]), axis=1), np.arange(0, time_start, Ts))

        x[i,:, 0:x_final.shape[1]] = x_final
        t_final[i,0:time_final.shape[0]] = time_final
        stop_time[i] = x_final.shape[1]

    total_comm = compute_nb_comm(robots, N)

    return x, t_final, total_comm, time_start, triggers, stop_time
