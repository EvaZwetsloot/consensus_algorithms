#Author: Eva Zwetsloot
#Python script for running Tarry's algorithm for consensus

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
import run_sims
import settings
import functools

from scipy import integrate


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

    # Check if there is something in the queue
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

    def clear_queue(self):
        self.queue = []
        self.data_id = []
        self.front = self.rear = 0

def control(t, x, K, u_max, x_final, error):
    u = -K * (x - x_final)
    if np.any(u > u_max):
        u[np.where(u > u_max)] = u_max

    if np.any(u < -u_max):
        u[np.where(u < -u_max)] = -u_max

    dx = u

    return dx

def zero_crossing(t, x, K, u_max, x_final, error):

    return max(np.absolute(np.subtract(x, x_final))) - error


class Agent:

    def __init__(self, i, x0):
        self.id = i
        self.x0 = x0
        self.data = x0
        self.sum = np.zeros([1, D])
        self.father = []
        self.com = []
        self.rec = []
        self.neigh = []

    # Send data to next neighbour or to father node if all neighbours have been reached
    def send_data(self, queues, k, time, id, delay, triggers):

        if sum(self.rec) > sum(self.com) and id == 1:
            for i in self.neigh:
                n = np.where(self.neigh == i)
                # tarry algorithm
                if self.com[n] < 1 and i not in self.father:
                    # Send data to neighbouring non-father agent
                    queues[i].Enqueue(self.sum + self.data, self.id)

                    # Update variables after sending the sum
                    self.data = np.zeros([1, D])
                    self.com[n] += 1

                    triggers[self.id, 0, k] = self.id + 1
                    k += 1
                    time += max(Ts, delay)
                    break

                elif i == self.neigh[-1] and self.id not in self.father:
                    # Only if the last neighbour is a parent -- so all neighbours are parents send it back unless parent is self
                    queues[self.father[0]].Enqueue(self.sum + self.data, self.id)
                    self.data = np.zeros([1, D])

                    n = np.where(self.neigh == self.father[0])
                    self.com[n] += 1

                    triggers[self.id, 0, k] = self.id + 1
                    k += 1
                    time += max(Ts, delay)

                    # self.reset_param()
                    break
        elif id == 0 and (min(self.rec) >= max(self.com)) and max(self.rec) > 0:
            for i in self.neigh:
                n = np.where(self.neigh == i)
                queues[i].Enqueue(self.sum, self.id)
                self.com[n] += 1

            #triggers[self.id, 0, k] = self.id + 1

        return time, k, triggers

    def poll_data(self, queues, k, triggers):
        for i in self.neigh:
            if queues[self.id].check_queue(i):
                # Assign as parent when it is the first msg received
                if len(self.father) < 1:
                    self.father.insert(-1, i)

                self.sum = queues[self.id].Dequeue(i)
                triggers[self.id, 1, k] = self.id + 1
                self.rec[np.where(self.neigh == i)] += 1

        return triggers

    def check_received(self, id, dia):
        out = True
        if id == 1:
            if sum(self.rec) > len(self.neigh):
                out = False
        else:
            for i in range(len(self.neigh)):
                if (self.rec[i] < dia):
                    out = False

        return out

    def controller(self, N, error, u_max, t_start, x, t):
        x_final = (1 / N * self.sum).reshape(-1)

        zero_crossing.terminal = True
        zero_crossing.direction = -1
        if max(np.absolute(np.subtract(self.x0, x_final))) - error > 0:
            data = integrate.solve_ivp(control, [t_start, final_time], self.x0, events=zero_crossing,
                                   args=[K, u_max, x_final, error], max_step=Ts)
            t_time = np.append(t, data.t[:])
            x = np.append(x, data.y[:, :], axis=1)
        else:
            t_time = t

        return x, t_time

    def reset_param(self, param):
        if param:
            self.data = self.x0
        self.father = []
        self.com = np.zeros(len(self.neigh))
        self.rec = np.zeros(len(self.neigh))


def create_network(N, L):
    E = np.asarray(np.where(np.tril(L) == -1))
    num_E = E.shape[1]

    E_full = np.asarray(np.where(L == -1))

    # visualize network
    G = nx.Graph()
    G.add_nodes_from(np.arange(N))
    G.add_edges_from(list(map(tuple, E.T)))

    return num_E, E, E_full


def find_neighbours(i, E):
    id = np.asarray(np.where(E[0, :] == i))
    neighbours = E[1, id].reshape(id.shape[1])

    return neighbours


def compute_nb_comm(robots, N):
    Sum = 1
    for i in range(N):
        Sum += sum(robots[i].com)

    return Sum


def phase(queues, robots, start, k, time, delay, N, triggers, dis):
    nb_robots_finished = []
    dia = np.zeros(N)
    m = 0

    k_start = k
    for i in start:
        robots[i].rec = np.zeros(len(robots[i].neigh)) + 1
        robots[i].com = np.zeros(len(robots[i].neigh)) + 1
        _, _, triggers = robots[i].send_data(queues, k, time, 0, delay, triggers)

    k += 1

    for i in range(N):
        if i in start:
            dia[i] = dis + 1
        else:
            dia[i] = dis

    while len(nb_robots_finished) < N:
        for i in range(N):
            if k % 2 == 0:
                triggers = robots[i].poll_data(queues, k, triggers)
            else:
                _, _, triggers = robots[i].send_data(queues, k, time, 0, delay, triggers)
                m += 1

            # this line is not yet correct
            if robots[i].check_received(0, dia[i]):
                if i not in nb_robots_finished:
                    nb_robots_finished.append([i])

        if m > 0:
            # time += max(Ts, delay)
            m = 0
        k += 1

    return time, triggers


def tarry(queues, robots, delay, N, triggers, diameter):
    init_agents = 1
    start = random.sample(range(0, N), init_agents)
    total_com = np.zeros(N)

    k = 0
    time = 0

    robots[start[0]].rec[0] = 1
    robots[start[0]].father.append(start[0])
    time, k, triggers = robots[start[0]].send_data(queues, k, time, 1, delay, triggers)

    while robots[start[0]].check_received(1, 0):
        for i in range(N):
            # if k > 0:  #and robots[i].rec > 0
            triggers = robots[i].poll_data(queues, k, triggers)
            time, k, triggers = robots[i].send_data(queues, k, time, 1, delay, triggers)

    # Clear parameters for everyone
    for i in range(N):
        robots[i].reset_param(False)
        queues[i].clear_queue()

    #Execute phase algorithm after Tarry to distribute average consensus
    phase_time, _ = phase(queues, robots, start, k, time, delay, N, triggers, diameter)

    for i in range(N):
        total_com += robots[i].com[0]

    time = phase_time

    return time, start, total_com, triggers


def run_simulation(N, area, D, end_time, Ts, delay, L, x0, diameter, error, u_max):
    # create network
    triggers = np.zeros([N, 3, end_time + 100])
    x = np.zeros((N, D, end_time))
    #t_final = np.zeros((N, end_time))
    e, E, E_full = create_network(N, L)

    # create agents
    robots = [Agent(i, x0[i, :]) for i in range(N)]

    # create queues
    queues = [Queue() for j in range(N)]

    # assign neighbours/queues to each robot
    for i in range(N):
        robots[i].neigh = find_neighbours(i, E_full)
        robots[i].com = np.zeros(robots[i].neigh.shape)
        robots[i].rec = np.zeros(robots[i].neigh.shape)

    # first round of communications
    time_start, init_agent, total_comm, triggers = tarry(queues, robots, delay, N, triggers, diameter)

    start_time = np.repeat(time_start / Ts, N)
    stop_time = np.zeros(N)

    x = np.zeros((N, D, end_time + int(start_time[0])))
    t_final = np.zeros((N, end_time + int(start_time[0])))

    # run trajectory
    for i in range(N):
        x_final, time_final = robots[i].controller(N, error, u_max, time_start, np.repeat(x0[i,:, np.newaxis], int(start_time[0]), axis=1), np.arange(0, time_start, Ts))

        x[i,:, 0:x_final.shape[1]] = x_final

        t_final[i,0:time_final.shape[0]] = time_final[0:time_final.shape[0]]
        stop_time[i] = x_final.shape[1]

    # total_comm = math.floor(compute_nb_comm(robots))

    return x, t_final, total_comm, time_start, init_agent, triggers, stop_time
