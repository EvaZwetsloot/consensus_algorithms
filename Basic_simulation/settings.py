# Author: Eva Zwetsloot
# Python script used for set-up of the simulations

import numpy as np
from numpy import random as rn
import networkx as nx
import matplotlib.pyplot as plt
import random
import math

Ts = 0.001
area = np.array([10, 10])
D = 2
K = 100000
final_time = 35000

def create_initial_pos(N, area, D, L, radius):
    #random.seed(10) #10 #24 #54 #0 #1234
    p = np.zeros((N, D))
    for x in np.arange(N):
        for n in np.arange(D):
            p[x, n] = round(random.uniform(0, radius*2), 2)
            n = n + 1
        x = x + 1

    return p

def create_network(N, L, name, imagename, plot, radius):
    E = np.asarray(np.where(np.tril(L) == -1))
    num_E = E.shape[1]

    E_full = np.asarray(np.where(L == -1))

    # visualize network
    G = nx.Graph()
    G.add_nodes_from(np.arange(N))
    G.add_edges_from(list(map(tuple, E.T)))

    #Laplacian based positions
    p = nx.spring_layout(G, dim=2, k=1, scale=radius, center=[radius,radius])
    p_out = np.array(list(p.values()))

    d_max = abs(p_out-[5,5]).max()

    if plot:
        plt.figure()
        plt.title('Network topology')
        nx.draw(G, with_labels=True)
        num = str(N)
        plt.savefig(imagename + "_" + num + "_" + "graph" + '.png')

    return num_E, E, E_full, p_out, d_max

def select_network(network, N):

    if network == "Star":
        G = nx.star_graph(N - 1)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)
    elif network == "Mesh":
        G = nx.complete_graph(N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)
    elif network == "Line":
        G = nx.path_graph(N) #line graph
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)
    elif network == "Grid":
        if N == 9:
            G = nx.grid_2d_graph(3, 3)
            L = nx.laplacian_matrix(G).toarray()
        elif N == 16:
            G = nx.grid_2d_graph(4, 4)
            L = nx.laplacian_matrix(G).toarray()
        elif N % 3 == 0:
            G = nx.grid_2d_graph(math.ceil(N/3), 3)
            L = nx.laplacian_matrix(G).toarray()
        elif N % 2 == 0:
            G = nx.grid_2d_graph(math.ceil(N/2), 2)
            L = nx.laplacian_matrix(G).toarray()
        else :
            N = N-1
            G = nx.grid_2d_graph(math.ceil(N/2), 2)
            L = nx.laplacian_matrix(G).toarray()

        N = G.number_of_nodes()
        dis = nx.diameter(G)
    elif network == "Barbell":
        if N % 2 == 0:
            G = nx.barbell_graph(math.ceil((N-2)/2), 2)
            L = nx.laplacian_matrix(G).toarray()
        else:
            G = nx.barbell_graph(math.ceil((N-1)/2), 1)
            L = nx.laplacian_matrix(G).toarray()
        dis = nx.diameter(G)
    elif network == "Bino_tree":
        G = nx.binomial_tree(math.ceil(math.log(N, 2)))
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)
    elif network == "Rand_tree":
        G = nx.random_tree(N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "Tree":
        G = nx.balanced_tree(2,N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "Cycle":
        G = nx.cycle_graph(N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network =="Lattice":
        G = nx.triangular_lattice_graph(N, 3)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "Cube":
        G = nx.hypercube_graph(N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "Clique":
        #n = np.array([3,4,5,6,7,8])
        G = nx.ring_of_cliques(4, N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "Complete":
        G = nx.complete_graph(N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "Random":
        G = nx.random_regular_graph(3,N)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "SmallWorld":
        G = nx.connected_watts_strogatz_graph(N,2, 0.6)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    elif network == "Caveman":
        G = nx.connected_caveman_graph(N,4)
        L = nx.laplacian_matrix(G).toarray()
        N = G.number_of_nodes()
        dis = nx.diameter(G)

    else:
        L = np.array([[2, -1, -1, 0, 0], [-1, 2, -1, 0, 0], [-1, -1, 3, -1, 0], [0, 0, -1, 2, -1], [0, 0, 0, -1, 1]])
        N = 5
        dis = 3

    return L, N, dis


