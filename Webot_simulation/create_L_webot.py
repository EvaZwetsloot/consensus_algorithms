import numpy as np
from numpy import random as rn
import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import os.path
from os import path

# Cube graph
n = 4
G = nx.hypercube_graph(n)
L = nx.laplacian_matrix(G).toarray()
N = G.number_of_nodes()

dis = nx.diameter(G)
radius = 0.2

# Create topology depend positions
radius2 = 0.9
top_p = nx.spring_layout(G, dim=2, k=0.6, scale=radius2, center=[radius2, radius2])
top_p_out = np.array(list(top_p.values()))+0.1

filename = "webot_" + "cube_" + "shapes" + ".txt"
if not path.exists(filename):
    file = open(filename, "w+")
    file.close()

file = open(filename, "a+") #w+ is to create and overwrite
file.write('int ' + 'L[ROBOTS][ROBOTS] = {')
for i in range(N):
    file.write('{')
    for j in range(N):
        file.write('%d' %L[i][j])

        if (j < (N-1)):
            file.write(', ')

    if (i < (N-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')

pos = nx.circular_layout(G, radius, dim=2)
p_out = np.array(list(pos.values()))

plt.figure()
plt.scatter(p_out[:,0], p_out[:,1])
plt.show()


Bx = np.ones((1,N))*p_out[:,0].reshape((N,1)) - np.ones((N,1))*p_out[:,0]
By = np.ones((1,N))*p_out[:,1].reshape((N,1)) - np.ones((N,1))*p_out[:,1]

L_stripped = L
L_stripped[L<0] = 1
L_stripped[L_stripped >0] = 1

Bx = L_stripped*(np.ones((1,N))*p_out[:,0].reshape((N,1)) - np.ones((N,1))*p_out[:,0])
By = L_stripped*(np.ones((1,N))*p_out[:,1].reshape((N,1)) - np.ones((N,1))*p_out[:,1])

Bx_sup = (np.ones((1,N))*p_out[:,0].reshape((N,1)) - np.ones((N,1))*p_out[:,0])
By_sup = (np.ones((1,N))*p_out[:,1].reshape((N,1)) - np.ones((N,1))*p_out[:,1])

file.write('float ' + 'Bx[ROBOTS][ROBOTS] = {')
for i in range(N):
    file.write('{')
    for j in range(N):
        file.write('%.3f' %Bx[i][j])

        if (j < (N-1)):
            file.write(', ')

    if (i < (N-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('float ' + 'By[ROBOTS][ROBOTS] = {')
for i in range(N):
    file.write('{')
    for j in range(N):
        file.write('%.3f' %By[i][j])

        if (j < (N-1)):
            file.write(', ')

    if (i < (N-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('Supervisor:')
file.write('\n')
file.write('int ' + 'L[ROBOTS][ROBOTS] = {')
for i in range(N):
    file.write('{')
    for j in range(N):
        file.write('%d' %L[i][j])

        if (j < (N-1)):
            file.write(', ')

    if (i < (N-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('float ' + 'Bx[ROBOTS][ROBOTS] = {')
for i in range(N):
    file.write('{')
    for j in range(N):
        file.write('%.3f' %Bx_sup[i][j])

        if (j < (N-1)):
            file.write(', ')

    if (i < (N-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('float ' + 'By[ROBOTS][ROBOTS] = {')
for i in range(N):
    file.write('{')
    for j in range(N):
        file.write('%.3f' %By_sup[i][j])

        if (j < (N-1)):
            file.write(', ')

    if (i < (N-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('float ' + 'top_pos[D][ROBOTS] = {')
for i in range(2):
    file.write('{')
    for j in range(N):
        file.write('%.3f' %top_p_out[j][i])

        if (j < (N-1)):
            file.write(', ')

    if (i < (1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('double B[D][ROBOTS] = {')
for i in range(2):
    file.write('{')
    for j in range(N):
        file.write('%.4f' %p_out[j][i])

        if (j < (N-1)):
            file.write(', ')

    if (i < (1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')

Ni = np.zeros(N, dtype=int)
L = nx.laplacian_matrix(G).toarray()

for i in range(N):
    for j in range(N):
        if L[i][j] < 0 :
            Ni[i] += 1

file.write('int Ni[ROBOTS] = {')
for j in range(N):
    file.write('%d' %Ni[j])

    if (j < (N-1)):
        file.write(', ')
file.write('};')
file.write('\n')
file.close()

