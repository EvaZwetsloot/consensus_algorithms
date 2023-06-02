import numpy as np
from numpy import random as rn
import networkx as nx
import matplotlib.pyplot as plt
import random
import math
import os.path
from os import path

# Cube graph
n = 2
G = nx.hypercube_graph(n)
L = nx.laplacian_matrix(G).toarray()
N = G.number_of_nodes()

dis = nx.diameter(G)
radius = 0.2 

filename = "elisa_" + "cube_" + "shapes" + ".txt"
if not path.exists(filename):
    file = open(filename, "w+")
    file.close()

file = open(filename, "a+") #w+ is to create and overwrite
file.write('int ' + 'L[18][18] = {')
for i in range(18):
    file.write('{')
    for j in range(18):
        if (i < N and j < N):
            file.write('%d' %L[i][j])
        else :
            file.write('0')

        if (j < (18-1)):
            file.write(', ')

    if (i < (18-1))  :
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

Bx = L_stripped*(np.ones((1,N))*p_out[:,0].reshape((N,1)) - np.ones((N,1))*p_out[:,0])*1000
By = L_stripped*(np.ones((1,N))*p_out[:,1].reshape((N,1)) - np.ones((N,1))*p_out[:,1])*1000

Bx_sup = (np.ones((1,N))*p_out[:,0].reshape((N,1)) - np.ones((N,1))*p_out[:,0])*1000
By_sup = (np.ones((1,N))*p_out[:,1].reshape((N,1)) - np.ones((N,1))*p_out[:,1])*1000

p_out = p_out*1000

file.write('float ' + 'Bx[18][18] = {')
for i in range(18):
    file.write('{')
    for j in range(18):
        if (j<N and i<N):
            file.write('%d' %Bx[i][j])
        else :
            file.write('0')

        if (j < (18-1)):
            file.write(', ')

    if (i < (18-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('float ' + 'By[18][18] = {')
for i in range(18):
    file.write('{')
    for j in range(18):
        if (j<N and i<N):
            file.write('%d' %By[i][j])
        else :
            file.write('0')

        if (j < (18-1)):
            file.write(', ')

    if (i < (18-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('Supervisor:')
file.write('\n')
file.write('float ' + 'Bx[18][18] = {')
for i in range(18):
    file.write('{')
    for j in range(18):
        if (j<N and i<N):
            file.write('%d' %Bx_sup[i][j])
        else :
            file.write('0')

        if (j < (18-1)):
            file.write(', ')

    if (i < (18-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('float ' + 'By[18][18] = {')
for i in range(18):
    file.write('{')
    for j in range(18):
        if (j<N and i<N):
            file.write('%d' %By_sup[i][j])
        else :
            file.write('0')

        if (j < (18-1)):
            file.write(', ')

    if (i < (18-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('Phase:')
file.write('\n')
file.write('double Bx[18][18] = {')
for i in range(18):
    file.write('{')
    for j in range(18):
        if (j<N):
            file.write('%d' %p_out[j][0])
        else :
            file.write('0')

        if (j < (18-1)):
            file.write(', ')

    if (i < (18-1))  :
        file.write('}, ')
    else:
        file.write('}')
file.write('};')
file.write('\n')
file.write('double By[18][18] = {')
for i in range(18):
    file.write('{')
    for j in range(18):
        if(j < N):
            file.write('%d' %p_out[j][1])
        else :
            file.write('0')

        if (j < (18-1)):
            file.write(', ')

    if (i < (18-1))  :
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

