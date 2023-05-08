import matplotlib.pyplot as plt
import numpy as np
import math
from mpl_toolkits.mplot3d import Axes3D
plt.close("all")

N_min = 3
N_max = 8
N_delay = 18
N_error = 10
N_graph = 4

d_fixed = 6
e_fixed = 1
node = 0

# Load data
net = ["Cube", "Line"]
filename = "Results/RAW_data/network_" + net[0] + "_error" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig2, ax2 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])
i = 0



for n in range:
    ax2[0].plot(data5[:,3], data5[:,n], label=types[i])
    i+=1
ax2[0].set(xlabel="Error radius", ylabel="Time [sec]")
ax2[0].set_title("Convergence time")
ax2[0].legend()

i=0
range = range+6
for n in range:
    ax2[1].plot(data5[:,3], data5[:,n], label=types[i])
    i+=1
ax2[1].set(xlabel="Error radius", ylabel="Events")
ax2[1].set_title("Number of Events")
ax2[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig2.suptitle("Effect of error radius on hypercube graph with random positions")

#----------------------------------------Position ------------------

filename = "Results/RAW_data/network_" + net[0] + "_init" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig, ax = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])
data_x = [10,15,20]


i=0
for n in range:
    ax[0].plot(data_x, data5[:,n], label=types[i])
    i+=1
ax[0].set(xlabel="Radius", ylabel="Time [sec]")
ax[0].set_title("Convergence time")
ax[0].legend()

i = 0
range = range+6
for n in range:
    ax[1].plot(data_x, data5[:,n], label=types[i])
    i+=1
ax[1].set(xlabel="Radius", ylabel="Events")
ax[1].set_title("Number of Events")
ax[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig.suptitle("Effect of radius on hypercube graph with random positions")

#------------------------------- DELAY 1 ---------------------------------------------------

filename = "Results/RAW_data/network_" + net[0] + "_delay" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig3, ax3 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])



i=0
for n in range:
    ax3[0].plot(data5[:,2], data5[:,n], label=types[i])
    i+=1
ax3[0].set(xlabel="Delay", ylabel="Time [sec]")
ax3[0].set_title("Convergence time")
ax3[0].legend()

i=0
range = range+6
for n in range:
    ax3[1].plot(data5[:,2], data5[:,n], label=types[i])
    i+=1
ax3[1].set(xlabel="Delay", ylabel="Events")
ax3[1].set_title("Number of Events")
ax3[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig3.suptitle("Effect of delay on hypercube graph with random positions")


#----------------------------- AC: Star ---------------------

filename = "Results/RAW_data/network_Star" + "_ac" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig3, ax3 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])

i = 0
for n in range:
    ax3[0].plot(data5[:,0], data5[:,n], label=types[i])
    i+=1
ax3[0].set(xlabel="Nodes", ylabel="Time [sec]")
ax3[0].set_title("Convergence time")
ax3[0].legend()

i=0
range = range+6
for n in range:
    ax3[1].plot(data5[:,0], data5[:,n], label=types[i])
    i+=1
ax3[1].set(xlabel="Nodes", ylabel="Events")
ax3[1].set_title("Number of Events")
ax3[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig3.suptitle("Effect of nodes on Star graph with random positions")

#----------------------------- AC: Path ---------------------

filename = "Results/RAW_data/network_Line" + "_ac" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig5, ax5 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])

i = 0
for n in range:
    ax5[0].plot(data5[:,17], data5[:,n], label=types[i])
    i+=1
ax5[0].set(xlabel="Nodes", ylabel="Time [sec]")
ax5[0].set_title("Convergence time")
ax5[0].legend()

i=0
range = range+6
for n in range:
    ax5[1].plot(data5[:,17], data5[:,n], label=types[i])
    i+=1
ax5[1].set(xlabel="Nodes", ylabel="Events")
ax5[1].set_title("Number of Events")
ax5[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig5.suptitle("Effect of lambda on Path graph with random positions")


#----------------------------- AC: Star ---------------------

filename = "Results/RAW_data/network_Star" + "_ac" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig3, ax3 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])
edge = np.array([])

i = 0
for n in range:
    ax3[0].plot(data5[:,4], data5[:,n]-0.1*(data5[:,0]-1), label=types[i])
    i+=1
ax3[0].set(xlabel="Nodes", ylabel="Time [sec]")
ax3[0].set_title("Convergence time")
ax3[0].legend()

i=0
range = range+6
for n in range:
    ax3[1].plot(data5[:,4], data5[:,n]-data5[:,0]*2, label=types[i])
    i+=1
ax3[1].set(xlabel="Nodes", ylabel="Events")
ax3[1].set_title("Number of Events")
ax3[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig3.suptitle("Effect of umax on Star graph with random positions")

#----------------------------- AC: Path 2 ---------------------

filename = "Results/RAW_data/network_Line" + "_ac_s" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig5, ax5 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])

i = 0
for n in range:
    ax5[0].plot(data5[:,18], data5[:,n], label=types[i])
    i+=1
ax5[0].set(xlabel="Nodes", ylabel="Time [sec]")
ax5[0].set_title("Convergence time")
ax5[0].legend()

i=0
range = range+6
for n in range:
    ax5[1].plot(data5[:,18], data5[:,n], label=types[i])
    i+=1
ax5[1].set(xlabel="Nodes", ylabel="Events")
ax5[1].set_title("Number of Events")
ax5[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig5.suptitle("Effect of lambda on Path graph with spring positions")


#----------------------------- AC: Cube ---------------------

filename = "Results/RAW_data/network_Cube" + "_ac" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig5, ax5 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])

i = 0
for n in range:
    ax5[0].plot(data5[:,0], data5[:,n], label=types[i])
    i+=1
ax5[0].set(xlabel="Nodes", ylabel="Time [sec]")
ax5[0].set_title("Convergence time")
ax5[0].legend()

i=0
range = range+6
for n in range:
    ax5[1].plot(data5[:,0], data5[:,n], label=types[i])
    i+=1
ax5[1].set(xlabel="Nodes", ylabel="Events")
ax5[1].set_title("Number of Events")
ax5[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig5.suptitle("Effect of lambda on Cube graph with random positions")

#----------------------------- AC: Cube 2 ---------------------

filename = "Results/RAW_data/network_Cube" + "_ac_s" + ".txt"
data5 = np.loadtxt(filename, comments="#", delimiter=',', usecols=(0,1,2,3,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19))

# Make a plot of the delays
fig5, ax5 = plt.subplots(2)

# range = np.arange(N_graph-1)+1
types = ["Tarry", "Phase", "ETC"]

range = np.array([6, 8, 10])

i = 0
for n in range:
    ax5[0].plot(data5[:,0], data5[:,n], label=types[i])
    i+=1
ax5[0].set(xlabel="Nodes", ylabel="Time [sec]")
ax5[0].set_title("Convergence time")
ax5[0].legend()

i=0
range = range+6
for n in range:
    ax5[1].plot(data5[:,0], data5[:,n], label=types[i])
    i+=1
ax5[1].set(xlabel="Nodes", ylabel="Events")
ax5[1].set_title("Number of Events")
ax5[1].legend()


plt.subplots_adjust(left=0.1,
                    bottom=0.1,
                    right=0.9,
                    top=0.85,
                    wspace=0.4,
                    hspace=0.7)

fig5.suptitle("Effect of lambda on Cube graph with spring positions")


plt.show()


