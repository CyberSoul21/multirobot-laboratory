import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

num_agent = 10
comm_range = 2
# Defining initial components

# Grid 
x_min, x_max = -10, 10
y_min, y_max = -10, 10
Nx, Ny = 100, 100
x = np.linspace(x_min, x_max, Nx)
y = np.linspace(y_min, y_max, Ny)
X, Y = np.meshgrid(x, y) 

# Agents
A = [] # set of agents
for n in num_agent:
    A.append(agent(id = n, comm_range = comm_range)) # I guess we will define an agent class

# Targets
Q = [] # set of targets
Q = target_location(shape = 'A', n = num_agent ) # we can try to define a shape and distribute de target locations along it with the number of agents


# Code to iterate over all agents at each time step


# Code to display grid