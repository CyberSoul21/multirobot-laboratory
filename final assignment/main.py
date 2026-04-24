import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import math
from Agent import Agent

num_agents = 10
comm_range = 4 * math.sqrt(2) # As defined in paper, where r here is 1
# Here we do not define vm, we are considering all robots are able to move one step at each iteration
# Defining initial components

# Grid 
x_min, x_max = 0 , 10
y_min, y_max = 0 , 10
Nx, Ny = 11, 11
# x_min, x_max = 0 , 100
# y_min, y_max = 0 , 100
# Nx, Ny = 101, 101
x = np.linspace(x_min, x_max, Nx)
y = np.linspace(y_min, y_max, Ny)
X, Y = np.meshgrid(x, y) 
all_positions = np.array([(xi, yi) for xi in range(Nx) for yi in range(Ny)])
chosen_indices = np.random.choice(len(all_positions), size=num_agents, replace=False)
grid_pos = all_positions[chosen_indices] # different grid positions without repeating 

# Agents
A = [] # set of agents
for a in range(num_agents):
    A.append(Agent(id = a, comm_range = comm_range, posx=grid_pos[a,0], posy=grid_pos[a,1])) # I guess we will define an agent class

# # Targets
# Q = [] # set of targets
# Q = target_location(shape = 'A', n = num_agents ) # we can try to define a shape and distribute de target locations along it with the number of agents
# # Agents have limited range, so they wouldn't know all the robots goal, so the algorithm assumes that some robots will
# # have same target and they will re arrange. So the targel location will be random, we wont distribute targets at all.

# # Code to iterate over all agents at each time step


# Code to display grid
fig, ax = plt.subplots(figsize=(6, 6))

for agent in A:
    posx, posy = agent.get_pos()
    ax.plot(posx, posy, 'ro')
    
ax.set_title(f"Target formation with {num_agents} agents")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_xticks(range(x_min, x_max + 1))
ax.set_yticks(range(y_min, y_max + 1))
ax.set_xlim(x_min - 0.5, x_max + 0.5)
ax.set_ylim(y_min - 0.5, y_max + 0.5)
ax.grid(True)
plt.show()