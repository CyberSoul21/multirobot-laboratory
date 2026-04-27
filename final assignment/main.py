import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import random
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


# ***********************************************************
# Javier: TARGET GENERATION FUNCTION
# This function creates the desired shape Q inside the grid.
# Q is the set of target positions from the paper.
# ***********************************************************

def target_location(shape, n, x_min, x_max, y_min, y_max):
    Q = []

    if shape == "line":
        cx = (x_min + x_max) // 2
        y_values = np.linspace(y_min, y_max, n, dtype=int)
        Q = [(cx, int(y)) for y in y_values]

    elif shape == "square":
        side = int(np.ceil(np.sqrt(n)))
        cx = (x_min + x_max) // 2
        cy = (y_min + y_max) // 2

        start_x = cx - side // 2
        start_y = cy - side // 2

        for i in range(side):
            for j in range(side):
                if len(Q) < n:
                    qx = start_x + i
                    qy = start_y + j

                    if x_min <= qx <= x_max and y_min <= qy <= y_max:
                        Q.append((qx, qy))

    elif shape == "random_connected":
        cx = (x_min + x_max) // 2
        cy = (y_min + y_max) // 2

        Q = [(cx, cy)]
        frontier = [(cx, cy)]

        while len(Q) < n and frontier:
            bx, by = random.choice(frontier)

            neighbors = [
                (bx + 1, by),
                (bx - 1, by),
                (bx, by + 1),
                (bx, by - 1),
            ]

            random.shuffle(neighbors)

            added = False
            for nx, ny in neighbors:
                if (
                    x_min <= nx <= x_max
                    and y_min <= ny <= y_max
                    and (nx, ny) not in Q
                ):
                    Q.append((nx, ny))
                    frontier.append((nx, ny))
                    added = True
                    break

            if not added:
                frontier.remove((bx, by))

    else:
        raise ValueError("Unknown shape. Use: line, square, random_connected")

    if len(Q) < n:
        raise ValueError("Grid is too small for this number of targets.")

    return Q[:n]
# ***********************************************************
# ***********************************************************

# ***********************************************************
# AGENTS INITIALIZATION
# ***********************************************************

# Agents
A = [] # set of agents
for a in range(num_agents):
    A.append(Agent(id = a, comm_range = comm_range, posx=grid_pos[a,0], posy=grid_pos[a,1])) # I guess we will define an agent class





# ***********************************************************
# Javier: CREATE TARGETS
# only need to change the shape here.
# Options: "line", "square", "random_connected"
# ***********************************************************
# TODO: Make recive shape as argument
# # Targets
# Q = [] # set of targets
# Q = target_location(shape = 'A', n = num_agents ) # we can try to define a shape and distribute de target locations along it with the number of agents

Q = target_location(
    shape="square",
    n=num_agents,
    x_min=x_min,
    x_max=x_max,
    y_min=y_min,
    y_max=y_max
)
# ***********************************************************
# ***********************************************************

# # Agents have limited range, so they wouldn't know all the robots goal, so the algorithm assumes that some robots will
# # have same target and they will re arrange. So the targel location will be random, we wont distribute targets at all.

# # Code to iterate over all agents at each time step

# ***********************************************************
# INITIAL GOAL ASSIGNMENT
# For now, assign random goals.
# This allows repeated goals, as expected in the paper.
# Later the goal_selector should fix duplicates.
# ***********************************************************
for agent in A:
    agent.goal = random.choice(Q)
# ***********************************************************
# ***********************************************************    


# ***********************************************************
# ALVARO PART: SIMULATION LOOP LOGIC
# The function should implement:
# 1. call each agent.motion_planner()
# 2. detect waypoint conflicts
# 3. move only safe agents
# 4. call each agent.goal_selector()
# 5. implement local task swapping
# ***********************************************************
def simulation_step():
    # TODO:
    # for agent in A:
    #     agent.motion_planner(...)
    #
    # TODO:
    # resolve collisions and waypoint conflicts
    #
    # TODO:
    # update agent positions
    #
    # TODO:
    # resolve duplicate goals and swaps
    pass
#***********************************************************
#***********************************************************


#TODO: Check this test, chatGPT generated...
# ============================================================
# VISUALIZATION
# Red circles: agents
# Blue squares: target positions Q
# ============================================================

fig, ax = plt.subplots(figsize=(6, 6))

for agent in A:
    posx, posy = agent.get_pos()
    ax.plot(posx, posy, 'ro')

for qx, qy in Q:
    ax.plot(qx, qy, 'bs', markersize=8, fillstyle='none')

ax.set_title(f"Target formation with {num_agents} agents")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_xticks(range(x_min, x_max + 1))
ax.set_yticks(range(y_min, y_max + 1))
ax.set_xlim(x_min - 0.5, x_max + 0.5)
ax.set_ylim(y_min - 0.5, y_max + 0.5)
ax.grid(True)
plt.show()