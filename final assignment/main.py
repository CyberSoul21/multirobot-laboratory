import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import random
from Agent import Agent

num_agents = 18
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
    import numpy as np
    import math

    Q = []

    if shape == "circle":
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2
        radius = min(x_max - x_min, y_max - y_min) / 3

        angles = np.linspace(0, 2 * math.pi, n, endpoint=False)

        for theta in angles:
            x = round(cx + radius * math.cos(theta))
            y = round(cy + radius * math.sin(theta))
            Q.append((x, y))

    elif shape == "A":
        # Simple letter A using grid points
        raw_points = [
            (5, 9),
            (4, 8), (6, 8),
            (3, 7), (7, 7),
            (3, 6), (7, 6),
            (3, 5), (4, 5), (5, 5), (6, 5), (7, 5),
            (3, 4), (7, 4),
            (3, 3), (7, 3),
            (3, 2), (7, 2),
        ]

        Q = raw_points[:n]

    elif shape == "line":
        cx = (x_min + x_max) // 2
        ys = np.linspace(y_min, y_max, n, dtype=int)
        Q = [(cx, int(y)) for y in ys]

    else:
        raise ValueError("Unknown shape. Use: circle, A, line")

    # Remove repeated points caused by rounding
    Q_unique = []
    for p in Q:
        if p not in Q_unique:
            Q_unique.append(p)

    if len(Q_unique) < n:
        raise ValueError(
            f"Only generated {len(Q_unique)} unique targets. "
            f"Increase grid size or reduce num_agents."
        )

    return Q_unique[:n]
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
    shape="A",
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


# ============================================================
# TEST YOUR PART: TARGET GENERATION
# ============================================================

test_shapes = ["line", "square", "random_connected"]

for shape in test_shapes:
    Q_test = target_location(
        shape=shape,
        n=num_agents,
        x_min=x_min,
        x_max=x_max,
        y_min=y_min,
        y_max=y_max
    )

    print(f"\nShape: {shape}")
    print("Q =", Q_test)
    print("Number of targets:", len(Q_test))
    print("Unique targets:", len(set(Q_test)))

    assert len(Q_test) == num_agents, "Wrong number of targets"
    assert len(set(Q_test)) == num_agents, "Repeated target found"

    for qx, qy in Q_test:
        assert x_min <= qx <= x_max, "Target x outside grid"
        assert y_min <= qy <= y_max, "Target y outside grid"

print("\nAll target-generation tests passed.")



plt.show()