# ----------------------------------------------
# Javier Almario, NIP: 962449
# Alvaro Provencio, NIP: 960625
# ----------------------------------------------

from matplotlib import pyplot as plt
from Agent import Agent
import numpy as np
import random
import math
from matplotlib.animation import FuncAnimation

# To show the system along the time
def update(frame):
    simulation_step()

    xs = [agent.get_pos()[0] for agent in A]
    ys = [agent.get_pos()[1] for agent in A]
    
    points.set_data(xs, ys)
    
    return points,

# It does an iteration time step for all the agents with a Listen-Think-Walk manner
def simulation_step():
    # Listen (Add as neigbors all the agents within communication range)
    for agent in A:
        neighbors = []
        for neighbor in A:
            if(neighbor is not agent and agent.can_communicate(neighbor)):
                neighbors.append(neighbor)
        agent.setNeighbors(neighbors = neighbors)

    # Think (Reorganize goal and plan next waypoint)
    for agent in A:
        agent.goal_selector(A,Q)
        agent.motion_planner(x_min, x_max, y_min, y_max)

    # Move 
    for agent in A:
        agent.move()

# It defines the goal positions for several formations 
def target_location(shape, n, x_min, x_max, y_min, y_max):
    Q = []

    if shape == "circle":
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2
        radius = min(x_max - x_min, y_max - y_min) / 3

        samples = max(10 * n, 100)
        angles = np.linspace(0, 2 * math.pi, samples, endpoint=False)

        for theta in angles:
            x = round(cx + radius * math.cos(theta))
            y = round(cy + radius * math.sin(theta))

            p = (x, y)

            if (
                x_min <= x <= x_max
                and y_min <= y <= y_max
                and p not in Q
            ):
                Q.append(p)

            if len(Q) == n:
                break

    elif shape == "A":
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

        for p in raw_points:
            x, y = p
            if (
                x_min <= x <= x_max
                and y_min <= y <= y_max
                and p not in Q
            ):
                Q.append(p)

            if len(Q) == n:
                break

    elif shape == "line":
        cx = (x_min + x_max) // 2
        available_y = list(range(y_min, y_max + 1))

        if n > len(available_y):
            raise ValueError(
                f"Line shape needs {n} targets, but only "
                f"{len(available_y)} vertical grid cells are available."
            )

        indices = np.linspace(0, len(available_y) - 1, n, dtype=int)

        for i in indices:
            p = (cx, available_y[i])
            if p not in Q:
                Q.append(p)

    else:
        raise ValueError("Unknown shape. Use: circle, A, line")

    if len(Q) < n:
        raise ValueError(
            f"Only generated {len(Q)} unique targets. "
            f"Increase grid size, reduce num_agents, or use another shape."
        )

    return Q[:n]

if __name__ == "__main__":


    # Defining initial components
    num_agents = 18
    comm_range = 4 * math.sqrt(2) # As defined in paper, where r here is 1
    Total_time = 20
    # Here we do not define vm, we are considering all robots are able to move one step at each iteration

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

    # Targets
    Q = target_location(shape="A", n=num_agents, x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max)
    # Agents have limited range, so they wouldn't know all the robots goal, so the algorithm assumes that some robots will
    # have same target and they will re arrange. So the targel location will be random, we wont distribute targets at all.
    for agent in A:
        agent.goal = random.choice(Q)

    # Code to display grid
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title(f"Target formation with {num_agents} agents")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_xticks(range(x_min, x_max + 1))
    ax.set_yticks(range(y_min, y_max + 1))
    ax.set_xlim(x_min - 0.5, x_max + 0.5)
    ax.set_ylim(y_min - 0.5, y_max + 0.5)
    ax.grid(True)

    # Display goals
    for qx, qy in Q:
        ax.plot(qx, qy, 'bs', markersize=8, fillstyle='none')

    # Code to iterate over all agents at each time step
    points, = ax.plot([], [], 'ro')
    ani = FuncAnimation(fig, update, frames=Total_time, interval=500)
    plt.show()

