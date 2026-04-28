# ----------------------------------------------
# Javier Almario, NIP: 962449
# Alvaro Provencio, NIP: 960625
# ----------------------------------------------

from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
from Agent import Agent
import numpy as np
import random
import math

# To show the system development along the time
def update(frame):
    global prev_positions
    global current_positions
    step = (frame % substeps) / substeps

    if frame % substeps == 0:
        # To avoid iterating a lot to obtain the position
        try:
            current_positions
        except:
            prev_positions = [agent.get_pos() for agent in A]
        else:
            prev_positions = current_positions

        simulation_step()
        current_positions = [agent.get_pos() for agent in A]

    xs = []
    ys = []
    
    for (x0, y0), (x1, y1) in zip(prev_positions, current_positions):
        x = x0 + (x1 - x0) * step
        y = y0 + (y1 - y0) * step
        xs.append(x)
        ys.append(y)

    points.set_offsets(np.c_[xs, ys])
    points.set_color(colors)
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
        # history[agent.id].append((agent.get_pos()))
        history[agent.id].append(agent.get_pos())

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
    comm_range = 4 * math.sqrt(2) # As defined in paper, where agent here is 1
    Total_time = 50
    shape =  "A" #"circle" "line" "A"
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
    Q = target_location(shape=shape, n=num_agents, x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max)
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
        ax.plot(qx, qy, 'gs', markersize=8, fillstyle='none')

    # Code to iterate over all agents at each time step
    points = ax.scatter([], [])
    colors = plt.cm.nipy_spectral(np.linspace(0, 1, num_agents))  # un color por agente
    # ani = FuncAnimation(fig, update, frames=Total_time, interval=500)
    substeps = 10
    history = {agent.id: [(agent.x, agent.y)] for agent in A}
    ani = FuncAnimation(fig, update, frames=Total_time*substeps, interval=500/substeps)
    plt.show()


    # Plot trayectories to show collision. 
    # Right now we show evoulution on X and Y, which does not help at all
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
    num_iter = len(next(iter(history.values())))
    v_x = np.arange(num_iter)

    ax1.set_title("Evolution of the x-coordinates")
    ax2.set_title("Evolution of the y-coordinates")
    ax1.set_ylabel("x-coordinate")
    ax2.set_ylabel("y-coordinate")
    ax2.set_xlabel("Iterations")

    for agent in A:
        traj = history[agent.id]

        xk = [p[0] for p in traj]
        yk = [p[1] for p in traj]

        for ax, data in zip((ax1, ax2), (xk, yk)):

            line, = ax.plot(v_x, data, marker='.')
            c = line.get_color()
            ax.plot(v_x[0],  data[0],  marker='x', color=c)
            ax.plot(v_x[-1], data[-1], marker='o', color=c)

        ax1.text(v_x[0], xk[0], f'{agent.id}', fontsize=8, ha='right', va='bottom', color=c)
        ax2.text(v_x[0], yk[0], f'{agent.id}', fontsize=8, ha='right', va='bottom', color=c)
        ax1.text(v_x[-1], xk[-1], f'{agent.id}', fontsize=8, ha='right', va='bottom', color=c)
        ax2.text(v_x[-1], yk[-1], f'{agent.id}', fontsize=8, ha='right', va='bottom', color=c)

    plt.tight_layout()
    plt.show()
