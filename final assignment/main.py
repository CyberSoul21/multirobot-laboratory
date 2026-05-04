# ----------------------------------------------
# Javier Almario, NIP: 962449
# Alvaro Provencio, NIP: 960625
# ----------------------------------------------

from matplotlib.animation import FuncAnimation
from matplotlib import pyplot as plt
from Agent import Agent
import numpy as np
import random
import time
import csv

ani = None
step_times = []
#--METRICS--
# how far all agents are from their currently assigned goals
def compute_J1():
    total = 0
    for agent in A:
        total += agent.manhattan_distance(agent.get_actual_wp(), agent.get_goal())
    return total

# how many target positions are not currently assigned to any robot
def compute_J2():
    assigned_goals = set(agent.get_goal() for agent in A)
    return len(Q) - len(assigned_goals)

def compute_blocked_agents():
    return sum(1 for agent in A if not agent.valid_movement)

def compute_completed_agents(tol=1e-3):
    completed = 0

    for agent in A:
        x, y = agent.get_pos()
        gx, gy = agent.get_goal()

        if abs(x - gx) < tol and abs(y - gy) < tol:
            completed += 1

    return completed

# Simulation function
def update(frame):

    step_start = time.time()

    simulation_step()

    step_end = time.time()
    step_times.append(step_end - step_start)

    achieved_goals.append(compute_completed_agents())
    J1_history.append(compute_J1())
    J2_history.append(compute_J2())
    blocked_history.append(compute_blocked_agents())

    xs = [agent.get_pos()[0] for agent in A]
    ys = [agent.get_pos()[1] for agent in A]

    points.set_offsets(np.c_[xs, ys])
    points.set_color(colors)
    points.set_sizes([25])

    if achieved_goals[-1] == num_agents:
        print("Formation completed!")
        ani.event_source.stop()

    return points,

# It does an iteration time step for all the agents with a Listen-Think-Walk manner
def simulation_step():
    # Listen (Add as neigbors all the agents within communication range)
    for agent in A:
        neighbors = []
        for neighbor in A:
            if neighbor is not agent and agent.can_communicate(neighbor):
                neighbors.append(neighbor)
        agent.setNeighbors(neighbors=neighbors)

    # Think: Gradient propagation
    for agent in A:
        agent.update_gradient_candidate(Q)

    # Think: Goal selector
    for agent in A:
        agent.goal_selector(Q)

    # Think: Local task swapping before motion planning
    processed_pairs = set()
    for agent in A:
        for other in agent.neighbors:
            pair = tuple(sorted((agent.id, other.id)))
            if pair in processed_pairs:
                continue

            agent.try_goal_swap(other)
        
            processed_pairs.add(pair)

    # Think: Motion planning after final goal for this step is known
    for agent in A:
        agent.motion_planner(x_min, x_max, y_min, y_max)

    # Move: ALL agents decide simultaneously (no one moves yet)
    for agent in A:
        agent.decide_move()

    # Move: ALL agents act on their pre-computed decision
    for agent in A:
        agent.commit_move(x_min, x_max, y_min, y_max)
        history[agent.id].append(agent.get_pos())  

# It defines the goal positions for the letter H, variable on grid dimensions and agent number
def generate_H(x_min, x_max, y_min, y_max, n_points):
    # Adjust grid
    cx = (x_min + x_max) / 2
    cy = (y_min + y_max) / 2
    grid_w = x_max - x_min
    grid_h = y_max - y_min

    # H has 3 arms
    half_h = grid_h * 0.40   
    arm_x  = grid_w * 0.20   
    bar_hw = arm_x * 0.55    
    leg_len = 2 * half_h
    bar_len = 2 * bar_hw
    total_len = 2 * leg_len + bar_len
    n_leg_l = max(2, round(n_points * leg_len / total_len))
    n_leg_r = max(2, round(n_points * leg_len / total_len))
    n_bar   = max(2, n_points - n_leg_l - n_leg_r)

    # Makes the shape smoother
    def snap(pts):
        snapped = set()
        for x, y in pts:
            sx = int(round(x))
            sy = int(round(y))
            sx = max(x_min, min(x_max, sx))
            sy = max(y_min, min(y_max, sy))
            snapped.add((sx, sy))
        return sorted(snapped)

    # Builds each arm
    left_x  = cx - arm_x
    ys_l = np.linspace(cy - half_h, cy + half_h, n_leg_l)
    leg_left = snap([(left_x, y) for y in ys_l])

    right_x = cx + arm_x
    ys_r = np.linspace(cy - half_h, cy + half_h, n_leg_r)
    leg_right = snap([(right_x, y) for y in ys_r])

    xs_b = np.linspace(cx - bar_hw, cx + bar_hw, n_bar)
    bar = snap([(x, cy) for x in xs_b])

    all_points = list(dict.fromkeys(leg_left + leg_right + bar))

    # In case they are not defined all the goals
    if len(all_points) < n_points:
        i = 0
        while len(all_points) < n_points:
            p = all_points[i % len(all_points)]

            # Expand locally
            candidates = [
                (p[0] + 1, p[1]),
                (p[0] - 1, p[1]),
                (p[0], p[1] + 1),
                (p[0], p[1] - 1),
            ]

            for c in candidates:
                c = (max(x_min, min(x_max, c[0])),
                     max(y_min, min(y_max, c[1])))
                if c not in all_points:
                    all_points.append(c)
                    if len(all_points) == n_points:
                        break
            i += 1
            # Check anti-loop
            if i > 100000:
                break

    return all_points

if __name__ == "__main__":
    # Defining initial components
    num_agents = 20
    comm_range = 2.5 # l is 1 unit, R (comm_range) needs to be > 2*l, Agent radius is expected to be < l/(2*sqrt(2))
    Total_time = 50
    grid_size = 10

    # Grid 
    x_min, x_max = 0 , grid_size
    y_min, y_max = 0 , grid_size
    Nx, Ny = x_max + 1 , y_max + 1
    x = np.linspace(x_min, x_max, Nx)
    y = np.linspace(y_min, y_max, Ny)
    X, Y = np.meshgrid(x, y) 
    all_positions = np.array([(xi, yi) for xi in range(Nx) for yi in range(Ny)])
    chosen_indices = np.random.choice(len(all_positions), size=num_agents, replace=False)
    grid_pos = all_positions[chosen_indices] # different grid positions without repeating 

    # Agents
    A = [] # set of agents
    for a in range(num_agents):
        A.append(Agent(id=a, comm_range=comm_range,
                    posx=int(grid_pos[a, 0]),   
                    posy=int(grid_pos[a, 1])))  

    # Targets
    Q = generate_H(n_points=num_agents, x_min=x_min, x_max=x_max, y_min=y_min, y_max=y_max)
    print("Number of agents:",len(A))
    print("Number of goals:", len(Q))
    
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
        ax.plot(qx, qy, 'gs', markersize=300/(1.5*grid_size), fillstyle='none')

    # Code to iterate over all agents at each time step
    points = ax.scatter([], [])
    achieved_goals = []
    J1_history = []
    J2_history = []
    blocked_history = []
    
    start_time = time.time()

    colors = plt.cm.nipy_spectral(np.linspace(0, 1, num_agents)) 
    history = {agent.id: [agent.pos] for agent in A}
    ani = FuncAnimation(fig, update, frames=Total_time, interval=Total_time)
    plt.show()

    end_time = time.time()
    total_time = end_time - start_time

    print(f"Total simulation time: {total_time:.4f} seconds")

    if step_times:
        avg_step_time = sum(step_times) / len(step_times)
        max_step_time = max(step_times)
        min_step_time = min(step_times)

        print(f"Average step time: {avg_step_time:.6f} s")
        print(f"Max step time: {max_step_time:.6f} s")
        print(f"Min step time: {min_step_time:.6f} s")
    else:
        print("No simulation steps were executed.")


    # =========================
    # SUMMARY METRICS
    # =========================

    # Convergence: first time all agents reach goals
    convergence_time = None
    for t, val in enumerate(achieved_goals):
        if val == num_agents:
            convergence_time = t
            break

    # Final metrics
    final_J1 = J1_history[-1]
    final_J2 = J2_history[-1]
    max_completed = max(achieved_goals)

    # Blocked agents stats
    avg_blocked = sum(blocked_history) / len(blocked_history)
    max_blocked = max(blocked_history)

    print("\n===== METRICS SUMMARY =====")
    print(f"Convergence time: {convergence_time}")
    print(f"Final J1: {final_J1}")
    print(f"Final J2: {final_J2}")
    print(f"Max completed agents: {max_completed}/{num_agents}")
    print(f"Average blocked agents: {avg_blocked:.2f}")
    print(f"Max blocked agents: {max_blocked}")        

    # Store metrics
    with open("simulation_data.csv", "w", newline="") as f:
        writer = csv.writer(f)

        # Header
        writer.writerow([
            "step",
            "completed_agents",
            "J1",
            "J2",
            "blocked_agents",
            "step_time"
        ])

        # Data rows
        for t in range(len(achieved_goals)):
            writer.writerow([
                t,
                achieved_goals[t],
                J1_history[t],
                J2_history[t],
                blocked_history[t],
                step_times[t] if t < len(step_times) else None
            ])
    

    # To show final positions and goals achieved
    for agent in A:
        print(agent)

    plt.figure()
    plt.plot(achieved_goals)
    plt.xlabel("Time")
    plt.ylabel("Number of achieved goals")
    plt.title("Goals achieved over time")
    plt.grid()
    plt.show()

    #Metrics
    plt.figure()
    plt.plot(achieved_goals)
    plt.xlabel("Time step")
    plt.ylabel("Completed agents")
    plt.title("Completed agents over time")
    plt.grid()
    plt.show()

    plt.figure()
    plt.plot(J1_history)
    plt.xlabel("Time step")
    plt.ylabel("J1")
    plt.title("J1: Total Manhattan distance to assigned goals")
    plt.grid()
    plt.show()

    plt.figure()
    plt.plot(J2_history)
    plt.xlabel("Time step")
    plt.ylabel("J2")
    plt.title("J2: Number of missing assigned goals")
    plt.grid()
    plt.show()

    plt.figure()
    plt.plot(blocked_history)
    plt.xlabel("Time step")
    plt.ylabel("Blocked agents")
    plt.title("Blocked agents over time")
    plt.grid()
    plt.show()       