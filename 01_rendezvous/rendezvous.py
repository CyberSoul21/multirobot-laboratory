from Robot import Robot as rbt
from graph import graph
import matplotlib.pyplot as plt
import numpy as np
import random
import threading
import time

#Thread function
def robot_thread(robot, t_total):
    start_time = time.time()
    actual_time = start_time
    last_call = start_time
    while actual_time - start_time < t_total:
        actual_time = time.time()
        if actual_time - last_call > robot.t_local:
            last_call = actual_time
            robot.gossip_step()


if __name__ == "__main__":
    # Initial params
    Num_robots = 6
    t_total = 15
    bx = 0
    by = 0

    # Graph setup
    communication_graph = np.array([[5, 1], [0, 2], [1, 3], [2, 4],
                                    [3, 5], [4, 0], [1, 4], [5, 0]])
                                    # [3, 5], [4, 0]]) # 2 disconnected graphs 
    # undirected = True
    # alpha = 0.125
    # my_graph = graph(Num_robots, communication_graph, undirected)
    # my_graph.define_W_matrix(alpha)
    # my_graph.eigenvals()
    # my_graph.plot_graph()

    # Robot declaration and neighbours setup
    robots = []
    for i in range(Num_robots):
        # Random initial positions
        x = random.randint(0, 10)
        y = random.randint(0, 10)
        robots.append(rbt(i, x, y, bx, by))

    for i, j in communication_graph:
        robots[i].addNode(robots[j])
        robots[j].addNode(robots[i])


    # Print initial positions of robots
    print("Initial Positions of Robots:")
    for robot in robots:
        print(robot)

    # Trail with past steps
    history = {r.id: [(r.x, r.y)] for r in robots}
    x0 = [r.x for r in robots]
    y0 = [r.y for r in robots]


    # Launch threads
    threads = []
    for robot in robots:
        t = threading.Thread(target=robot_thread, args = (robot,t_total))
        t.start()
        threads.append(t)

    # Consensus process
    start_time = time.time()
    actual_time = start_time
    T_plot = 0.05   # periodo de muestreo

    while actual_time - start_time < t_total:
        # print(actual_time - start_time)
        time.sleep(T_plot)
        actual_time = time.time()

        # Update interactive plot
        current_positions = [r.get_pos() for r in robots]

        xs, ys = [], []
        for i, (curr_x, curr_y) in enumerate(current_positions):
            r_id = robots[i].id
            history[r_id].append((curr_x,curr_y))


    # Join threads
    for t in threads:
        t.join()

    print("-----End consensus-----")
    for robot in robots:
        print(robot)

    # Plot the sequence
    plt.ion()
    n_steps = len(next(iter(history.values())))

    fig, ax = plt.subplots()
    init_scat = ax.scatter(x0, y0, marker='x',alpha=0.6)
    scat = ax.scatter(x0, y0)

    labels = []
    for r in robots:
        l = ax.text(r.x+0.1, r.y+0.1, f'{r.id}', fontsize=8)
        labels.append(l)

    ax.set_xlim(-5,15)
    ax.set_ylim(-5,15)
    ax.set_title("Gossip Consensus")
    plt.draw()
    plt.pause(0.1)

    # Trail with past steps
    lines = {}
    for r in robots:
        line, = ax.plot([],[],
                        linewidth=1,
                        alpha=0.2)
        lines[r.id] = line

    for k in range(n_steps):
        time.sleep(T_plot)

        xs, ys = [], []
        for i, r in enumerate(robots):
            xk, yk = history[r.id][k]
            xs.append(xk)
            ys.append(yk)

            labels[i].set_position((xk+0.1, yk+0.1))
            hx = [p[0] for p in history[r.id][:k]]
            hy = [p[1] for p in history[r.id][:k]]
            lines[r.id].set_data(hx, hy)

        scat.set_offsets(list(zip(xs,ys)))
        fig.canvas.draw_idle()
        plt.pause(T_plot)

    print("-----End plot-----")
    plt.ioff()
    plt.show()
