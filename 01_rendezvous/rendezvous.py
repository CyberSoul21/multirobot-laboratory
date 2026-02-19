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
    while actual_time - start_time < t_total:
        actual_time = time.time()
        if actual_time - last_call > robot.t_local:
            last_call = actual_time
            robot.gossip_step()


if __name__ == "__main__":
    # Initial params
    Num_robots = 6
    iterations = 100
    t_total = 20
    line_set = True
    # line_set = False
    bx = 0.5
    by = 2

    # Graph setup
    communication_graph = np.array([[5, 1], [0, 2], [1, 3], [2, 4],
                                    [3, 5], [4, 0], [1, 4], [5, 0]])
                                    # [3, 5], [4, 0]]) # 2 disconnected graphs 
    undirected = True
    alpha = 0.125
    my_graph = graph(Num_robots, communication_graph, undirected)
    my_graph.define_W_matrix(alpha)
    my_graph.eigenvals()
    # my_graph.plot_graph()

    # Robot declaration and neighbours setup
    robots = []
    for i in range(Num_robots):
        # Random initial positions
        x = random.randint(0, 10)
        y = random.randint(0, 10)
        robots.append(rbt(i, x, y, line = line_set))

    for i, j in communication_graph:
        robots[i].addNode(robots[j])
        robots[j].addNode(robots[i])


    # Print initial positions of robots
    print("Initial Positions of Robots:")
    for robot in robots:
        print(robot)


    # Interactive plot initialization
    plt.ion()   # interactive mode ON
    fig, ax = plt.subplots()

    x0 = [r.x for r in robots]
    y0 = [r.y for r in robots]
    init_scat = ax.scatter(x0, y0, marker='x',alpha=0.6)
    scat = ax.scatter(x0, y0)

    labels = []
    for r in robots:
        t = ax.text(r.x+0.1, r.y+0.1, f'{r.id}', fontsize=8)
        labels.append(t)

    ax.set_xlim(-5,15)
    ax.set_ylim(-5,15)
    ax.set_title("Gossip Consensus")
    plt.draw()
    plt.pause(0.1)

    # Trail with past steps
    history = {r.id: [(r.x, r.y)] for r in robots}
    lines = {}

    for r in robots:
        line, = ax.plot([],[],
                        linewidth=1,
                        alpha=0.2)
        lines[r.id] = line

    # Launch threads
    threads = []
    for robot in robots:
        t = threading.Thread(target=robot_thread, args = (robot,t_total))
        t.start()
        threads.append(t)


    # Consensus process
    for iteration in range(iterations):
        robot = random.choice(robots)

        if line_set: # To create a line
            robot.gossip_line(bx,by)
        else: # To meet at center
            robot.gossip_update()

        # Update interactive plot
        xs = [r.x for r in robots]
        ys = [r.y for r in robots]
        scat.set_offsets(list(zip(xs,ys)))

        for r,t in zip(robots,labels):
            history[r.id].append((r.x,r.y))
            t.set_position((r.x+0.1,r.y+0.1))
            hx = [p[0] for p in history[r.id]]
            hy = [p[1] for p in history[r.id]]
            lines[r.id].set_data(hx,hy)

        fig.canvas.draw_idle()
        plt.pause(0.05)

    # Join threads
    for t in threads:
        t.join()

    # Freeze last plot
    print("-----End-----")
    plt.ioff()
    plt.show()
