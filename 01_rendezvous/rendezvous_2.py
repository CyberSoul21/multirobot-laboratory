from Robot_2 import Robot as rbt
from graph import graph
import matplotlib.pyplot as plt
import numpy as np
import random


# Number of robots
N = 6

#TODO: Create two topologies to test
# Adjacency List for a Ring Topology (Cyclic Communication Graph)
# Robot ID: [List of Neighbor IDs]

# communication_graph = [
#     (5, 1),
#     (0, 2),
#     (1, 3),
#     (2, 4),
#     (3, 5),
#     (4, 0),
#     (1, 4),
#     (5, 0)
# ]

line_set = True
# line_set = False
bx = 0.5
by = 2

communication_graph = np.array([[5, 1], [0, 2], [1, 3], [2, 4],
                                [3, 5], [4, 0], [1, 4], [5, 0]])
                                # [3, 5], [4, 0]]) # 2 disconnected graphs 
undirected = True
alpha = 0.125

my_graph = graph(N, communication_graph, undirected)
my_graph.define_W_matrix(alpha)
my_graph.eigenvals()
# my_graph.plot_graph()

#TODO: put inside of one method
# Initialize robots with random positions and neighbors based on the communication graph
robots = []
for i in range(N):
    # Random initial positions
    x = random.randint(0, 10)
    y = random.randint(0, 10)
    robots.append(rbt(i, x, y))

for i, j in communication_graph:
    robots[i].addNode(robots[j])
    robots[j].addNode(robots[i])

# for i in range(N):
#     # Create the robot and assign neighbors based on the graph
#     neighbors = [robots[neighbor_id] for neighbor_id in communication_graph[i]]
#     robots[i].setNeighbors(neighbors)

# Print initial positions of robots
#TODO: Plot
print("Initial Positions of Robots:")
for robot in robots:
    print(robot)


#Simulation
#TODO: Check with threads, and topics, can we implement in ROS?
#--------------------------------------------------------------------
# Run the simulation for a few iterations

plt.ion()   # interactive mode ON

fig, ax = plt.subplots()

# posiciones iniciales
xs = [r.x for r in robots]
ys = [r.y for r in robots]

# ---------------- INITIAL POSITIONS ----------------
x0 = [r.x for r in robots]
y0 = [r.y for r in robots]

#initial position
init_scat = ax.scatter(x0, y0,
                       marker='x',
                       alpha=0.6)

labels = []
for r in robots:
    t = ax.text(r.x+0.1,
                r.y+0.1,
                f'{r.id}',
                fontsize=8)
    labels.append(t)


scat = ax.scatter(xs, ys)

ax.set_xlim(-5,15)
ax.set_ylim(-5,15)
ax.set_title("Gossip Consensus")

plt.draw()
plt.pause(0.1)

iterations = 100

# trail
history = {r.id: [(r.x, r.y)] for r in robots}
lines = {}

for r in robots:
    line, = ax.plot([],[],
                    linewidth=1,
                    alpha=0.2)
    lines[r.id] = line


for iteration in range(iterations):
    # print(f"\nIteration {iteration + 1} - Robots Updating Positions:")
    
    robot = random.choice(robots)

    if line_set:
        robot.gossip_line(bx,by)
    else:
        robot.gossip_update()


    xs = [r.x for r in robots]
    ys = [r.y for r in robots]

    scat.set_offsets(list(zip(xs,ys)))

    # for r in robots:
    for r,t in zip(robots,labels):
        history[r.id].append((r.x,r.y))
        t.set_position((r.x+0.1,r.y+0.1))
        hx = [p[0] for p in history[r.id]]
        hy = [p[1] for p in history[r.id]]
        lines[r.id].set_data(hx,hy)


    fig.canvas.draw_idle()
    plt.pause(0.05)
    
    # Call gossip_update() for one robot robot
    # for robot in robots:
        # robot.gossip_update()

    # # Print the updated positions after this iteration
    # for robot in robots:
    #     print(robot)

print("-----End-----")
plt.ioff()
plt.show()
