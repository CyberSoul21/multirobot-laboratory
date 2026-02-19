import Robot as rbt
import random


# Number of robots
N = 6

#TODO: ask or set a point to converge
#TODO: Implement Tlocal
#TODO: check request and update of gossip_update.
#TODO: plot: implemente queue_position_plot, see the document
#TODO: Ask about this requirement: "...in which the robot will
#randomly select a neighbor, will request a gossip update, and will update its state."

#TODO: Create two topologies to test
# Adjacency List for a Ring Topology (Cyclic Communication Graph)
# Robot ID: [List of Neighbor IDs]
communication_graph = {
    0: [5, 1],
    1: [0, 2],
    2: [1, 3],
    3: [2, 4],
    4: [3, 5],
    5: [4, 0]
}

#TODO: put inside of one method
# Initialize robots with random positions and neighbors based on the communication graph
robots = []
for i in range(N):
    # Random initial positions
    x = random.randint(0, 10)
    y = random.randint(0, 10)
    # Create the robot and assign neighbors based on the graph
    neighbors = [robots[neighbor_id] for neighbor_id in communication_graph[i]]
    robot = rbt(i, x, y, neighbors)
    robots.append(robot)

# Print initial positions of robots
#TODO: Plot
print("Initial Positions of Robots:")
for robot in robots:
    print(robot)


#Simulation
#TODO: Check with threads, and topics, can we implement in ROS?
#--------------------------------------------------------------------
# Run the simulation for a few iterations
iterations = 5
for iteration in range(iterations):
    print(f"\nIteration {iteration + 1} - Robots Updating Positions:")
    
    # Call gossip_update() for each robot
    for robot in robots:
        robot.gossip_update()
    
    # Print the updated positions after this iteration
    for robot in robots:
        print(robot)