#-----------------------------------
# Javier Almario, NIP: 962449
# Alvaro Provencio, NIP: 960625
#-----------------------------------

import numpy as np
import random
import math
import matplotlib.pyplot as plt


#Polygon
def points_around(center, n: int, radius: float = 1.0, rotation: float = 0.0):

    cx = center[0,0]
    cy = center[1,0]
    rotation_rad = math.radians(rotation)
    c_around = np.zeros((2,n)) #Create matrix

    for i in range(n):
        c_around[0,i] = cx + radius * math.cos(2 * math.pi * i / n + rotation_rad)
        c_around[1,i] = cy + radius * math.sin(2 * math.pi * i / n + rotation_rad)

    return c_around

def robots_on_regular_polygon(center, n_robots: int, n_sides: int, radius: float = 1.0, rotation: float = 0.0):

    cx, cy = center[0, 0], center[1, 0]
    theta0 = math.radians(rotation)

    vertices = np.zeros((2, n_sides))
    for i in range(n_sides):
        theta = 2 * math.pi * i / n_sides + theta0
        vertices[0, i] = cx + radius * math.cos(theta)
        vertices[1, i] = cy + radius * math.sin(theta)

    side_lengths = np.zeros(n_sides)
    for i in range(n_sides):
        next_i = (i + 1) % n_sides
        side_lengths[i] = np.linalg.norm(vertices[:, next_i] - vertices[:, i])

    perimeter = np.sum(side_lengths)
    step = perimeter / n_robots 

    robots = np.zeros((2, n_robots))
    edge_i = 0
    distance_on_edge = 0.0

    for r in range(n_robots):
        remaining = step
        while remaining > 0:
            next_i = (edge_i + 1) % n_sides
            edge_vec = vertices[:, next_i] - vertices[:, edge_i]
            edge_len = side_lengths[edge_i]

            if distance_on_edge + remaining <= edge_len:
                t = (distance_on_edge + remaining) / edge_len
                robots[:, r] = vertices[:, edge_i] + t * edge_vec
                distance_on_edge += remaining
                remaining = 0
            else:
                remaining -= (edge_len - distance_on_edge)
                edge_i = (edge_i + 1) % n_sides
                distance_on_edge = 0.0

    return robots

Num_robots = 12
N = Num_robots + 1 # Robot + target
t_steps = 1000 # for testing
Kc = 1
dt = 0.05
target = np.array([[random.randint(0,10)],[random.randint(0,10)]])

# Initial positions Q (2 x N)
q_robots = np.random.rand(2, Num_robots) * 10 
q = np.hstack((q_robots,target))

# Reference formation C (2 x N) centered at zero for rotation math
# c_robots = points_around(np.array([[0],[0]]), n=Num_robots, radius=1.0, rotation=45) #Circle
c_robots = robots_on_regular_polygon(np.array([[0],[0]]), n_robots=Num_robots, n_sides=6,radius=1.0, rotation=45) #Polygon
c = np.hstack((c_robots,np.array([[0],[0]])))

# Compute Q and C with N+N columns
Q = np.zeros((2,N*N))
C = np.zeros((2,N*N))
idx = 0
for i in range(N):
    for j in range(N):
        Q[:,idx] = q[:,i]-q[:,j]
        C[:,idx] = c[:,i]-c[:,j]
        idx += 1

# Compute Rotation Matrix
A = C @ Q.T
U, S, Vt = np.linalg.svd(A)
V = Vt.T
d = np.linalg.det(V @ U.T)
D = np.eye(2) # R must be 2x2 for 2d robotics
D[1, 1] = np.sign(d)
R = V @ D @ U.T

# Compute enclosing
trajectories = np.zeros((t_steps,2,Num_robots))
target_idx = N - 1
for k in range(t_steps):
    for i in range(Num_robots):
        trajectories[k, :, i] = q[:, i]
        q_Ni = q[:, target_idx] - q[:, i]
        c_Ni = c[:, target_idx] - c[:, i]
        dq_i = Kc * (q_Ni - R @ c_Ni)
        q[:, i] += dq_i * dt
        

# --- PLOTTING ---
plt.figure(figsize=(8, 8))
plt.scatter(q_robots[0], q_robots[1], color='green', label="Initial positions", alpha=0.5)
plt.plot(target[0], target[1], 'rx', markersize=12, markeredgewidth=3, label="Target")

# Trajectories
for i in range(Num_robots):
    plt.plot(trajectories[:, 0, i], trajectories[:, 1, i], 'g-', alpha=0.3)


plt.scatter(q[0, :Num_robots], q[1, :Num_robots], c='blue', marker='o', label="Final Position")
c_rotated = R @ c_robots + target
plt.scatter(c_rotated[0], c_rotated[1], facecolors='none', edgecolors='r', s=100, label="Desired position")

plt.axis('equal')
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Enclosing")
plt.legend()
plt.grid(True)
plt.show()
