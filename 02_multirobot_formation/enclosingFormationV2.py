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

Num_robots = 10
t_steps = 200 #for testing
Kc = 5
dt = 0.05
target = np.array([[random.randint(0,10)],[random.randint(0,10)]]) #np.array([[5.0], [5.0]])

# Initial positions Q (2 x N)
Q = np.random.rand(2, Num_robots) * 10 
# Reference formation C (2 x N) centered at zero for rotation math
C = points_around(np.array([[0],[0]]), n=Num_robots, radius=1.0, rotation=45)

trajectories = np.zeros((t_steps+1, 2, Num_robots))


# --- PLOTTING ---
plt.figure(figsize=(8, 8))
# Plot initial positions (Q) - blue points
plt.plot(Q[0], Q[1], 'go', label="Initial positions")

trajectories[0] = Q


for k in range(t_steps):


    # --- KABSCH ALGORITHM STEP ------------------
    # Centering (Crucial for SVD rotation)
    #Centering: The Kabsch algorithm finds the rotation between two sets of points. 
    # For this to work, we must subtract the centroid (mean) from both Q and C before computing the SVD.
    centroid_Q = np.mean(Q, axis=1, keepdims=True)
    centroid_C = np.mean(C, axis=1, keepdims=True)
    
    Q_centered = Q - centroid_Q
    C_centered = C - centroid_C
    #-----------------------------------------------


    print("Q_centered: ")
    print(Q_centered)

    print("C_centered: ")
    print(C_centered)

    A = C_centered @ Q_centered.T
    
    U, S, Vt = np.linalg.svd(A)
    V = Vt.T
    
    d = np.linalg.det(V @ U.T)
    # R must be 2x2 for 2d robotics
    D = np.eye(2)
    D[1, 1] = np.sign(d)
    
    # Compute Rotation Matrix
    R = V @ D @ U.T
    
    # CONTROL LAW STEP 
    # q_Ni = target + R @ C_i
    Q_target = target + (R @ C)
    
    # Error vector 
    error = Q_target - Q
    
    # Update positions
    dq = Kc * error
    Q = Q + dq * dt
    
    trajectories[k+1] = Q



plt.plot(C[0,:] + target[0], C[1,:] + target[1], 'ro', label="Target Formation", alpha=0.3)
for i in range(Num_robots):
    plt.plot(trajectories[:, 0, i], trajectories[:, 1, i], 'g-', alpha=0.4)
plt.scatter(Q[0], Q[1], c='blue', marker='x', label="Final Position")
plt.legend()
plt.axis('equal')

# Add labels and title
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Robot Movement: Initial, Desired, and Final Positions with Trajectories")
plt.legend()
plt.grid(True)

plt.show()