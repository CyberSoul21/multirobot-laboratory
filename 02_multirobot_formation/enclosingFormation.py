import numpy as np
import random
import math
import matplotlib.pyplot as plt



def points_around(center, n: int, radius: float = 1.0, rotation: float = 0.0):

    cx = center[0,0]
    cy = center[1,0]
    rotation_rad = math.radians(rotation)
    c_around = np.zeros((2,n)) #Create matrix

    for i in range(n):
        c_around[0,i] = cx + radius * math.cos(2 * math.pi * i / n + rotation_rad)
        c_around[1,i] = cy + radius * math.sin(2 * math.pi * i / n + rotation_rad)

    return c_around





Num_robots = 10; # number of bots #square
#number iteration
t = 1000
Kc = 10
# Time step
dt = 0.1


robots = []
Q = np.zeros((2,Num_robots)) #Create matrix
C = np.zeros((2,Num_robots)) #Create matrix
target = np.array([[random.randint(0,10)],[random.randint(0,10)]])
d = 1

for i in range(Num_robots):
    # Random initial positions
    x = random.randint(0, 10)
    y = random.randint(0, 10)
    Q[0,i] = x
    Q[1,i] = y
    #robots.append(Robot(i, x, y))

    #c[0,i] =  random.randint(0, 10) 

#Desired formastion, TODO: create a class called shape
# Square (4 points), rotated 45° so sides are axis-aligned
C = points_around(target, n=Num_robots, radius=1.0, rotation=45) 


# Prepare to track the trajectories
trajectories = np.zeros((t, 2, Num_robots))  # For storing robot positions at each time step
print(trajectories)
#input()


# Plot initial positions (Q) - blue points
plt.figure(figsize=(8, 6))
plt.plot(Q[0], Q[1], 'go', label="Final positions")

for i in range(t):

    A = C @ Q.T
    #U,S,Vt = np.linalg.svd(A,full_matrices=True)
    U,S,Vt = np.linalg.svd(A)
    V = Vt.T
    VUt = V @ U.T
    d = np.sign(np.linalg.det(VUt))
    # Build D matrix
    D = np.eye(2)
    D[-1, -1] = d
    # Compute R
    R = V @ D @ U.T
    #R = np.dot(V,D,U.T)
    print("U: ")
    print(U)

    print("V: ")
    print(V)


    print("R: ")
    print(R)
    print("C: ")
    print(C)    
    print("Q: ")
    print(Q)    


    RC = R @ C
    print("RC: ")
    print(RC)  

    Q_RC =target + (Q-RC)
    print("Q_RC: ")
    print(Q_RC) 

    dq = Kc*(Q_RC-Q)


    #dq = Kc * (Q - np.dot(R,C))
    Q = Q + dq * dt
    # Print the updated positions (optional)
    print(f"Updated positions at time step {t}: {Q}")

    print("Q: ")
    print(Q) 

    #input()
    # Store the updated positions for plotting the trajectories
    trajectories[i] = Q  # Transpose Q to store robot positions at this time step




# Plot the results



# Plot desired positions (C) - red points
plt.plot(C[0], C[1], 'ro', label="Desired positions (C)")

# Plot initial positions (Q) - blue points
plt.plot(Q[0], Q[1], 'bx', label="Initial positions (Q)")

# Plot trajectories of robots (paths taken)
#for i in range(Num_robots):
#    plt.plot(trajectories[:, 0, i], trajectories[:, 1, i], 'g--', alpha=0.7)

# Add labels and title
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Robot Movement: Initial, Desired, and Final Positions with Trajectories")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()