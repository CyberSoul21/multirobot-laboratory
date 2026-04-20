#-----------------------------------
# Javier Almario, NIP: 962449
# Alvaro Provencio, NIP: 960625
#-----------------------------------

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
A = -0.25           # A < 0           #Environmental Decay
B = 1.0             # B > 0           #Robot Effectiveness ($B$): 
T = 0.25            # sample time     # The time step for each calculation (0.25 seconds).
total_t = 100       # Total time
F = np.exp(A * T)                     #Discrete Evolution (F and G): 
G = (B/A) * (np.exp(A*T) - 1)         #These are the exact transition matrices required to update the state in a computer simulation.
# 3.3 params
K = 2       # coverture intensity
Kv = 0.05   # agent velocity
R = 1.2     # Influence radius

# Time
N_ITER = int(total_t / T)
time = np.arange(N_ITER+1) * T

# Domain
# A 2D grid of $100 \times 100$ points (Nx, Ny) representing the area to be covered.
x_min, x_max = -10, 10
y_min, y_max = -10, 10
Nx, Ny = 100, 100
x = np.linspace(x_min, x_max, Nx)
y = np.linspace(y_min, y_max, Ny)
X, Y = np.meshgrid(x, y) 
dA = (x[1] - x[0]) * (y[1] - y[0]) 


#--------------------- 3.1 Environment ---------------------#
alpha = 0           # agents
Lambda = np.zeros((N_ITER+1, Nx, Ny))
#Desired Coverage ($\Lambda^*$): Though the snippet is partial, the theory suggests a target level Lambda_star that the robots try to maintain.
Lambda_star = 2 * np.ones((Nx, Ny))
Lambda[0, :, :] = 2   # initial spatial condition

#The Coverage Update (The "Environment" Logic)
#The core of the simulation loop implements the discrete equation :
for k in range(N_ITER):
    Lambda[k+1, : ,:] = F * Lambda[k, :, :] + G * alpha

#This line updates every single point on the map. If alpha (the robot's action) is 0, 
# the map simply decays based on $F$. If a robot is present, 
# its "footprint" ($\sigma$) is added through the alpha term

error = np.sum((Lambda_star - Lambda)**2, axis=(1, 2)) * dA

# Coverage decay over time for a point
plt.figure()
plt.plot(time, Lambda[:, 0,0]) #plot some points
plt.title("3.1 Coverage decay over time for a point")
plt.xlabel("time")
plt.ylabel("Lambda(x,t)")
plt.show()

# Cuadratic coverage error
plt.figure()
plt.plot(time, error)
plt.title("3.1 Cuadratic coverage error of the domain")
plt.xlabel("time")
plt.ylabel("E(t)")
plt.show()


#--------------------- 3.2 A first agent ---------------------#
n_agents = 1
K = 1     #coverture intensity
R = 0.8   # Influence radius
p = np.array([0.0, 0.0])

Lambda = np.zeros((N_ITER+1, Nx, Ny))
Lambda_star = 2 * K * np.ones((Nx, Ny))

for k in range(N_ITER):
    sigma = np.exp(-((X - p[0])**2 + (Y - p[1])**2) / (2 * R**2))
    alpha = K * sigma
    Lambda[k+1, :] = F * Lambda[k, :] + G * alpha

error = np.sum((Lambda_star - Lambda)**2, axis=(1, 2)) * dA

# Final status of the grid
plt.pcolormesh(X, Y, Lambda[-1, :, :], cmap='viridis', shading='auto')
plt.colorbar(label="Lambda(x,y)")
plt.title(f"3.2 Coverture (t={total_t})")
plt.xlabel("x")
plt.ylabel("y")
plt.show()

# Evolution of Lambda at certain points
plt.figure()
plt.plot(time, Lambda[:, Ny//2, Nx//2], label="Center (under the agent)")
plt.plot(time, Lambda[:, (Ny//2 + 5), (Nx//2 + 5)], label="close to the agent")
plt.plot(time, Lambda[:, 0, 0], label="Corner (far from the agent)")
plt.title("3.2 Evolution of Lambda at certain points")
plt.xlabel("Time")
plt.ylabel("Lambda(x,y)")
plt.legend()
plt.show()

# Cuadratic coverage error
plt.figure()
plt.plot(time, error)
plt.title("3.2 Cuadratic coverage error")
plt.xlabel("time")
plt.ylabel("E(t)")
plt.show()


#--------------------- 3.3 Static path planning ---------------------#
n_agents = 5
p = np.random.uniform(x_min, x_max, size=(n_agents, 2))
p_history = np.zeros((N_ITER+1, n_agents, 2))
p_history[0] = p
dx = x[1] - x[0] 
dy = y[1] - y[0] 

Lambda = np.zeros((N_ITER+1, Nx, Ny))
Lambda_star = 2 * K * np.ones((Nx, Ny))


#Motion Control (The "Robot" Logic)
# The code implements a Gradient-Based Control Law to move the agents:
# -Influence Radius (R): Defines the robot's sensor/actuator range.
# -Error Calculation: The robot calculates the difference between where the map is and 
# -where it should be: error_k = Lambda_star - Lambda.

for k in range(N_ITER):
    # Compute alpha for each agent
    alpha = np.zeros((Ny, Nx))
    for i in range(n_agents):
        sigma = np.exp(-((X - p[i,0])**2 + (Y - p[i,1])**2) / (2 * R**2))
        alpha += K * sigma
    
    # Update lambda
    Lambda[k+1, :] = F * Lambda[k, :] + G * alpha

    # Move agents
    error_k =  Lambda_star - Lambda[k, :, :]
    for i in range(n_agents):
        # Move to the position with more error
        dist_x = X - p[i, 0]
        dist_y = Y - p[i, 1]
        phi_i = np.exp(-(dist_x**2 + dist_y**2) / (2 * R**2))
        d_phi_x = phi_i * dist_x / (R**2)
        d_phi_y = phi_i * dist_y / (R**2)
        
        #This line tells the robot to move in the direction of the highest error .
        dp_x = Kv * np.sum(error_k * d_phi_x) * dx
        dp_y = Kv * np.sum(error_k * d_phi_y) * dy
        #------------------------------------------------------------------------
        p[i, 0] += dp_x * T
        p[i, 1] += dp_y * T
        p[i, 0] = np.clip(p[i, 0], x_min, x_max)
        p[i, 1] = np.clip(p[i, 1], y_min, y_max)
    p_history[k+1] = p

error = np.sum((Lambda_star - Lambda)**2, axis=(1, 2)) * dA

#Integrator Dynamics: The robot position p is updated by adding a velocity dp times time T (p[i, 0] += dp_x * T), confirming the $\dot{p} = u$ model.
# Quadratic Error: At the end, the code calculates the performance metric mentioned in the lab: error = np.sum((Lambda_star - Lambda)**2 ...). This total "cost" tells us how well the multi-robot team is performing its task.
# Saturation/Clipping: The code uses np.clip to ensure robots stay within the map boundaries (x_min, x_max), which is a practical constraint for any real-world MRS.


# Agent movement
fig, ax = plt.subplots(figsize=(8, 6))
im = ax.pcolormesh(X, Y, Lambda[0, :, :], cmap='viridis', shading='auto', vmin=0, vmax=Lambda_star.max()*1.2)
plt.colorbar(im, label="Lambda level")

agent_plots = []
for i in range(n_agents):
    line, = ax.plot([], [], 'r-', alpha=0.3)
    point, = ax.plot([], [], 'ro', markersize=8, label=f"Agent {i+1}")
    agent_plots.append((line, point))

ax.set_title(f"3.3 Coverage with {n_agents} agents")
ax.set_xlabel("X")
ax.set_ylabel("Y")

def update(k):
    """Update frame"""
    im.set_array(Lambda[k, :, :].ravel())
    for i in range(n_agents):
        _ , point = agent_plots[i]
        point.set_data([p_history[k, i, 0]], [p_history[k, i, 1]])
    return [im] + [p for plots in agent_plots for p in plots]

ani = FuncAnimation(fig, update, frames=range(0, N_ITER, 2), interval=50, blit=True)
plt.legend()
plt.show()

# Cuadratic coverage error
plt.figure()
plt.plot(time, error)
plt.title("3.3 Cuadratic coverage error of the domain")
plt.xlabel("time")
plt.ylabel("E(t)")
plt.show()