#-----------------------------------
# Javier Almario, NIP: 962449
# Alvaro Provencio, NIP: 960625
#-----------------------------------

import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

#path = "resultCircle.txt"
#path = "result.txt"
path = "resultSquare.txt"

times = []
rows = []

with open(path, "r") as f:
    for line in f:
        parts = line.strip().split()
        if not parts:
            continue
        nums = [float(x) for x in parts]
        times.append(nums[0])
        rows.append(nums[1:])

n_agents = len(rows[0]) // 2

traj_x = [[] for _ in range(n_agents)]
traj_y = [[] for _ in range(n_agents)]

for row in rows:
    for i in range(n_agents):
        traj_x[i].append(row[2*i])
        traj_y[i].append(row[2*i + 1])

fig, ax = plt.subplots(figsize=(8, 8))
lines = []
points = []
markersize = 3.0 #1.5 # Set same markersize as agent radius

# Initialization
for i in range(n_agents):
    line, = ax.plot([], [], linewidth=0.6)
    point, = ax.plot([], [], 'o', markersize=markersize)    
    color = line.get_color()
    point.set_color(color)
    
    lines.append(line)
    points.append(point)


ax.set_xlim(min(map(min, traj_x)), max(map(max, traj_x)))
ax.set_ylim(min(map(min, traj_y)), max(map(max, traj_y)))

# Update function
current_x = [[] for _ in range(n_agents)]
current_y = [[] for _ in range(n_agents)]

def update(frame):
    for i in range(n_agents):
        current_x[i].append(traj_x[i][frame])
        current_y[i].append(traj_y[i][frame])

        lines[i].set_data(current_x[i], current_y[i])
        points[i].set_data([traj_x[i][frame]], [traj_y[i][frame]])
    
    return lines + points

# Animation
anim = FuncAnimation(
    fig,
    update,
    frames=len(traj_x[0]),
    interval=1,  # ms entre frames, modify for visualization
    blit=True,
    repeat=False
)

plt.show()