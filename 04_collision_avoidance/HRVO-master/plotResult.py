import math
import matplotlib.pyplot as plt

path = "result.txt"

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

plt.figure(figsize=(8, 8))
for i in range(n_agents):
    plt.plot(traj_x[i], traj_y[i], linewidth=0.6)

plt.xlabel("x")
plt.ylabel("y")
plt.title(f"Agent trajectories ({n_agents} agents)")
plt.axis("equal")
plt.grid(True)
plt.show()