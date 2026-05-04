import csv
import matplotlib.pyplot as plt

# files = {
#     "10 agents": "scalability_data_10.csv",
#     "20 agents": "scalability_data_20.csv",
#     "30 agents": "scalability_data_30.csv",
#     "50 agents": "scalability_data_50.csv"
# }

files = {
    "50 agents": "scalability_data_50_osclt.csv"
}


# -------- J1 plot --------
#sum of distances of ALL agents to their goals
plt.figure()

for label, filename in files.items():
    steps = []
    J1 = []

    with open(filename, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row["step"]))
            J1.append(float(row["J1"]))

    plt.plot(steps, J1, label=label)

plt.xlabel("Time step")
plt.ylabel("J1")
plt.title("Convergence of J1 for different swarm sizes")
plt.legend()
plt.grid()
plt.savefig("J1_scalability.png")
plt.show()


# -------- Completed agents plot --------
plt.figure()

for label, filename in files.items():
    steps = []
    completed = []

    with open(filename, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row["step"]))
            completed.append(int(row["completed_agents"]))

    plt.plot(steps, completed, label=label)

plt.xlabel("Time step")
plt.ylabel("Completed agents")
plt.title("Formation completion over time")
plt.legend()
plt.grid()
plt.savefig("completion_scalability.png")
plt.show()