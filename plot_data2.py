import csv
import matplotlib.pyplot as plt

files = {
    "grid 5": "density_data_5.csv",
    "grid 10": "density_data_10.csv",
    "grid 20": "density_data_20.csv",
    "grid 40": "density_data_40.csv"
}

# -------- Blocked agents plot --------
plt.figure()

for label, filename in files.items():
    steps = []
    blocked = []

    with open(filename, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            steps.append(int(row["step"]))
            blocked.append(int(row["blocked_agents"]))

    plt.plot(steps, blocked, label=label)

plt.xlabel("Time step")
plt.ylabel("Blocked agents")
plt.title("Blocked agents for different grid sizes (density effect)")
plt.legend()
plt.grid()
plt.savefig("density_blocked.png")
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
plt.title("Formation completion under different densities")
plt.legend()
plt.grid()
plt.savefig("density_completion.png")
plt.show()