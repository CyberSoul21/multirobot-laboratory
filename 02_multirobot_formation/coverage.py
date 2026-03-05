import numpy as np
import random
import math
from matplotlib import pyplot as plt
from matplotlib import cm

A = -0.5              # A < 0
B = 1.0
K = 1 # 3.2
T = 0.5               # sample time
total_t = 100
alpha = 0  # agents

# Domain
x_min, x_max = -5, 5
Nx = 200
x = np.linspace(x_min, x_max, Nx)
dx = x[1] - x[0]

# Time
N_ITER = int(total_t / T)
time = np.arange(N_ITER+1) * T

#3.4 more agents
n_agents = 5
p = np.linspace(-3, 3, n_agents) # Initial position
sigma = 0.8                      # Influence radius
Kv = 0.5                         # Velocity

# Coverage
Lambda = np.zeros((N_ITER+1, Nx))
# Lambda[0, :] = 200   # initial spatial condition 3.1
Lambda[0, :] = 0   # 3.3
F = np.exp(A * T)
G = (B/A) * (np.exp(A*T) - 1)

# 3.2
# Lambda_star = 50 * np.ones(Nx)  # 3.2 
Lambda_star = 80 * np.exp(-x**2 / 4) # 3.3
alpha_hist = np.zeros((N_ITER, Nx))

# 3.4
p_history = np.zeros((N_ITER+1, n_agents))
p_history[0, :] = p

for k in range(N_ITER):
    #3.4 Agent contribution
    alpha_k = np.zeros(Nx)
    for i in range(n_agents):
        kernel = np.exp(-(x - p[i])**2 / (2 * sigma**2))
        alpha_k += kernel
    Lambda[k+1, :] = F * Lambda[k, :] + G * alpha_k
    
    error_k = Lambda_star - Lambda[k+1, :]
    for i in range(n_agents):
        # Move to the position with more error
        phi_i = np.exp(-(x - p[i])**2 / (2 * sigma**2))
        d_phi = phi_i * (x - p[i]) / (sigma**2)
        # dp = Kv * np.sum(error_k * d_phi) * dx
        dp = (Kv/Nx) * np.sum(error_k * d_phi) * dx
        p[i] += dp
        p[i] = np.clip(p[i], x_min, x_max)
        
    p_history[k+1, :] = p
    # end 3.4

    # # 3.2
    # error_k = Lambda_star - Lambda[k, :]
    # alpha = K * error_k
    # alpha_hist[k, :] = alpha

    # # 3.1
    # Lambda[k+1, :] = F * Lambda[k, :] + G * alpha 

# #3.1
# # Spatial evolution
# plt.figure()
# # for k in range(0, N_ITER+1, 5):
# #     plt.plot(x, Lambda[k, :], label=f't={time[k]:.1f}')
# plt.plot(time, Lambda)
# plt.title("Coverage in domain")
# plt.xlabel("time")
# plt.ylabel("Lambda(x,t)")
# plt.legend()
# plt.show()

# 3.2
# 1. Evolución Temporal de la Cobertura
plt.figure(figsize=(10, 5))
plt.plot(time, Lambda[:, ::40]) # Graficamos solo algunos puntos espaciales para no saturar
plt.title("Evolución de la Cobertura con Control (3.2)")
plt.xlabel("Tiempo")
plt.ylabel("$\Lambda(x,t)$")
plt.legend(["Punto x1", "Punto x2", "Punto x3", "Punto x4", "Punto x5", "Objetivo"])
plt.grid(True)
plt.show()

# Cuadratic coverage error
# error = np.sum(Lambda**2, axis=1) * dx # 3.1
error = np.sum((Lambda - Lambda_star)**2, axis=1) * dx # 3.2

plt.figure()
plt.plot(time, error)
plt.title("Cuadratic coverage error")
plt.xlabel("time")
plt.ylabel("E(t)")
plt.show()


# 3.3
fig, ax = plt.subplots(subplot_kw={"projection": "3d"}, figsize=(12, 7))
X, Y = np.meshgrid(x, time)
surf = ax.plot_surface(X, Y, Lambda, cmap=cm.viridis, linewidth=0, antialiased=False)

ax.set_title("Evolución de Cobertura Espacial (3.3)")
ax.set_xlabel("Espacio (x)")
ax.set_ylabel("Tiempo (t)")
ax.set_zlabel("$\Lambda(x,t)$")
fig.colorbar(surf, shrink=0.5, aspect=5)
plt.show()

# --- Comparación Final ---
plt.figure(figsize=(10, 5))
plt.plot(x, Lambda_star, 'r--', label="Objetivo $\Lambda^*(x)$")
plt.plot(x, Lambda[-1, :], 'b', label="Estado Final $t=20$")
plt.fill_between(x, Lambda[-1, :], color='blue', alpha=0.2)
plt.title("Perfil de Cobertura Final vs Objetivo")
plt.xlabel("Espacio (x)")
plt.ylabel("Cobertura")
plt.legend()
plt.grid(True)
plt.show()


# 3.4
# 1. Trayectorias de los agentes sobre el error de cobertura
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.contourf(x, time, Lambda, cmap='viridis')
for i in range(n_agents):
    plt.plot(p_history[:, i], time, 'r', label=f'Agente {i+1}' if i==0 else "")
plt.title("Evolución de Cobertura y Trayectorias de Agentes (3.4)")
plt.ylabel("Tiempo")
plt.colorbar(label="Cobertura $\Lambda$")
plt.legend()

# 2. Perfil final vs Objetivo
plt.subplot(2, 1, 2)
plt.plot(x, Lambda_star, 'k--', label="Objetivo $\Lambda^*$")
plt.plot(x, Lambda[-1, :], 'b', label="Cobertura Final")
for pos in p:
    plt.axvline(x=pos, color='r', alpha=0.3, linestyle=':')
plt.title("Ajuste Final del Perfil")
plt.xlabel("Espacio (x)")
plt.legend()
plt.tight_layout()
plt.show()