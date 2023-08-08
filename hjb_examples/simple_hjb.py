import numpy as np
import matplotlib.pyplot as plt

# Pendulum dynamics
def dynamics(theta, theta_dot, u):
    g = 9.81  # Gravity
    m = 1.0   # Mass
    l = 1.0   # Length

    theta_double_dot = (-g / l) * np.sin(theta) + (u / (m * l ** 2))
    return theta_double_dot

# Value iteration for HJB equation
def value_iteration(N_theta, N_theta_dot, N_u, theta_vals, theta_dot_vals, u_vals, delta_t, discount_factor):
    v = np.zeros((N_theta, N_theta_dot))
    policy = np.zeros((N_theta, N_theta_dot), dtype=int)

    for _ in range(N_u):
        for i in range(N_theta):
            for j in range(N_theta_dot):
                theta = theta_vals[i]
                theta_dot = theta_dot_vals[j]

                min_cost = float('inf')
                optimal_u = None

                for u_idx, u in enumerate(u_vals):
                    theta_next = theta + delta_t * theta_dot
                    theta_dot_next = theta_dot + delta_t * dynamics(theta, theta_dot, u)
                    
                    # Discretize theta_next and theta_dot_next to find the closest grid points
                    i_next = np.argmin(np.abs(theta_next - theta_vals))
                    j_next = np.argmin(np.abs(theta_dot_next - theta_dot_vals))

                    cost = delta_t + discount_factor * v[i_next, j_next]
                    if cost < min_cost:
                        min_cost = cost
                        optimal_u = u_idx

                v[i, j] = min_cost
                policy[i, j] = optimal_u

    return v, policy

# Parameters
N_theta = 51
N_theta_dot = 51
N_u = 21
theta_vals = np.linspace(-np.pi, np.pi, N_theta)
theta_dot_vals = np.linspace(-10, 10, N_theta_dot)
u_vals = np.linspace(-2, 2, N_u)
delta_t = 0.1
discount_factor = 0.9

# Solve HJB equation using value iteration
value_function, optimal_policy = value_iteration(N_theta, N_theta_dot, N_u, theta_vals, theta_dot_vals, u_vals, delta_t, discount_factor)

# Plot the value function and optimal policy
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.imshow(value_function.T, origin='lower', extent=[-np.pi, np.pi, -10, 10], aspect='auto')
plt.colorbar(label='Value function')
plt.xlabel('Theta')
plt.ylabel('Theta Dot')
plt.title('Value Function')

plt.subplot(1, 2, 2)
plt.imshow(optimal_policy.T, origin='lower', extent=[-np.pi, np.pi, -10, 10], aspect='auto', cmap='coolwarm')
plt.colorbar(label='Optimal control (u)')
plt.xlabel('Theta')
plt.ylabel('Theta Dot')
plt.title('Optimal Policy')

plt.tight_layout()
plt.show()
