from math import pi
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

M = 5
m = 1
l = 1
g =10

def F(t):
    return 1.0

def pendulum_cart(t, y):
    x, x_dot, theta, theta_dot = y
    
    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    denominator = M + m - m * cos_theta**2
    
    # Acceleration equations from your image
    x_ddot = (F(t) + m * g * sin_theta * cos_theta - m * l * theta_dot**2 * sin_theta) / denominator
    theta_ddot = ((F(t) + m * g * sin_theta * cos_theta - m * l * theta_dot**2 * sin_theta) * cos_theta / denominator + g * sin_theta) / l
    
    return [x_dot, x_ddot, theta_dot, theta_ddot]

y0 = [0.0, 0.0, np.radians(180), 0.0]

t_span = (0, 10)
t_eval = np.linspace(*t_span, 1000)

solution = solve_ivp(pendulum_cart, t_span, y0, method='RK45', t_eval=t_eval)

plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(solution.t, solution.y[0])
plt.title('Cart Position x(t)')
plt.xlabel('Time (s)')
plt.ylabel('x (m)')
plt.grid()

plt.subplot(1, 2, 2)
plt.plot(solution.t, np.degrees(solution.y[2]))
plt.title('Pendulum Angle θ(t)')
plt.xlabel('Time (s)')
plt.ylabel('θ (degrees)')
plt.grid()

plt.tight_layout()
plt.show()
