from math import pi
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

M = 5
m = 1
l = 1
g = 10

Kp = -200
Ki = -5
Kd = -50

theta_target = 0.0

integral_error = 0.0
previous_error = 0.0

def pid_control(theta, theta_dot, t):
    global integral_error, previous_error

    error = theta - theta_target
    dt = 0.01

    integral_error += error * dt
    derivative_error = (error - previous_error) / dt
    previous_error = error

    F = Kp * error + Ki * integral_error + Kd * derivative_error
    return 0

def pendulum_cart(t, y):
    x, x_dot, theta, theta_dot = y

    sin_theta = np.sin(theta)
    cos_theta = np.cos(theta)
    denominator = M + m - m * cos_theta**2

    F = pid_control(theta, theta_dot, t)

    x_ddot = (F + m * g * sin_theta * cos_theta - m * l * theta_dot**2 * sin_theta) / denominator
    theta_ddot = ((F + m * g * sin_theta * cos_theta - m * l * theta_dot**2 * sin_theta) * cos_theta / denominator + g * sin_theta) / l

    return [x_dot, x_ddot, theta_dot, theta_ddot]

y0 = [0.0, 0.0, np.radians(5), 0.0]

t_span = (0, 10)
t_eval = np.linspace(*t_span, 1000)

solution = solve_ivp(pendulum_cart, t_span, y0, method='RK45', t_eval=t_eval)

x_vals = solution.y[0]
theta_vals = solution.y[2]
t_vals = solution.t

cart_width = 0.5
cart_height = 0.2
pole_length = l 

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-0.5, 1.5)

ax.plot([-10, 10], [0, 0], 'k--')

cart = Rectangle((0, 0), cart_width, cart_height, fc='blue')
ax.add_patch(cart)
pole, = ax.plot([], [], lw=3, c='black')

def animate(i):
    x = x_vals[i]
    theta = theta_vals[i]

    cart.set_xy((x - cart_width/2, 0))

    cart_center_x = x
    cart_top_y = cart_height
    pole_x = cart_center_x + pole_length * np.sin(theta)
    pole_y = cart_top_y + pole_length * np.cos(theta)

    pole.set_data([cart_center_x, pole_x], [cart_top_y, pole_y])

    return cart, pole

ani = FuncAnimation(fig, animate, frames=len(t_vals), interval=10, blit=True)
plt.title("Inverted Pendulum on a Cart (Simulation)")
plt.xlabel("Position")
plt.ylabel("Height")
plt.show()