import numpy as np
import matplotlib.pyplot as plt

Kp = 2.0
Ki = 0.1
Kd = 1.0

dt = 0.01
t = np.arange(0, 10, dt)
n = len(t)

setpoint = 1.0
x = 0.0
x_history = []
error_sum = 0.0
last_error = 0.0

for i in range(n):
    error = setpoint - x
    error_sum += error * dt
    d_error = (error - last_error) / dt

    u = Kp * error + Ki * error_sum + Kd * d_error

    dx = -x + u
    x += dx * dt

    x_history.append(x)
    last_error = error

plt.plot(t, x_history, label='System Output')
plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
plt.xlabel('Time')
plt.ylabel('x(t)')
plt.title('PID Control on First-Order System')
plt.legend()
plt.grid(True)
plt.show()