import matplotlib.pyplot as plt
import numpy as np

def dubins_car(x, y, yaw, v, steer, dt, L):
    # calculate the next state based on the Dubins car dynamic model
    x_next = x + v * np.cos(yaw) * dt
    y_next = y + v * np.sin(yaw) * dt
    yaw_next = yaw + v * np.tan(steer) / L * dt
    return x_next, y_next, yaw_next

def plot_dubins_car(x, y, yaw):
    # Plot the Dubins car as a triangle given its x, y, and yaw
    plt.plot(x, y, "ko")
    H = 0.1
    W = 0.05
    plt.plot([x + H * np.cos(yaw), x - W * np.sin(yaw), x - W * np.sin(yaw) + H * np.cos(yaw)],
             [y + H * np.sin(yaw), y + W * np.cos(yaw), y - W * np.cos(yaw) + H * np.sin(yaw)], "k-")

# Initial conditions
x = 0.0
y = 0.0
yaw = np.deg2rad(30.0)
v = 1.0
steer = np.deg2rad(30.0)
dt = 0.1
L = 1.0

# Simulate the Dubins car for N time steps
N = 200
x_hist = [x]
y_hist = [y]
yaw_hist = [yaw]
for i in range(N):
    x, y, yaw = dubins_car(x, y, yaw, v, steer, dt, L)
    x_hist.append(x)
    y_hist.append(y)
    yaw_hist.append(yaw)

# Plot the results
plt.axis("equal")
for x_, y_, yaw_ in zip(x_hist, y_hist, yaw_hist):
    plot_dubins_car(x_, y_, yaw_)
plt.show()
