import math
import numpy as np
import matplotlib.pyplot as plt

# Define system parameters
L = 3  # length of the car
MAX_STEER = math.pi / 3  # max steering angle
MAX_SPEED = 2  # max speed in m/s

def simulate(initial_state, desired_state):
    # Define the trajectory
    dt = 0.1  # time step
    # Calculate the number of timesteps based on the distance between the initial and desired states and the maximum speed
    timesteps = int(np.ceil(np.linalg.norm(desired_state[:2] - initial_state[:2]) / MAX_SPEED / dt))
    # Generate arrays of x, y, and theta values for each timestep along the trajectory
    xs = np.linspace(initial_state[0], desired_state[0], timesteps)
    ys = np.linspace(initial_state[1], desired_state[1], timesteps)
    thetas = np.linspace(initial_state[2], desired_state[2], timesteps)

    # Initialize the state to the initial state
    state = initial_state.copy()

    # Simulate the system
    for i in range(timesteps):
        # Compute the control inputs for the current timestep
        delta = compute_steering_control(state, xs[i], ys[i], thetas[i])  # steering angle
        v = MAX_SPEED  # constant speed

        # Update the state using the control inputs and time step
        state = update_state(state, v, delta, dt)

        # Plot the state of the car and the trajectory so far
        plt.cla()  # clear the previous plot
        plot_car(state)  # plot the car
        plt.plot(xs[:i+1], ys[:i+1], 'g--')  # plot the trajectory so far
        plt.plot(xs[i], ys[i], 'go')  # plot the current position of the car on the trajectory
        plt.plot(desired_state[0], desired_state[1], 'ro')  # plot the desired state as a red dot
        plt.axis('equal')  # make the x and y axis scale equal
        plt.pause(0.001)  # pause for a short time to allow the plot to update

    # Show the final plot
    plt.show()

def update_state(state, v, delta, dt):
    # Extract the x, y, and theta values from the state array
    x, y, theta = state
    # Update the x and y positions using the current velocity and steering angle
    x = x + v * math.cos(theta) * dt
    y = y + v * math.sin(theta) * dt
    # Update the theta value using the current velocity, steering angle, and car length
    theta = theta + v / L * math.tan(delta) * dt
    # Combine the updated values into a new state array and return it
    return np.array([x, y, theta])

def compute_steering_control(state, xd, yd, thetad):
    # Extract the x, y, and theta values from the state array
    x, y, theta = state
    # Calculate the desired steering angle to reach the desired state
    beta = math.atan2(yd - y, xd - x)
    delta = beta - theta
    # Limit the steering angle to the maximum allowed value
    delta = max(-MAX_STEER, min(delta, MAX_STEER))
    # Return the computed steering angle
    return delta

def plot_car(state):
    # Extract the x, y,
    x, y, theta = state
    outline = np.array([[-1.5, 1.5, 1.5, -1.5, -1.5],
                        [0.8, 0.8, -0.8, -0.8, 0.8]])
    R = np.array([[math.cos(theta), math.sin(theta)],
                  [-math.sin(theta), math.cos(theta)]])
    outline = np.dot(R, outline)
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(outline[0, :], outline[1, :], 'k')

# Define the initial and desired states
initial_state = np.array([0, 0, 0])
desired_state = np.array([20, 20, math.pi/4])

# Simulate the system
simulate(initial_state, desired_state)

