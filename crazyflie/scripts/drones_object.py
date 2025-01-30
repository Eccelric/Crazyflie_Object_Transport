import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Define the time vector
t = np.linspace(0, 10, 1000)

# Define drone trajectories (for simplicity, let's assume some predefined paths)
drone1_path = np.array([np.sin(t), np.cos(t), 1.5 + 0.5*np.sin(t/2)]).T
drone2_path = np.array([np.sin(t + np.pi/2), np.cos(t + np.pi/2), 1.5 + 0.5*np.sin(t/2)]).T
drone3_path = np.array([np.sin(t + np.pi), np.cos(t + np.pi), 1.5 + 0.5*np.sin(t/2)]).T

# Function to compute the load position
def compute_load_position(drone1, drone2, drone3):
    # Solve for the load position assuming the cables form a triangle and the load is directly beneath the centroid.
    load_position = (drone1 + drone2 + drone3) / 3
    load_position[2] -= 0.5  # Adjust z-axis for suspension below drones
    return load_position

# 3D Animation Setup
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
ax.set_zlim(0, 2)

# Initialize the plots
drone_plots = [ax.plot([], [], [], 'bo')[0] for _ in range(3)]
load_plot, = ax.plot([], [], [], 'ro')
cable_lines = [ax.plot([], [], [], 'k-')[0] for _ in range(3)]

# Update function for animation
def update(num):
    # Update drones
    drone1 = drone1_path[num]
    drone2 = drone2_path[num]
    drone3 = drone3_path[num]
    
    drone_plots[0].set_data(drone1[0], drone1[1])
    drone_plots[0].set_3d_properties(drone1[2])
    
    drone_plots[1].set_data(drone2[0], drone2[1])
    drone_plots[1].set_3d_properties(drone2[2])
    
    drone_plots[2].set_data(drone3[0], drone3[1])
    drone_plots[2].set_3d_properties(drone3[2])
    
    # Compute load position
    load_pos = compute_load_position(drone1, drone2, drone3)
    
    # Update load
    load_plot.set_data(load_pos[0], load_pos[1])
    load_plot.set_3d_properties(load_pos[2])
    
    # Update cables
    cable_lines[0].set_data([drone1[0], load_pos[0]], [drone1[1], load_pos[1]])
    cable_lines[0].set_3d_properties([drone1[2], load_pos[2]])
    
    cable_lines[1].set_data([drone2[0], load_pos[0]], [drone2[1], load_pos[1]])
    cable_lines[1].set_3d_properties([drone2[2], load_pos[2]])
    
    cable_lines[2].set_data([drone3[0], load_pos[0]], [drone3[1], load_pos[1]])
    cable_lines[2].set_3d_properties([drone3[2], load_pos[2]])
    
    return drone_plots + [load_plot] + cable_lines

# Create animation
ani = FuncAnimation(fig, update, frames=len(t), interval=20, blit=False)

plt.show()
