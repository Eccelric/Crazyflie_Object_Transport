import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
class DroneObjectSystem:
    def __init__(self, x, y, z,yaw, theta1_1, theta2_1, theta1_2, theta2_2, theta1_3, theta2_3, length, side):
        # Initialize the position of the system
        self.x = x
        self.y = y
        self.z = z
        self.yaw=yaw
        # Initialize the angles for each tuple
        self.theta1_1 = theta1_1
        self.theta2_1 = theta2_1
        self.theta1_2 = theta1_2
        self.theta2_2 = theta2_2
        self.theta1_3 = theta1_3
        self.theta2_3 = theta2_3
        self.length = length
        self.side = side
    def rotation_matrix_vertex(self,theta1,theta2):
        rot_x = np.array([
                          [1,0,0],
                          [0,np.cos(theta1),-np.sin(theta1)],
                          [0,np.sin(theta1),np.cos(theta1)]])
        rot_y = np.array([[np.cos(theta2),0,np.sin(theta2)],[0,1,0],[-np.sin(theta2),0,np.cos(theta2)]])
        #print(np.dot(rot_y,rot_x))
        return np.dot(rot_y,rot_x)
        #return np.dot(rot_x,rot_y)
    
    def rotation_matrix_COM(self,phi):
        rot_matrix = np.array([
            [np.cos(phi),-np.sin(phi),0],
            [np.sin(phi),np.cos(phi),0],
            [0,0,1]
        ])
        return rot_matrix

    def forward_kinematics(self):
        ## These are the rotation matrices for the transformation from vertex to the drone ############
        self.rot_vertex_1 = self.rotation_matrix_vertex(self.theta1_1,self.theta2_1)
        self.rot_vertex_2 = self.rotation_matrix_vertex(self.theta1_2,self.theta2_2)
        self.rot_vertex_3 = self.rotation_matrix_vertex(self.theta1_3,self.theta2_3)
        ###############################################################################################
        pos_vector = np.array([[0],[0],[self.length]])
        pos_vector_dash = np.array([[0],[self.side/np.sqrt(3)],[self.length]])
        pos_v1 = np.dot(self.rot_vertex_1,pos_vector)
        pos_v2 = np.dot(self.rot_vertex_2,pos_vector)
        pos_v3 = np.dot(self.rot_vertex_3,pos_vector)
        pos_v2_dash = np.dot(self.rot_vertex_2,pos_vector_dash)
        ##print(pos_v1)
        #print(pos_v2)
        #print(pos_v3)
        #print(pos_vector)
        #print(pos_v1)
        ################################################################################################
        rot_mat_1 = self.rotation_matrix_COM(0)
        rot_mat_2 = self.rotation_matrix_COM(np.pi*2/3)
        rot_mat_3 = self.rotation_matrix_COM(np.pi*4/3)
        ##print(rot_mat_1)
        pos_com1 = np.array([[0],[self.side/np.sqrt(3)],[0]]) + np.dot(rot_mat_1,pos_v1)
        
        pos_com2 = np.array([[-self.side/(2)],[-self.side/(2*np.sqrt(3))],[0]]) + np.dot(rot_mat_2,pos_v2)
        # print("Method1: ",pos_com2)
        # print("Method2: ",np.dot(rot_mat_2,pos_v2_dash))
        pos_com3 = np.array([[self.side/(2)],[-self.side/(2*np.sqrt(3))],[0]]) + np.dot(rot_mat_3,pos_v3)
        ################################################################################################
        pos_of_COM = np.array([[self.x],[self.y],[self.z]])
        self.drone1_pos = pos_of_COM + np.dot(self.rotation_matrix_COM(self.yaw),pos_com1)
        self.drone2_pos = pos_of_COM + np.dot(self.rotation_matrix_COM(self.yaw),pos_com2)
        self.drone3_pos = pos_of_COM + np.dot(self.rotation_matrix_COM(self.yaw),pos_com3)

        self.vertex1_pos = pos_of_COM+np.dot(self.rotation_matrix_COM(self.yaw),np.array([[0],[self.side/np.sqrt(3)],[0]]))
        self.vertex2_pos = pos_of_COM+np.dot(self.rotation_matrix_COM(self.yaw),np.array([[-self.side/(2)],[-self.side/(2*np.sqrt(3))],[0]]))
        self.vertex3_pos = pos_of_COM+np.dot(self.rotation_matrix_COM(self.yaw),np.array([[self.side/(2)],[-self.side/(2*np.sqrt(3))],[0]]))
        #return self.drone1_pos,self.drone2_pos,self.drone3_pos
        

drone_sys = DroneObjectSystem(1,2,4,np.pi/4,-np.pi/4,0,-np.pi/4,0,-np.pi/4,0,1,2)
#drone_sys = DroneObjectSystem(0,0,0,0,0,0,0,0,0,0,1,2)
drone_sys.forward_kinematics()
#print("drone 1:", drone_sys.drone1_pos)
#print("drone 2:",drone_sys.drone2_pos)
#print("drone 3:",drone_sys.drone3_pos)


drone_positions = np.hstack([drone_sys.drone1_pos, drone_sys.drone2_pos, drone_sys.drone3_pos])
#print(drone_positions)
triangle_vertices = np.hstack([drone_sys.vertex1_pos,drone_sys.vertex2_pos,drone_sys.vertex3_pos])
com_position = np.array([1,2,4])
# Your existing class definition and initialization
# (assuming it's already defined as per your code)

# Plot drone positions and triangle vertices
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract x, y, z coordinates from the drone positions
x = drone_positions[0, :]
y = drone_positions[1, :]
z = drone_positions[2, :]

# Extract x, y, z coordinates from the triangle vertices
x_t = triangle_vertices[0, :]
y_t = triangle_vertices[1, :]
z_t = triangle_vertices[2, :]

all_values = np.concatenate([x, y, z, x_t, y_t, z_t])
max_range = np.ptp(all_values) / 2.0  # Calculate the range for equal scaling
mid_x = np.mean([np.min(x), np.max(x)])
mid_y = np.mean([np.min(y), np.max(y)])
mid_z = np.mean([np.min(z), np.max(z)])

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)
# Plot drone positions
ax.scatter(x, y, z, color='blue', label='Drones')

# Plot triangle vertices
ax.scatter(x_t, y_t, z_t, color='green', label='Triangle')

# Plot COM of the equilateral triangle
ax.scatter(com_position[0], com_position[1], com_position[2], color='red', label='COM')

# Draw lines from each vertex to the corresponding drone
for i in range(3):
    ax.plot([x_t[i], x[i]], [y_t[i], y[i]], [z_t[i], z[i]], color='black', linestyle='--')

# Connect the vertices of the triangle
for i in range(3):
    ax.plot([x_t[i], x_t[(i + 1) % 3]], [y_t[i], y_t[(i + 1) % 3]], [z_t[i], z_t[(i + 1) % 3]], color='green')

# Optionally, label the points
for i in range(3):
    ax.text(x[i], y[i], z[i], f'Drone {i+1}', fontsize=12, color='blue')
    ax.text(x_t[i], y_t[i], z_t[i], f'Vertex {i+1}', fontsize=12, color='green')

ax.text(com_position[0], com_position[1], com_position[2], 'COM', fontsize=12, color='red')

# Set labels and title
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('Drone Positions and Triangle')

# Add a legend
ax.legend()

# Show the plot
plt.show()
