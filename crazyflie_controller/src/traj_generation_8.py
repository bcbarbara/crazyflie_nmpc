import numpy as np
import matplotlib.pyplot as plt

# Parameters
r = 0.5          # Radius
h0 = 0.6         # Initial height
tf = 35          # Total duration
ts = 0.015       # Sampling time
m = round(tf/ts)  # Number of intervals
t_1=10
t_2=tf-t_1
# Time vector
t1 = np.arange(0, t_1, ts)    # Initial segment (rise to height 0.6)
t2 = np.arange(t_1, tf + ts, ts)  # Subsequent segment (shape of '8')

# Segment 1: Rise to height 0.6
theta1 = np.pi/2 * t1 / t_1  # Angular parameter for the first segment
x1 = np.zeros_like(t1)    # X-coordinate remains 0
y1 = np.zeros_like(t1)    # Y-coordinate remains 0
# z1 = h0 * np.sin(theta1)   # Rising to height 0.6
z1=np.linspace(0.0,0.6,len(t1))
# Segment 2: '8' design at constant height of 0.6 starting from (0, 0, 0.6)
theta2 = 2 * np.pi * (t2 - t_1) / t_2  # Angular parameter for the '8' pattern
x2 = r * np.cos(theta2) -r            # X-coordinate for the '8' pattern
y2 = r * np.sin(2 * theta2)         # Y-coordinate for the '8' pattern
z2 = np.full_like(t2, h0)           # Constant Z-coordinate at height 0.6

# Concatenate the segments
t = np.concatenate((t1, t2))
x = np.concatenate((x1, x2))
y = np.concatenate((y1, y2))
z = np.concatenate((z1, z2))
q1 = np.full_like(t, 1.0000)     # Constant q1-coordinate
q2 = np.full_like(t, 0.0000)     # Constant q2-coordinate
q3 = np.full_like(t, 0.0000)     # Constant q3-coordinate
q4 = np.full_like(t, 0.0000)     # Constant q4-coordinate
v1 = np.full_like(t, 0.0000)     # Constant v1-coordinate
v2 = np.full_like(t, 0.0000)     # Constant v2-coordinate
v3 = np.full_like(t, 0.0000)     # Constant v3-coordinate
v4 = np.full_like(t, 0.0000)     # Constant v4-coordinate
v5 = np.full_like(t, 0.0000)     # Constant v5-coordinate
v6 = np.full_like(t, 0.0000)     # Constant v6-coordinate
t1 = np.full_like(t, 15.7777)     # Constant t1-coordinate
t2 = np.full_like(t, 15.7777)     # Constant t2-coordinate
t3 = np.full_like(t, 15.7777)     # Constant t3-coordinate
t4 = np.full_like(t, 15.7777)     # Constant t4-coordinate
# Create a matrix for the trajectory
trajectory = np.column_stack((x, y, z, q1, q2, q3, q4, v1, v2, v3, v4, v5,v6,t1, t2, t3, t4))

# Save the trajectory to a file
np.savetxt('trajectory_design_8_constant_height.txt', trajectory, delimiter='\t', fmt='%.4f')

# Plot the trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z)
ax.set_title('Helical Trajectory with Design "8" at Constant Height 0.6')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.grid(True)
plt.show()