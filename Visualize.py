import yaml,os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import yaml
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pandas as pd
import shutil
os.chdir(os.path.dirname(os.path.abspath(__file__)))

#Load the txt file
# data = np.loadtxt('trajectory_design_8_constant_height.txt', dtype=float, delimiter='\t')
data = np.loadtxt('OBS_PID_trajectory.txt', dtype=float, delimiter=' ')
# data = np.loadtxt('OBS_MPC_trajectory.txt', dtype=float, delimiter=' ')


mode = "OBS_PID_payload"
csv_file = 'pid_obs_payload.csv'
controller = 'pid' #MPC
visualize = False
save = True
forward_truncate_scale = 6/100
truncation_point_scale = 5/100

if not os.path.exists(mode):
    os.mkdir(mode)

if os.path.exists(mode):
    shutil.rmtree(mode)
    os.mkdir(mode)


# Extract the position data
x_positions = data[:, 0]
y_positions = data[:, 1]
z_positions = data[:, 2]#-0.1


## Load the csv file
pid_traj = pd.read_csv(csv_file, sep=',',header=None)
pid_traj.columns = ['timestamp','x_positions','y_positions','z_positions']
pid_traj = pid_traj.drop([0], axis=0)
forward_truncate_point = int(len(pid_traj)*forward_truncate_scale)
truncation_point = int(len(pid_traj)*(1-truncation_point_scale))

pid_traj = pid_traj.iloc[forward_truncate_point:truncation_point,:]

timestamp = pid_traj['timestamp'].to_numpy().astype(float)
x_positions_pid = pid_traj['x_positions'].to_numpy().astype(float)
y_positions_pid = pid_traj['y_positions'].to_numpy().astype(float)
z_positions_pid = pid_traj['z_positions'].to_numpy().astype(float)
normalized_timestamp = (timestamp - timestamp[0])

os.chdir(mode)

# Plot the 3D trajectory
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_positions, y_positions, z_positions, label='Reference')
ax.plot(x_positions_pid, y_positions_pid, z_positions_pid, label=controller)
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title(f'Crazyflie Trajectory Reference vs {controller}(3D)')
ax.legend(loc='upper left')
if visualize:
    plt.show()
if save:
    plt.savefig(f'{mode}_3D.png')

interpolated_timestamp = np.linspace(0,normalized_timestamp[-1],len(x_positions))
###
# Plot the 2D trajectory in X-direction
fig_x = plt.figure()
ax = fig.add_subplot(111)
ax.plot(interpolated_timestamp,x_positions,label='Reference',linestyle='--')
ax.plot(normalized_timestamp, x_positions_pid,label=controller)
ax.set_xlabel('Time')
ax.set_ylabel('X Position')
ax.set_title('Trajectory Tracking X-direction')

# Plot the 2D trajectory in Y-direction
fig_y = plt.figure()
ax = fig.add_subplot(111)
ax.plot(interpolated_timestamp,y_positions,label='Reference',linestyle=(0, (5, 5)))
ax.plot(normalized_timestamp, y_positions_pid,label=controller)
ax.set_xlabel('Time')
ax.set_ylabel('Y Position')
ax.set_title('Trajectory Tracking Y-direction')




# Plot the 2D trajectory in Z-direction
fig_z = plt.figure()
ax = fig.add_subplot(111)
ax.plot(interpolated_timestamp,z_positions,label='Reference',linestyle='--')
ax.plot(normalized_timestamp, z_positions_pid,label=controller)
ax.set_xlabel('Time')
ax.set_ylabel('Z Position')
ax.set_title('Trajectory Tracking Z-direction')



## make a subplot combining fig_x, fig_y, fig_z
fig, axs = plt.subplots(3, 1,figsize=(7,6))
fig.suptitle('Trajectory Tracking')
axs[0].plot(interpolated_timestamp,x_positions,label='Reference')
axs[0].plot(normalized_timestamp, x_positions_pid,label=controller)
axs[0].set_ylabel('X Position')
axs[0].legend(loc='upper left')
axs[1].plot(interpolated_timestamp,y_positions,label='Reference')
axs[1].plot(normalized_timestamp, y_positions_pid,label=controller)
axs[1].set_ylabel('Y Position')
axs[1].legend(loc='upper left')
axs[2].plot(interpolated_timestamp,z_positions,label='Reference')
axs[2].plot(normalized_timestamp, z_positions_pid,label=controller)
axs[2].set_ylabel('Z Position')
axs[2].legend(loc='upper left')
axs[2].set_xlabel('Time')
if visualize:
    plt.show()
if save:
    plt.savefig(f'{mode}_XYZ.png')