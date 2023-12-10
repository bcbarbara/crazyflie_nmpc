import numpy as np
import math
def generate_trajectory_file(start, goal, step, filename):
    
    n_x = math.ceil(abs(goal[0] - start[0]) / step)
    n_y = math.ceil(abs(goal[1] - start[1]) / step)
    n_z = math.ceil(abs(goal[2] - start[2]) / step)
    n = max(n_x, n_y, n_z)
    x_values = np.linspace(start[0], goal[0], n)
    y_values = np.linspace(start[1], goal[1], n)
    z_values = np.linspace(start[2], goal[2], n)
    with open(filename, 'w') as f:
        for i in range(n):
            line = f"{x_values[i]:.4f} {y_values[i]:.4f} {z_values[i]:.4f} 1.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 15.7777 15.7777 15.7777 15.7777\n"
            f.write(line)

def add_trajectory_file(start, goal, step, filename):
    
    n_x = math.ceil(abs(goal[0] - start[0]) / step)
    n_y = math.ceil(abs(goal[1] - start[1]) / step)
    n_z = math.ceil(abs(goal[2] - start[2]) / step)
    n = max(n_x, n_y, n_z)
    x_values = np.linspace(start[0], goal[0], n)
    y_values = np.linspace(start[1], goal[1], n)
    z_values = np.linspace(start[2], goal[2], n)
    with open(filename, 'a') as f:
        for i in range(n):
            line = f"{x_values[i]:.4f} {y_values[i]:.4f} {z_values[i]:.4f} 1.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 15.7777 15.7777 15.7777 15.7777\n"
            f.write(line)

if __name__ == '__main__':
    file_path = '/home/aneesh/acsi_ws/src/acsi_crazyflie_nmpc/crazyflie_controller/traj/traj_exp.txt'
    generate_trajectory_file([0, 0, 0], [0, 0, 0.6], 0.001, file_path)
    add_trajectory_file([0, 0, 0.6], [0, -0.8, 0.6], 0.001, file_path)
    # print(os.path.abspath('traj_exp.txt'));