#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from rrt_planner.srv import GetRRTPlan
from std_msgs.msg import Int16MultiArray, String
from std_srvs.srv import Trigger
import numpy as np
import math

def generate_or_add_trajectory(start, goal, step, filename, is_first_call):
    if is_first_call:
        generate_trajectory_file(start, goal, step, filename)
    else:
        add_trajectory_file(start, goal, step, filename)

def generate_trajectory_for_poses(plan, step, filename):
    if not plan or len(plan.poses) <= 1:
        print("Invalid or empty plan.")
        return
    for pose in plan.poses:
        pose.pose.position.z = 0.6
    current_pose = plan.poses[0]
    current_pose.pose.position.z = 0.0
    is_first_call = True
    for next_pose in plan.poses[1:]:
        start = [current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z]
        goal = [next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z]

        generate_or_add_trajectory(start, goal, step, filename, is_first_call)
        is_first_call = False
        current_pose = next_pose

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

def call_rrt_planner_service(start_pose, goal_pose, obstacle_ids=[],map_filename=""):
    rospy.wait_for_service('rrt_planner_server')
    try:
        rrt_planner_service = rospy.ServiceProxy('rrt_planner_server', GetRRTPlan)

        response = rrt_planner_service(start_pose,goal_pose,Int16MultiArray(data=obstacle_ids),String(data=f"{map_filename}"))

        return response.plan

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

def call_generate_map_service():
    rospy.wait_for_service('generate_map')
    try:
        generate_map = rospy.ServiceProxy('generate_map', Trigger)
        resp = generate_map()
        return resp.message
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('rrt_planner_client', anonymous=True)

    # Define start and goal poses
    start_pose = PoseStamped()
    start_pose.pose.position.x = 0.0
    start_pose.pose.position.y = 0.0

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = -1.6

    # Optional: Specify obstacle IDs
    obstacle_ids = [1,2,3]
    map_filename = call_generate_map_service()
    # Call the RRT planner service
    plan = call_rrt_planner_service(start_pose, goal_pose, obstacle_ids, map_filename)

    # Print the plan
    if plan:
        for pose in plan.poses:
            print(f"X: {pose.pose.position.x}, Y: {pose.pose.position.y}")
        
        # Generate trajectories for the plan
        step_size = 0.001  # Define your desired step size here
        trajectory_filename = "/home/aneesh/acsi_ws/src/acsi_crazyflie_nmpc/crazyflie_controller/traj/trajectory.txt"  # Define your filename here
        generate_trajectory_for_poses(plan, step_size, trajectory_filename)
    else:
        print("Failed to retrieve a valid plan.")