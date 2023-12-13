#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf
import yaml

def obstacle_callback_1(pose):
    global obstacle_marker_pub_1

    marker = Marker()
    marker.header.frame_id = "world"  # Replace "world" with your frame ID
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = pose.pose.position.x  # Use the pose from the received PoseStamped message
    marker.pose.position.y = pose.pose.position.z  # Use the pose from the received PoseStamped message
    marker.pose.position.z = 1  # Use the pose from the received PoseStamped message
    marker.scale.x = 0.1  # Set the cylinder dimensions as needed
    marker.scale.y = 0.1
    marker.scale.z = 2.0
    marker.color.r = 0.1
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    obstacle_marker_pub_1.publish(marker)

def obstacle_callback_2(pose):
    global obstacle_marker_pub_2

    marker = Marker()
    marker.header.frame_id = "world"  # Replace "world" with your frame ID
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.pose.position.x = pose.pose.position.x  # Use the pose from the received PoseStamped message
    marker.pose.position.y = pose.pose.position.z  # Use the pose from the received PoseStamped message
    marker.pose.position.z = 1  # Use the pose from the received PoseStamped message
    marker.scale.x = 0.1  # Set the cylinder dimensions as needed
    marker.scale.y = 0.1
    marker.scale.z = 2.0
    marker.color.r = 0.1
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    obstacle_marker_pub_2.publish(marker)


def pose_callback(pose):
    global path_pub, path_msg
    path_msg.poses.append(pose)
    path_msg.header = pose.header
    path_pub.publish(path_msg)

def crazyflie_pos(pose):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
        (0, 0, 0, 1),
        rospy.Time.now(),
        "base_link", 
        "world" 
    )   


def generate_trajectory():
    global trajectory_pub, marker_pub

    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.005
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    trajectory = Path()
    trajectory.header.frame_id = "world"

    with open('/home/ved/acsi_ws/src/acsi_crazyflie_nmpc/crazyflie_controller/traj/trajectory_obs.txt', 'r') as file:
        lines = file.readlines()

        for line in lines:
            coords = line.split()
            if len(coords) >= 3:
                point = Point()
                point.x = float(coords[0])
                point.y = float(coords[1])
                point.z = float(coords[2])
                marker.points.append(point)

    while not rospy.is_shutdown():
        trajectory_pub.publish(trajectory)
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rospy.sleep(1)

def save_poses_to_yaml():
    poses_list = []
    for pose in path_msg.poses:
        pose_dict = {
            'position': {
                'x': pose.pose.position.x,
                'y': pose.pose.position.y,
                'z': pose.pose.position.z
            },
            # Add orientation if needed
            # 'orientation': {
            #     'x': pose.pose.orientation.x,
            #     'y': pose.pose.orientation.y,
            #     'z': pose.pose.orientation.z,
            #     'w': pose.pose.orientation.w
            # }
        }
        poses_list.append(pose_dict)

    yaml_filename = 'poses.yaml'
    with open(yaml_filename, 'w') as file:
        yaml.dump(poses_list, file)
    

def main():
    global path_pub, path_msg, trajectory_pub, marker_pub, obstacle_marker_pub_1, obstacle_marker_pub_2
    rospy.init_node('trajectory_visualizer', anonymous=True)

    path_pub = rospy.Publisher('/cf_path', Path, queue_size=10)
    trajectory_pub = rospy.Publisher('/trajectory', Path, queue_size=10)
    marker_pub = rospy.Publisher('/trajectory_marker', Marker, queue_size=10)

    rospy.Subscriber('/crazyflie/external_position', PoseStamped, pose_callback)
    rospy.Subscriber('/crazyflie/external_position', PoseStamped, crazyflie_pos)
 
    path_msg = Path()
    path_msg.header.frame_id = "world"

    obstacle_marker_pub_1 = rospy.Publisher('/obstacle_markers_1', Marker, queue_size=10)
    obstacle_marker_pub_2 = rospy.Publisher('/obstacle_markers_2', Marker, queue_size=10)
    rospy.Subscriber('/mocap/object_pos_1', PoseStamped, obstacle_callback_1)
    rospy.Subscriber('/mocap/object_pos_2', PoseStamped, obstacle_callback_2)

    generate_trajectory()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
