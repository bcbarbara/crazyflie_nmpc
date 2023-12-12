#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class PoseRotator:
    def __init__(self):
        rospy.init_node('optitrack_rotate', anonymous=True)

        # Define the input and output topics
        self.input_topic = '/mocap/external_position'
        self.output_topic = '/crazyflie/external_position'

        # Set the rotation angle (in radians)
        self.theta = 90 * 3.14/180  # Change this to your desired rotation angle

        # Create a subscriber for the input PoseStamped topic
        rospy.Subscriber(self.input_topic, PoseStamped, self.pose_callback)

        # Create a publisher for the output PoseStamped topic
        self.pose_publisher = rospy.Publisher(self.output_topic, PoseStamped, queue_size=10)

    def pose_callback(self, input_pose):
        # Rotate the pose about the x-axis
        rotated_pose = self.rotate_pose(input_pose)

        # Publish the rotated pose
        self.pose_publisher.publish(rotated_pose)

    def rotate_pose(self, input_pose):
        # Extract quaternion from the original pose
        quaternion_original = (
            input_pose.pose.orientation.x,
            input_pose.pose.orientation.y,
            input_pose.pose.orientation.z,
            input_pose.pose.orientation.w
        )

        # Convert quaternion to Euler angles
        euler_angles = euler_from_quaternion(quaternion_original)

        # Rotate the pose about the x-axis by the specified angle
        euler_angles = (euler_angles[0] + self.theta, euler_angles[1], euler_angles[2])

        # Convert Euler angles back to quaternion
        quaternion_rotated = quaternion_from_euler(*euler_angles)


        # Update the pose with the rotated orientation
        rotated_pose = PoseStamped()
        rotated_pose.header = input_pose.header
        rotated_pose.pose.position.x = input_pose.pose.position.x
        rotated_pose.pose.position.y = input_pose.pose.position.z
        rotated_pose.pose.position.z = -input_pose.pose.position.y
        rotated_pose.pose.orientation.x = quaternion_rotated[0]
        rotated_pose.pose.orientation.y = quaternion_rotated[1]
        rotated_pose.pose.orientation.z = quaternion_rotated[2]
        rotated_pose.pose.orientation.w = quaternion_rotated[3]

        return rotated_pose

if __name__ == '__main__':
    try:
        pose_rotator = PoseRotator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
