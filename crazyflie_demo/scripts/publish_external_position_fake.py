#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
from crazyflie_driver.srv import UpdateParams

if __name__ == '__main__':

    rospy.init_node('publish_external_position_fake', anonymous=True)
    pub = rospy.Publisher("/crazyflie/external_position", PointStamped, queue_size=1)

    rate = rospy.Rate(10) # 10 hz
    msg = PointStamped()

    while not rospy.is_shutdown():

        msg.header.stamp = rospy.Time.now()
        msg.header.seq += 1
        msg.point.x = 0
        msg.point.y = 0
        msg.point.z = 0
        pub.publish(msg)

        rate.sleep()
