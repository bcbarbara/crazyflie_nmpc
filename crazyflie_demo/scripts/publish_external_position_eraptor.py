#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, TransformStamped
from visualization_msgs.msg import MarkerArray
from crazyflie_driver.srv import UpdateParams, Takeoff, Land

def onNewMarkerArray(marker_array):
    global msg
    global pub
    global firstTransform

    if len(marker_array.markers) == 0:
        rospy.logwarn("Received empty marker array!")
        return

    if len(marker_array.markers) > 1:
        rospy.logwarn("We expect just one marker but got {}!".format(len(marker_array.markers)))
        
    marker = marker_array.markers[0]

    if len(marker.points) == 0:
        rospy.logwarm("Received empty marker!")
        return
    
    if len(marker.points) > 1:
        rospy.logwarm("We received a marker with {} points but expected just one!".format(len(marker.points)))

    point = marker.points[0]

    if firstTransform:
        # initialize kalman filter
        rospy.set_param("kalman/initialX", point.x)
        rospy.set_param("kalman/initialY", point.y)
        rospy.set_param("kalman/initialZ", point.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

        rospy.set_param("kalman/resetEstimation", 1)
        # rospy.set_param("locSrv/extPosStdDev", 1e-4)
        update_params(["kalman/resetEstimation"]) #, "locSrv/extPosStdDev"])
        firstTransform = False
    else:
        msg.header.frame_id = marker.header.frame_id
        msg.header.stamp = marker.header.stamp
        msg.header.seq += 1
        msg.point.x = point.x
        msg.point.y = point.y
        msg.point.z = point.z
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_eraptor', anonymous=True)

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    msg = PointStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    pub = rospy.Publisher("external_position", PointStamped, queue_size=1)
    rospy.Subscriber("cortex_marker_array", MarkerArray, onNewMarkerArray)

    rospy.spin()
