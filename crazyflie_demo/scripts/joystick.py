#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import crazyflie
import time
from crazyflie_driver.srv import *
from std_srvs.srv import Empty


class Controller():
    def __init__(self, use_controller, joy_topic, prefix, tf):

        self.cf = crazyflie.Crazyflie(prefix,tf)

        self.cf.setParam("commander/enHighLevel", 1)
        self.cf.setParam("stabilizer/estimator", 2) # Use EKF
        self.cf.setParam("stabilizer/controller", 2) # Use mellinger controller
        # reset kalman
        self.cf.setParam("kalman/resetEstimation", 1)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
            rospy.loginfo("waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo("found land service")
            self.cf.landService

            rospy.loginfo("waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo("found takeoff service")
            self.cf.takeoffService
        else:
            self.cf.landService = None
            self.cf.takeoffService = None

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                # Button for LANDING in Logitech Gamepad -> A
                if i == 0 and data.buttons[i] == 1 and self.cf.landService != None:
                    self.cf.land(targetHeight = 0.0, duration = 2.0)
                    time.sleep(3.0)
                # Button for EMERGENGY in Logitech Gamepad ->  B
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
                # Button for TAKE OFF in Logitech Gamepad ->  X
                if i == 2 and data.buttons[i] == 1 and self.cf.takeoffService != None:
                    self.cf.takeoff(targetHeight = 0.5, duration = 2.0)
                    time.sleep(10.0)
                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))
                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)
                    self._update_params(["ring/headlightEnable"])
                    print(not value)

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    body_frame = rospy.get_param("~prefix", "/crazyflie")
    world_frame = rospy.get_param("~tf", "/mocap")
    controller = Controller(use_controller, joy_topic, body_frame, world_frame)

    rospy.spin()
