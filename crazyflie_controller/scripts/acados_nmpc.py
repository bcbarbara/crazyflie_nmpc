import rospy
from std_msgs.msg import String
from acados_template import *
import numpy as np

#  from crazyflie_controller.msg import
#  from crazyflie_controller.msg import
#  from crazyflie_controller.msg import
#  from crazyflie_controller.msg import
#  from crazyflie_controller.msg import

from crazyflie_full_model.c_generated_code import ra

from geometry.msg import PointStampedConst
from geometry.msg import Vector3Stamped
from geometry.msg import Twist
from sensor.msg import Imu

xq = 0
yq = 1
zq = 2
q1 = 3
q2 = 4
q3 = 5
q4 = 6
vbx = 7
vby = 8
vbz = 9
wx = 10
wy = 11
wz = 12

def krpm2pwm(Krpm):
  return ((Krpm*1000)-4070.3)/0.2685


class mpc:

    def __init__(self, control_pub):
        self.acados_sover = generate_solver(model, ra, json_file = 'crazyflie_full_model/acados_ocp.json')

        self.control_pub = control_pub
        self.N = 50
        self.Nx = 13
        self.Nu = 4

        self.x0 = np.array([0.20, -0.40 , 0.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.x0_last = np.array([0.20, -0.40 , 0.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    def mpc_iteration(self):

        self.acados_solver.set(0, "lbx", self.x0)
        self.acados_solver.set(0, "ubx", self.x0)

        status = self.acados_solver.solve()

        for j in range(nx):
            simX[i,j] = x0[j]

        for j in range(nu):
            simU[i,j] = u0[j]

        # get solution
        #  x0 = self.acados_solver.get(0, "x")
        self.u0 = self.acados_solver.get(0, "u")
        self.x0_last = self.acados_solver.get(0, "x")

        control_msg = Twist()
        control_msg.linear.x  = 0
        control_msg.linear.y  = 0
        control_msg.linear.z  = rpm2pwm((self.u0[w1]+self.u0[w2]+self.u0[w3]+self.u0[w4])/4)
        control_msg.angular.z = 0

        self.control_pub.publish(control_msg)

    def mocap_callback(self, msg):
        self.x0[xq] = msg.point.x
        self.x0[yq] = msg.point.y
        self.x0[zq] = msg.point.z

    def imu_callback(self):
        #  self.x0[wx] = msg.angular_velocity.x
        #  self.x0[wy] = msg.angular_velocity.y
        #  self.x0[wz] = msg.angular_velocity.z

        self.x0[wx] = 0
        self.x0[wy] = 0
        self.x0[wz] = 0

    def euler_callback(self):
        #  self.mocap_x = msg.vector.x
        #  self.mocap_y = msg.vector.y
        #  self.mocap_z = msg.vector.z

        self.x0[q1] = 1
        self.x0[q2] = 0
        self.x0[q3] = 0
        self.x0[q4] = 0

if __name__ == '__main__':

    try:
        rospy.init_node("acados_nmpc")

        rate = rospy.Rate(20)

        crazyflie_input = rospy.Publisher('/crazyflie/cmd_vel', Twist, queue_size=10)
        mpc = acados_solver(control_pub=crazyflie_input)

        # subscriber
        rospy.Subscriber("/crazyflie/external_position", PointStampedConst, mpc.mocap_callback, queue_size=10)
        rospy.Subscriber("/crazyflie/imu", Imu, mpc.imu_callback, queue_size=10)
        rospy.Subscriber("/crazyflie/euler_angles", Vector3Stamped, mpc.euler_callback, queue_size=10)



    except rospy.ROSInterruptException:
        pass
