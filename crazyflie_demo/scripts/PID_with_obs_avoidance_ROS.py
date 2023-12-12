# -*- coding: utf-8 -*-# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
import logging
import sys
import time
from threading import Event
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
import logging
import sys
import time
from threading import Event
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.motion_commander import _SetPointThread
from cflib.utils import uri_helper
import math
from cflib.utils.multiranger import Multiranger
from threading import Thread
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import PoseStamped

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.3
BOX_LIMIT = 0.5

deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)
pose = [0, 0, 0]


def is_close(range):
    MIN_DISTANCE = 0.3  # m

    if range is None:
        return False
    else:
        return range < MIN_DISTANCE
    
def PID_control_x(Kp, Ki, Kd, dt, error, E):
    P = Kp * error
    I = Ki * E
    D = (Kd * error)/ dt
    u = P + I + D
    return u

def PID_control_y(Kp, Ki, Kd, dt, error, E):
    P = Kp * error
    I = Ki * E
    D = (Kd * error)/ dt
    u = P + I + D
    return u

def PID_control_z(Kp, Ki, Kd, dt, error, E):
    P = Kp * error
    I = Ki * E
    D = (Kd * error)/ dt
    u = P + I + D
    return u

def move_linear_simple(scf, data_array):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        pos_x = np.array([])
        pos_y = np.array([])
        pos_z = np.array([])
        print('Taking off!')
        time.sleep(0.1)
        for i in range(len(data_array)):
            distance_x_m = data_array[i][0]
            distance_y_m = data_array[i][1]
            distance_z_m = data_array[i][2]
            des_pos = np.array([distance_x_m, distance_y_m, distance_z_m])
            prev_time = time.time()
            error = np.array([distance_x_m, distance_y_m, distance_z_m])
            cumm_error = np.array([0.0,0.0,0.0])
            print("Init estimate (0, 0, 0): ", pose[0], pose[1], pose[2])
            print('Starting control loop')
            while True: 
                curr_pos = np.array([pose[0], pose[1], pose[2]])
                pos_x = np.append(pos_x,pose[0])
                pos_y = np.append(pos_x,pose[1])
                pos_z = np.append(pos_z,pose[2])
                print("error: ",error)
                error = des_pos - curr_pos
                if(all(abs(error)<0.04)):
                    break
                print("estimate: ", pose[0], pose[1], pose[2])
                curr_time = time.time()
                # write a pid controller to control the velocity
                dt = curr_time - prev_time
                cumm_error += error * dt
                u_x = PID_control_x(0.1, 0.001, 0.15, dt, error[0], cumm_error[0])
                u_y = PID_control_y(0.8, 0.001, 0.1, dt, error[1], cumm_error[1])
                u_z = PID_control_z(0.1, 0.001, 0.15, dt, error[2], cumm_error[2])
                u_x = np.clip(u_x, -0.075, 0.075)
                u_y = np.clip(u_y, -0.075, 0.075)
                u_z = np.clip(u_z, -0.075, 0.075)
                print("control input u_x u_y: ", u_x, u_y, u_z)
                mc.start_linear_motion(u_x, u_y, u_z)
                time.sleep(0.1)
                prev_time = curr_time
        print(error)

        return pos_x, pos_y, pos_z


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        mc.stop()


def param_deck_flow(_, value_str):
    value = int(value_str)
    #print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')

class MOCAP:
    def __init__(self):
        rospy.init_node('optitrack_avoid_cf', anonymous=True)

        # Define the input topic:
        self.input_topic = '/crazyflie/external_position'

        # Create a subscriber for the input PoseStamped topic
        rospy.Subscriber(self.input_topic, PoseStamped, self.pose_callback)
    
    def pose_callback(self, input_pose):
        global pose
        pose[0] = input_pose.pose.position.x
        pose[1] = input_pose.pose.position.y
        pose[2] = input_pose.pose.position.z
        # rospy.loginfo(pose[0],pose[1], pose[2])
        return input_pose


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    obs_data = MOCAP()
    

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)
        
        data_array = []
        with open('traj_exp.txt', 'r') as file:
            for line in file:
                # Split the line by whitespace and take the first three values
                values = line.split()[:3]
            
                # Convert the values to floats and append to the array
                data_array.append([float(value) for value in values])
        
        
        pos_x, pos_y, pos_z = move_linear_simple(scf, data_array)
        len_x = np.linspace(0, np.size(pos_x), np.size(pos_x))
        len_y = np.linspace(0, np.size(pos_y), np.size(pos_y))
        plt.plot(len_x, pos_x)
        plt.show()
        plt.plot(len_y, pos_y)
        plt.show()

    rospy.spin()