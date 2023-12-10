import rospy
import numpy as np
import math
import yaml
from geometry_msgs.msg import PoseStamped
from collections import OrderedDict
from std_srvs.srv import Empty, EmptyResponse

class MapGenerator:
    def __init__(self):
        self.obstacles = ['sdsd']
        rospy.init_node('generate_map_server', anonymous=True)
        rospy.Subscriber('/mocap/object_pos_1', PoseStamped, self.pose_callback)
        self.service = rospy.Service('generate_map', Empty, self.handle_generate_map)
        
        # rospy.Subscriber('/mocap/object_pos_2', PoseStamped, self.pose_callback)
        # rospy.Subscriber('/mocap/object_pos_3', PoseStamped, self.pose_callback)
    
    def handle_generate_map(self, request):
        if len(self.obstacles) >= 1:  # number of obstacles, adjust as needed
            self.gen_map() 
        return EmptyResponse()
    def pose_callback(self, data):

        obstacle = {
            'id': 0,
            'x': data.pose.position.x,
            'y': data.pose.position.z,
            'r': 0.15  # radius, adjust as needed
        }
        if self.is_valid_pose(obstacle):
            self.obstacles[0]=obstacle
        
        

    def is_valid_pose(self, obstacle):
        return obstacle['x'] >= -5.0 and obstacle['x'] <= 10.0 and obstacle['y'] >= -5.0 and obstacle['y'] <= 10.0
    
    def gen_map(self):
        map_data = {
        'num_obstacles': len(self.obstacles),
        'x_min': -5.0,
        'x_max': 10.0,
        'y_min': -5.0,
        'y_max': 10.0,
        'obstacles': {obstacle['id']: {key: val for key, val in obstacle.items() if key != 'id'} for obstacle in self.obstacles}
    }
        filename='/home/aneesh/acsi_ws/src/acsi_crazyflie_nmpc/maps/config/map.yaml'
        with open(filename, 'w') as f:
            yaml.dump(map_data, f, sort_keys=False)
        # rospy.signal_shutdown('Map generated')

if __name__ == '__main__':
    MapGenerator()
    rospy.spin()