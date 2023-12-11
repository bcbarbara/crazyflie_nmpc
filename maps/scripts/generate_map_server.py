import rospy
import rospkg
import yaml
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerResponse
import datetime

class MapGenerator:
    def __init__(self):
        obstacle = {
            'id': -1,
            'x': 0.0,
            'y': 0.0,
            'r': 0.15  # radius, adjust as needed
        }
        self.obstacles = [obstacle, obstacle]
        self.num_obstacles = 0
        rospy.init_node('generate_map_server', anonymous=True)
        rospy.Subscriber('/mocap/object_pos_1', PoseStamped, self.pose_callback_1)
        rospy.Subscriber('/mocap/object_pos_2', PoseStamped, self.pose_callback_2)
        self.service = rospy.Service('generate_map', Trigger, self.handle_generate_map)
        

    def handle_generate_map(self, request):
        rospy.loginfo('Received generate map request')
        self.num_obstacles = 0
        if len(self.obstacles) >= 2:  # number of obstacles, adjust as needed
            filename=self.gen_map()
            rospy.loginfo(f'Generated map {filename}')
            return TriggerResponse(success=True, message=filename) 
        else:
            return TriggerResponse(success=False, message="Not enough obstacles")
       
    
    def pose_callback_1(self, data):

        obstacle = {
            'id': 0,
            'x': data.pose.position.x,
            'y': data.pose.position.z,
            'r': 0.15  # radius, adjust as needed
        }
        if self.is_valid_pose(obstacle):
            self.obstacles[0]=obstacle
    
    def pose_callback_2(self, data):

        obstacle = {
            'id': 1,
            'x': data.pose.position.x,
            'y': data.pose.position.z,
            'r': 0.15  # radius, adjust as needed
        }
        if self.is_valid_pose(obstacle):
            self.obstacles[1]=obstacle
        
        
    def is_valid_pose(self, obstacle):
        return obstacle['x'] >= -0.5 and obstacle['x'] <= 1.5 and obstacle['y'] >= -1.8 and obstacle['y'] <= 0.5
    
    def gen_map(self):
        for i in range(len(self.obstacles)):
            if(self.obstacles[i]['id']!=-1):
                self.num_obstacles+=1

        map_data = {
        'num_obstacles': self.num_obstacles,
        'x_min': -0.5,
        'x_max': 1.5,
        'y_min': -1.8,
        'y_max': 0.5,
        'obstacles': {obstacle['id']: {key: val for key, val in obstacle.items() if key != 'id'} for obstacle in self.obstacles if obstacle['id'] != -1}
    }
        
        curr_time = datetime.datetime.now().strftime('%d_%H_%M')
        # find maps package path using rospack find
        maps_package_path = rospkg.RosPack().get_path('maps')
        filename = f'{maps_package_path}/config/map_{curr_time}.yaml'

        with open(filename, 'w') as f:
            yaml.dump(map_data, f, sort_keys=False)

        return f"map_{curr_time}.yaml"

if __name__ == '__main__':
    MapGenerator()
    rospy.spin()