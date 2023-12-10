
import rospy
from geometry_msgs.msg import PoseStamped
from rrt_planner.srv import GetRRTPlan
from std_msgs.msg import Int16MultiArray

def call_rrt_planner_service(start_pose, goal_pose, obstacle_ids=[]):
    rospy.wait_for_service('rrt_planner_server')
    try:
        rrt_planner_service = rospy.ServiceProxy('rrt_planner_server', GetRRTPlan)

        # Prepare the request
        request = GetRRTPlan()
        request.start = start_pose
        request.goal = goal_pose
        request.obstacle_ids = Int16MultiArray(data=obstacle_ids)

        # Call the service
        response = rrt_planner_service(request)

        return response.plan

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('rrt_planner_client', anonymous=True)

    # Define start and goal poses
    start_pose = PoseStamped()
    start_pose.pose.position.x = 0.0
    start_pose.pose.position.y = 0.0

    goal_pose = PoseStamped()
    goal_pose.pose.position.x = 1.5
    goal_pose.pose.position.y = 1.0

    # Optional: Specify obstacle IDs
    obstacle_ids = ['0']

    # Call the RRT planner service
    plan = call_rrt_planner_service(start_pose, goal_pose, obstacle_ids)

    # Print the plan
    if plan:
        for pose in plan.poses:
            print(f"X: {pose.pose.position.x}, Y: {pose.pose.position.y}")
    else:
        print("Failed to retrieve a valid plan.")