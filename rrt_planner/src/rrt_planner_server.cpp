#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <rrt_planner/rrt_planner.h>
#include <rrt_planner/GetRRTPlan.h>
#include "std_srvs/Empty.h"
#include "ros/ros.h"

Map* env_map;
std::string map_path;
int numOfDOFs;
double epsilon;
double sample_resolution;

bool rrt_planner_callback(rrt_planner::GetRRTPlan::Request  &req, rrt_planner::GetRRTPlan::Response &res)
{
    // Start and goal points
    double* start_point = (double*)malloc(numOfDOFs*sizeof(double));
    double* goal_point = (double*)malloc(numOfDOFs*sizeof(double));
    start_point[0] = req.start.pose.position.x;
    start_point[1] = req.start.pose.position.y;
    goal_point[0] = req.goal.pose.position.x;
    goal_point[1] = req.goal.pose.position.y;

    // Obstacle ids of interest
    // req.obstacle_ids.data is a int16[] array
    std::vector<int> ids_to_find;
    if (req.obstacle_ids.data.size() == 0){
        // If no obstacle ids are specified, then we avoid all obstacles
        for (int i = 0; i < env_map->num_obstacles(); i++)
            ids_to_find.push_back(i);
    }
    else{
        for (int i = 0; i < req.obstacle_ids.data.size(); i++)
            ids_to_find.push_back(req.obstacle_ids.data[i]);
    }
    std::vector<int>* obstacle_ids;
    obstacle_ids = &ids_to_find;

    // Check if start and goal are valid
    // Ideally we want to stop here and ask for new start/goal
    if(!isValid(env_map, start_point[0], start_point[1], obstacle_ids)){
		ROS_INFO("Invalid start configuration!\n");
        return false;
	}
	if(!isValid(env_map, goal_point[0], goal_point[1], obstacle_ids)) {
		ROS_INFO("Invalid goal configuration!\n");
        return false;
	}

    // To store the plan
	double** plan = NULL;
	int planlength = 0;

    ros::Time begin = ros::Time::now();

    // Call the planner
    RRTplanner rrt(env_map, start_point, goal_point, numOfDOFs, obstacle_ids, &plan, &planlength, epsilon, sample_resolution);

    Tree tree = rrt.initTree();
    rrt.run(&tree);

    ros::Time end = ros::Time::now();
    ros::Duration duration = end - begin;
    ROS_INFO("RRT planner took %f seconds", duration.toSec());

    // These should never occur. Can be removed later
    if (!equalDoubleArrays(plan[0], start_point, numOfDOFs))
	{
		throw std::runtime_error("Start position not matching");
	}
	if (!equalDoubleArrays(plan[planlength-1], goal_point, numOfDOFs))
	{
		throw std::runtime_error("Goal position not matching");
	}

    // Publish the plan
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    for (int i = 0; i < planlength; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = plan[i][0];
        pose.pose.position.y = plan[i][1];
        pose.pose.position.z = 0.0;
        path.poses.push_back(pose);
    }

    res.plan = path;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt_planner_server");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    // ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("generate_map");
    // std_srvs::Empty srv;
    std::string map_path;
    pnh.param<std::string>("/rrt_planner_server/map_path", map_path, "/home/aneesh/acsi_ws/src/acsi_crazyflie_nmpc/maps/config/map.yaml");
    numOfDOFs = pnh.param<int>("numOfDOFs", 2);
    epsilon = pnh.param<double>("epsilon", 1.0);
    sample_resolution = pnh.param<double>("sample_resolution", 0.1);

    // Load map
    Map temp_map = Map(map_path);
    env_map = &temp_map;

    ros::ServiceServer service = nh.advertiseService("rrt_planner_server", rrt_planner_callback);

    ROS_INFO("RRT planner ready.");
    ros::spin();

    return 0;
}