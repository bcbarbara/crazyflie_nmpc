#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <rrt_planner/rrt_planner.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Path publisher
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("rrt_path", 1);

    // Get parameters
    // TODO: Take path from ros params and not hardcode
    // Currently path is hardcoded coz yaml-cpp complains when loading it from ros params
    std::string map_path = pnh.param<std::string>("map_path", "/home/devansh/airlab/mpc_ws/src/subcanopy_flight/maps/config/map.yaml");
    int numOfDOFs = pnh.param<int>("numOfDOFs", 2);
    double epsilon = pnh.param<double>("epsilon", 0.2);
    double sample_resolution = pnh.param<double>("sample_resolution", 0.1);

    // Load map
    Map* map;
    Map temp_map = Map(map_path);
    map = &temp_map;

    // Start and goal points
    double* start_point = (double*)malloc(numOfDOFs*sizeof(double));
	double* goal_point = (double*)malloc(numOfDOFs*sizeof(double));
	start_point[0] = 0.0;
	start_point[1] = -2.5;
	goal_point[0] = 0.0;
	goal_point[1] = 9.0;

    // Obstacle ids of interest
    std::vector<int> ids_to_find = {0};
    std::vector<int>* obstacle_ids;
	obstacle_ids = &ids_to_find;

    // Check if start and goal are valid
    // Ideally we want to stop here and ask for new start/goal
    if(!isValid(map, start_point[0], start_point[1], obstacle_ids)){
		ROS_INFO("Invalid start configuration!\n");
	}
	if(!isValid(map, goal_point[0], goal_point[1], obstacle_ids)) {
		ROS_INFO("Invalid goal configuration!\n");
	}

    // To store the plan
	double** plan = NULL;
	int planlength = 0;

    ros::Time begin = ros::Time::now();

    // Call the planner
    RRTplanner rrt(map, start_point, goal_point, numOfDOFs, obstacle_ids, &plan, &planlength, epsilon, sample_resolution);

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
    path.header.frame_id = "map";
    for (int i = 0; i < planlength; i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = plan[i][0];
        pose.pose.position.y = plan[i][1];
        pose.pose.position.z = 0.0;
        path.poses.push_back(pose);
    }

    // Keep publishing the path until the node is killed
    while(ros::ok())
        path_pub.publish(path);

    return 0;
}