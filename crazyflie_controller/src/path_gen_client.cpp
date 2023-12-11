#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rrt_planner/GetRRTPlan.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <cmath>
#include <fstream>
#include <vector>

void generateOrAddTrajectory(const std::vector<double>& start, const std::vector<double>& goal, double step, const std::string& filename, bool isFirstCall);

void generateTrajectoryForPoses(nav_msgs::Path& plan, double step, const std::string& filename);

void generateTrajectoryFile(const std::vector<double>& start, const std::vector<double>& goal, double step, const std::string& filename);

void addTrajectoryFile(const std::vector<double>& start, const std::vector<double>& goal, double step, const std::string& filename);

nav_msgs::Path callRRTPlannerService(geometry_msgs::PoseStamped& startPose, geometry_msgs::PoseStamped& goalPose, std::vector<int>& obstacleIds, std::string& mapFilename);

std::string callGenerateMapService();

int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_planner_client");
    ros::NodeHandle nh;

    // Define start and goal poses
    geometry_msgs::PoseStamped startPose;
    startPose.pose.position.x = 0.0;
    startPose.pose.position.y = 0.0;

    geometry_msgs::PoseStamped goalPose;
    goalPose.pose.position.x = 0.0;
    goalPose.pose.position.y = -1.6;

    // Optional: Specify obstacle IDs
    std::vector<int> obstacleIds = {};
    std::string mapFilename = callGenerateMapService();

    // Call the RRT planner service
    nav_msgs::Path plan = callRRTPlannerService(startPose, goalPose, obstacleIds, mapFilename);

    // Print the plan
    if (!plan.poses.empty()) {
        for (const auto& pose : plan.poses) {
            std::cout << "X: " << pose.pose.position.x << ", Y: " << pose.pose.position.y << std::endl;
        }

        // Generate trajectories for the plan
        double stepSize = 0.001;  // Define your desired step size here
        std::string trajectoryFilename = "/home/aneesh/acsi_ws/src/acsi_crazyflie_nmpc/crazyflie_controller/traj/trajectory.txt";  // Define your filename here
        generateTrajectoryForPoses(plan, stepSize, trajectoryFilename);
    } else {
        std::cout << "Failed to retrieve a valid plan." << std::endl;
    }

    return 0;
}

std::vector<double> linspace(double start, double end, int num) {
    std::vector<double> result;
    if (num <= 1) {
        result.push_back(start);
        return result;
    }

    double step = (end - start) / static_cast<double>(num - 1);
    for (int i = 0; i < num; ++i) {
        result.push_back(start + i * step);
    }

    return result;
}

void generateOrAddTrajectory(const std::vector<double>& start, const std::vector<double>& goal, double step, const std::string& filename, bool isFirstCall) {
    if (isFirstCall) {
        generateTrajectoryFile(start, goal, step, filename);
    } else {
        addTrajectoryFile(start, goal, step, filename);
    }
}

void generateTrajectoryForPoses(nav_msgs::Path& plan, double step, const std::string& filename) {
    if (plan.poses.empty()) {
        std::cout << "Invalid or empty plan." << std::endl;
        return;
    }

    for (auto& pose : plan.poses) {
        pose.pose.position.z = 0.6;
    }

    geometry_msgs::PoseStamped currentPose = plan.poses.front();
    currentPose.pose.position.z = 0.0;
    bool isFirstCall = true;

    for (auto& nextPose : plan.poses) {
        std::vector<double> start = {currentPose.pose.position.x, currentPose.pose.position.y, currentPose.pose.position.z};
        std::vector<double> goal = {nextPose.pose.position.x, nextPose.pose.position.y, nextPose.pose.position.z};

        generateOrAddTrajectory(start, goal, step, filename, isFirstCall);
        isFirstCall = false;
        currentPose = nextPose;
    }
}

void generateTrajectoryFile(const std::vector<double>& start, const std::vector<double>& goal, double step, const std::string& filename) {
    std::ofstream file(filename, std::ios::out);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    int nX = std::ceil(std::abs(goal[0] - start[0]) / step);
    int nY = std::ceil(std::abs(goal[1] - start[1]) / step);
    int nZ = std::ceil(std::abs(goal[2] - start[2]) / step);
    int n = std::max({nX, nY, nZ});

    std::vector<double> xValues = linspace(start[0], goal[0], n);
    std::vector<double> yValues = linspace(start[1], goal[1], n);
    std::vector<double> zValues = linspace(start[2], goal[2], n);

    for (int i = 0; i < n; ++i) {
        file << std::fixed << std::setprecision(4) << xValues[i] << " " << yValues[i] << " " << zValues[i]
             << " 1.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 15.7777 15.7777 15.7777 15.7777\n";
    }

    file.close();
}

void addTrajectoryFile(const std::vector<double>& start, const std::vector<double>& goal, double step, const std::string& filename) {
    std::ofstream file(filename, std::ios::app);

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    int nX = std::ceil(std::abs(goal[0] - start[0]) / step);
    int nY = std::ceil(std::abs(goal[1] - start[1]) / step);
    int nZ = std::ceil(std::abs(goal[2] - start[2]) / step);
    int n = std::max({nX, nY, nZ});

    std::vector<double> xValues = linspace(start[0], goal[0], n);
    std::vector<double> yValues = linspace(start[1], goal[1], n);
    std::vector<double> zValues = linspace(start[2], goal[2], n);

    for (int i = 0; i < n; ++i) {
        file << std::fixed << std::setprecision(4) << xValues[i] << " " << yValues[i] << " " << zValues[i]
             << " 1.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 15.7777 15.7777 15.7777 15.7777\n";
    }

    file.close();
}

std::string callGenerateMapService() {
    ros::NodeHandle nh;
    ros::ServiceClient generateMapClient = nh.serviceClient<std_srvs::Trigger>("generate_map");

    std_srvs::Trigger trigger;
    if (generateMapClient.call(trigger)) {
        return trigger.response.message;
    } else {
        ROS_ERROR("Failed to call generate_map service");
        return "";
    }
}

nav_msgs::Path callRRTPlannerService(geometry_msgs::PoseStamped& startPose,
                                     geometry_msgs::PoseStamped& goalPose,
                                     std::vector<int>& obstacleIds,
                                     std::string& mapFilename) {
    ros::NodeHandle nh;
    ros::ServiceClient rrtPlannerClient = nh.serviceClient<rrt_planner::GetRRTPlan>("rrt_planner_server");
    std_msgs::Int16MultiArray obstacleIdsMsg;
    obstacleIdsMsg.data.clear(); // Ensure the data field is empty
    std_msgs::String mapFilenameMsg;
    mapFilenameMsg.data = mapFilename;
    // Populate the data field with obstacleIds
    obstacleIdsMsg.data.insert(obstacleIdsMsg.data.end(), obstacleIds.begin(), obstacleIds.end());
    rrt_planner::GetRRTPlan srv;
    srv.request.start = startPose;
    srv.request.goal = goalPose;
    srv.request.obstacle_ids.data = obstacleIdsMsg.data;
    srv.request.map_filename.data = mapFilenameMsg.data;

    if (rrtPlannerClient.call(srv)) {
        return srv.response.plan;
    } else {
        ROS_ERROR("Failed to call rrt_planner_server service");
        return nav_msgs::Path();  // Return an empty path in case of failure
    }
}