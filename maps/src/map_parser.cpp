#include <maps/map_parser.h>

Map::Map(const std::string& filename)
{
    // Load the YAML file
    YAML::Node config = YAML::LoadFile(filename);

    // Read the number of obstacles
    num_obstacles_ = config["num_obstacles"].as<int>();

    // Read the map bounds
    x_min_ = config["x_min"].as<double>();
    x_max_ = config["x_max"].as<double>();
    y_min_ = config["y_min"].as<double>();
    y_max_ = config["y_max"].as<double>();

    // Read the obstacles
    YAML::Node obstacles = config["obstacles"];
    for (int i = 0; i < num_obstacles_; i++)
    {
        YAML::Node obstacle = obstacles[i];

        // Read the obstacle properties
        double x = obstacle["x"].as<double>();
        double y = obstacle["y"].as<double>();
        double r = obstacle["r"].as<double>();

        // Add the obstacle to the list
        obstacles_.push_back({x, y, r});
    }
}

std::vector<Obstacle> Map::find_obstacles(const std::vector<int>& obstacle_ids) const
{
    std::vector<Obstacle> result;
    for (int id : obstacle_ids)
    {
        if (id < 0 || id >= num_obstacles_)
        {
            std::cerr << "Invalid obstacle ID " << id << std::endl;
            continue;
        }
        result.push_back(obstacles_[id]);
    }
    return result;
}