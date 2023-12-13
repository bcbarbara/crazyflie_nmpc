#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>

struct Obstacle
{
    double x;
    double y;
    double r;
};

class Map
{
public:
    Map(const std::string& filename);

    int num_obstacles() const { return num_obstacles_; }
    double x_min() const { return x_min_; }
    double x_max() const { return x_max_; }
    double y_min() const { return y_min_; }
    double y_max() const { return y_max_; }

    const std::vector<Obstacle>& obstacles() const { return obstacles_; }

    std::vector<Obstacle> find_obstacles(const std::vector<int>& obstacle_ids) const;

private:
    int num_obstacles_;
    std::vector<Obstacle> obstacles_;
    double x_min_;
    double x_max_;
    double y_min_;
    double y_max_;
};

// int main()
// {
//     Map config("config.yaml");

//     // Print all obstacles
//     std::cout << "Number of obstacles: " << config.num_obstacles() << std::endl;
//     for (int i = 0; i < config.num_obstacles(); i++)
//     {
//         const Map::Obstacle& obstacle = config.obstacles()[i];
//         std::cout << "Obstacle " << i << ":" << std::endl;
//         std::cout << "  x = " << obstacle.x << std::endl;
//         std::cout << "  y = " << obstacle.y << std::endl;
//         std::cout << "  r = " << obstacle.r << std::endl;
//     }

//     // Find specific obstacles
//     std::vector<int> ids_to_find = {0, 2, 4};
//     std::vector<Map::Obstacle> found_obstacles = config.find_obstacles(ids_to_find);
//     for (const Map::Obstacle& obstacle : found_obstacles)
//     {
//         std::cout << "Found obstacle:" << std::endl;
//         std::cout << "  x = " << obstacle.x << std::endl;
//         std::cout << "  y = " << obstacle.y << std::endl;
//         std::cout << "  r = " << obstacle.r << std::endl;
//     }

//     return 0;
// }
