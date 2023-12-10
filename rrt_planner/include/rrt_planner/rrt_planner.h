#include <iostream>
#include <cstring>
#include <math.h>
#include <vector>

#include "tree.h"
#include <maps/map_parser.h>

#define TRAPPED 0
#define ADVANCED 1
#define REACHED 2

using std::vector;
using std::cout;
using std::endl;

class RRTplanner{
    public:
        RRTplanner(Map* map,
                    double* start_point,
                    double* goal_point,
                    int numofDOFs,
                    std::vector<int>* obstacle_ids,
                    double*** plan,
                    int* planlength,
                    double epsilon,
                    double sample_resolution);

        void run(Tree* tree);
        Tree initTree();

        double epsilon;
        double sample_resolution;

    private:
        Map* map;
        double* start_point;
        double* goal_point;
        int numofDOFs;
        std::vector<int>* obstacle_ids;
        double*** plan;
        int* planlength;
        int K;
        int k;
        double goal_bias;
        bool target_found;

        int extendTree(Tree* tree, double* q, int numofDOFs, const Map* map, const std::vector<int>* obstacle_ids);
        int newConfig(double* q, double* q_near, double* q_new, int numofDOFs, const Map* map, const std::vector<int>* obstacle_ids);
        int isAtGoal(double* q, double* qgoal, int numofDOFs);
};

bool isValid(const Map* map, double x, double y, const std::vector<int>* obstacle_ids)
{
    for (int obstacle_id : *obstacle_ids)
    {
        if (obstacle_id < 0 || obstacle_id >= map->num_obstacles())
        {
            std::cerr << "Invalid obstacle ID: " << obstacle_id << std::endl;
            continue;
        }

        const auto& obstacle = map->obstacles()[obstacle_id];
        double dx = obstacle.x - x;
        double dy = obstacle.y - y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq <= obstacle.r * obstacle.r)
        {
            // The point is inside the obstacle
            return false;
        }
    }
    // The point is not inside any obstacles
    return true;
}

// function to generate random point in the map
double* generateRandomPoint(const Map* map, const std::vector<int>* obstacle_ids)
{
    double* point = new double[2];    
    do
    {
        point[0] = map->x_min() + (map->x_max() - map->x_min()) * (double)rand() / RAND_MAX;
        point[1] = map->y_min() + (map->y_max() - map->y_min()) * (double)rand() / RAND_MAX;
    } while (!isValid(map, point[0], point[1], obstacle_ids));
    return point;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            return false;
        }
    }
    return true;
}

double distNorm(double* q1, double* q2, int numofDOFs)
{
    double dist = 0;
	for(int i=0; i<numofDOFs; i++)
	{
		dist = dist + (q1[i]-q2[i]) * (q1[i]-q2[i]);
	}
	return sqrt(dist);
}