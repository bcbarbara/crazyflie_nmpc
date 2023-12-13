#include <unordered_map>
#include <vector>
#include <math.h>

using namespace std;

class Tree{
    public:
        Tree(int nDOFs, double* start_node);
        int addNode(double* sample_node_rad);
        void addEdge(int child, int parent);
        double* getNode(int node_id);
        int getParentID(int id);
        int getNewNodeID();
        int nearestNeighbour(double* node);
        double distBetweenNodes(double* node1, double* node2);

        vector<double*> nodes;
        unordered_map<int, int> edges;    // child, parent
        unordered_map<int, double> costs; // idx, cost
        int numDOFS;
};


Tree::Tree(int nDOFs, double* start_node)
{
    numDOFS = nDOFs;
    nodes.clear();
    edges.clear();
    costs.clear();
    nodes.push_back(start_node);
    costs[0] = 0.0;
}

int Tree::addNode(double* sample_node_rad)
{
    int node_id = nodes.size();
    nodes.push_back(sample_node_rad);
    return node_id;
}

void Tree::addEdge(int child, int parent)
{
    edges[child] = parent;
}

double* Tree::getNode(int node_id)
{
    return nodes[node_id];
}

int Tree::getParentID(int id)
{
    return edges[id];
}

int Tree::getNewNodeID()
{
    return nodes.size()-1;
}

int Tree::nearestNeighbour(double* node)
{
    double min_dist = numeric_limits<double>::infinity();
    int nearest_node_id = 0;
    for (int i = 0; i < nodes.size(); i++) {
        double dist = 0;
        for (int j = 0; j < numDOFS; j++) {
            dist += (nodes[i][j] - node[j]) * (nodes[i][j] - node[j]);
        }
        dist = sqrt(dist);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_node_id = i;
        }
    }
    return nearest_node_id;
}

double Tree::distBetweenNodes(double* node1, double* node2)
{
    double dist = 0;
    for (int i = 0; i < numDOFS; i++) {
        dist += (node1[i] - node2[i]) * (node1[i] - node2[i]);
    }
    dist = sqrt(dist);
    return dist;
}