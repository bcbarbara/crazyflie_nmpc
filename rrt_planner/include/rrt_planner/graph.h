#include <unordered_map>
#include <vector>

using namespace std;

class Graph{
    public:
        Graph(int nDOFs, double* armstart_anglesV_rad, double* armgoal_anglesV_rad)
        {
            numDOFS = nDOFs;
            nodes.clear();
            edges.clear();
            g.clear();
            int start_node_id = addNode(armstart_anglesV_rad);
            start_angles = armstart_anglesV_rad;
            goal_angles = armgoal_anglesV_rad;
        }

        int addNode(double* angles)
        {
            int node_id = nodes.size();
            nodes.push_back(angles);

            // placeholder edge for given input node
            vector<int> blank;
            edges.push_back(blank);
            
            return node_id;
        }

        void addEdge(int n1, int n2)
        {
            edges[n1].push_back(n2);
            edges[n2].push_back(n1);
        }

        int numDOFS;
        vector<double* > nodes;
        vector<vector<int>> edges;
        unordered_map<int, double> g;
        int numofDOFs;
        double* start_angles;
        double* goal_angles;	

};