/*
	ROS-agnostic RRT planner
*/

#include <rrt_planner/rrt_planner.h>

// RRT planner contructor
RRTplanner::RRTplanner(
			Map* map_,
			double* start_point_,
			double* goal_point_,
            int numofDOFs_,
			std::vector<int>* obstacle_ids_,
            double*** plan_,
            int* planlength_,
			double epsilon_,
			double sample_resolution_)
{
	map = map_;
	start_point = start_point_;
	goal_point = goal_point_;
	numofDOFs = numofDOFs_;
	obstacle_ids = obstacle_ids_;
	plan = plan_;
	planlength = planlength_;
	epsilon = epsilon_;
	sample_resolution = sample_resolution_;

	//no plan by default
	*plan = NULL;
	*planlength = 0;

	// Number of samples
	K = 1000000;
	k = 0;

	// Goal bias
	goal_bias = 0.05;

	// target found flag
	target_found = 0;
	
}

// Extend tree towards the sample node
int RRTplanner::extendTree(Tree* tree, double* q, int numofDOFs, const Map* map, const std::vector<int>* obstacle_ids)
{
    int status = TRAPPED;
    int q_near_id = tree->nearestNeighbour(q);
    double* q_near = tree->getNode(q_near_id);
    double* q_new = (double*)malloc(numofDOFs*sizeof(double));
    if(newConfig(q, q_near, q_new, numofDOFs, map, obstacle_ids)) 
    {
        int q_new_id = tree->addNode(q_new);
        tree->addEdge(q_new_id, q_near_id);

		if(isAtGoal(q_new, q, numofDOFs))
			status = REACHED;
		else
			status = ADVANCED;
    }
    return status;
}

int RRTplanner::newConfig(double* q, double* q_near, double* q_new, int numofDOFs, const Map* map, const std::vector<int>* obstacle_ids) 
{
    // move by EPSILON towards q from q_near and return q_new

    double dist = 0;
    int success = 0;
    int i,j;

	// set distance as the sum of the squares of the differences calculated here
	dist = distNorm(q, q_near, numofDOFs);

    int numofsamples = (int)(dist/(sample_resolution));

    double* tmp_point = (double*)malloc(numofDOFs*sizeof(double));

    for (i = 1; i < numofsamples; i++)
    {
    	for(j = 0; j<numofDOFs; j++)
    	{
    		tmp_point[j] = q_near[j] + ((double)(i)/(numofsamples-1))*(q[j] - q_near[j]);
    	}
		double dist_temp = distNorm(tmp_point, q_near, numofDOFs);
    	if(isValid(map, tmp_point[0], tmp_point[1], obstacle_ids) && dist_temp < epsilon)
    	{
    		memcpy(q_new, tmp_point, numofDOFs*sizeof(double));
            success = 1;
    	}
    	else
    	{break;}
    }

    free(tmp_point);
    return success;

}

int RRTplanner::isAtGoal(double* q, double* qgoal, int numofDOFs)
{
    int reached = 0;
    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(q[j] - qgoal[j]))
            distance = fabs(q[j] - qgoal[j]);
    }
    int numofsamples = (int)(distance/(sample_resolution));
    if(numofsamples < 2){
        reached = 1;
    }
    return reached;
}


void RRTplanner::run(Tree* tree){

	// Run RRT algorithm
	while(!target_found && k < K)
	{
		k++;
		
		double* q_rand = (double*)malloc(numofDOFs*sizeof(double));
		// Sample a random node with goal bias
		if((double)rand() / RAND_MAX < goal_bias)
		{
			// Sample goal node
			// cout << "Sampling goal node" << endl;
			q_rand = goal_point;
		}
		else
		{
			// Sample random node
			// cout << "Sampling random node" << endl;
			q_rand = generateRandomPoint(map, obstacle_ids);
		}
		
		// Extend the tree towards the sample node
		if(extendTree(tree, q_rand, numofDOFs, map, obstacle_ids) == TRAPPED)
		{
			continue;
		}

		int q_new_id = tree->getNewNodeID();
		double* q_new = tree->getNode(q_new_id);

		// Check if the new node is close to the goal
		if (isAtGoal(q_new, goal_point, numofDOFs))
		{
			target_found = 1;
			cout << "Target found" << endl;
			cout << "Number of samples: " << k << endl;
			if (!equalDoubleArrays(q_new, goal_point, numofDOFs))
			{
				// we are very near to goal but not quite there
				// so add the goal to the tree
				int node_id = tree->addNode(goal_point);
				tree->addEdge(node_id,q_new_id);
			}
		}
	}

	// If target is found, construct and return the plan
	if(target_found)
	{
		int q_new_id = tree->getNewNodeID();
		double* q_new = tree->getNode(q_new_id);
		vector<int> path;
		int next_id = q_new_id;
		while (next_id != 0) {
			path.insert(path.begin(), next_id);
			next_id = tree->getParentID(next_id);
		}
		path.insert(path.begin(), 0);
		*planlength = path.size();
		*plan = (double**) malloc(path.size()*sizeof(double*));
		for(int i=0; i<path.size(); i++)
		{
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
			memcpy((*plan)[i], tree->getNode(path[i]), numofDOFs*sizeof(double));
		}
	}
	else
	{
		printf("Target not found\n");
	}

}

Tree RRTplanner::initTree()
{
	Tree tree(numofDOFs, start_point);
	return tree;
}


int main() {

	Map* map;
	double* start_point;
	double* goal_point;
	int numOfDOFs;
	std::vector<int>* obstacle_ids;

	Map temp_map = Map("map.yaml");
	map = &temp_map;
	numOfDOFs = 2;
	start_point = (double*)malloc(numOfDOFs*sizeof(double));
	goal_point = (double*)malloc(numOfDOFs*sizeof(double));
	start_point[0] = 0.0;
	start_point[1] = -2.5;
	goal_point[0] = 0.0;
	goal_point[1] = 9.0;
	std::vector<int> ids_to_find = {0};
	obstacle_ids = &ids_to_find;

	double epsilon = 0.2;
	double sample_resolution = 0.1;

	if(!isValid(map, start_point[0], start_point[1], obstacle_ids)){
		throw runtime_error("Invalid start configuration!\n");
	}
	if(!isValid(map, goal_point[0], goal_point[1], obstacle_ids)) {
		throw runtime_error("Invalid goal configuration!\n");
	}

	double** plan = NULL;
	int planlength = 0;

	// measure time
	clock_t begin = clock();

	// Call the planner
	RRTplanner rrt(map, start_point, goal_point, numOfDOFs, obstacle_ids, &plan, &planlength, epsilon, sample_resolution);

	Tree tree = rrt.initTree();
	rrt.run(&tree);

	// measure time
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Time taken: " << elapsed_secs << " s" << endl;

	if (!equalDoubleArrays(plan[0], start_point, numOfDOFs))
	{
		throw std::runtime_error("Start position not matching");
	}
	if (!equalDoubleArrays(plan[planlength-1], goal_point, numOfDOFs))
	{
		throw std::runtime_error("Goal position not matching");
	}

	// print the plan
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			cout << plan[i][k] << ",";
		}
		cout << endl;
	}
	cout << endl;

}
