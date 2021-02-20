#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

/**
 * Calculates the H Value for the given node.
 * \param node The node in the OpenStreetMap.
 * \return The calculated H Value of the node.
*/
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

/**
 * Expands the current node by adding its unvisited neighbour nodes to the open list.
 * \param current_node The node in the OpenStreetMap.
*/
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
	for(auto& neighbor_node : current_node->neighbors){
		if(neighbor_node->visited)
			continue;
		neighbor_node->parent = current_node;
		neighbor_node->g_value = current_node->g_value + neighbor_node->distance(*current_node);
		neighbor_node->h_value = CalculateHValue(neighbor_node);
		neighbor_node->visited = true;
		open_list.push_back(neighbor_node);		
	}
}

/**
 * Compares two given nodes according to sum of the h value and g value.
 * \param a The node in the OpenStreetMap.
 * \param b The node in the OpenStreetMap.
*/
bool Compare(const RouteModel::Node *a, const RouteModel::Node *b){
	return a->g_value + a->h_value > b->g_value + b->h_value;
}

/**
 * Sorts the open list according to the sum of the h value and g value 
 * and returns the next node with the lowest sum.
 * \return Next node to expand during the A* search. 
 */
RouteModel::Node *RoutePlanner::NextNode() {
	sort(open_list.begin(),open_list.end(), [] (const RouteModel::Node* a, const RouteModel::Node* b) {return a->g_value + a->h_value > b->g_value + b->h_value;});
	RouteModel::Node *next_node = open_list.back();
	open_list.pop_back();
	return next_node;
}

/**
 * Constructs the final path found from the A* Search.
 * \param current_node The current (final) node in the OpenStreetMap.
 * \return The list of nodes on the path from the start node to the given current (final) node.
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	
	while(!(current_node == start_node)){
		distance += current_node->distance(*current_node->parent);
		path_found.emplace_back(*current_node);
		current_node = current_node->parent;
	}
	path_found.emplace_back(*current_node);
	reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

/**
 * A* Search algorithm to find the path between 2 points on OpenStreetMap.
 * 
 * The final path is stored in the m_Model.path attribute before the method exits.
*/
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
	
	open_list.push_back(start_node);
	start_node->visited = true;
	
	while(!open_list.empty()){
		current_node = NextNode();
		if(current_node == end_node){
			m_Model.path = ConstructFinalPath(current_node);
			open_list.clear(); 
			return;
		}
		else{
			AddNeighbors(current_node);
		}
	}
}