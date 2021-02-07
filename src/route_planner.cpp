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

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors) 
    {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}

bool RoutePlanner::Compare(RouteModel::Node *node_a, RouteModel::Node *node_b)
{
    return (node_a->g_value + node_a->h_value) > (node_b->g_value + node_b->h_value);
}

// Sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), RoutePlanner::Compare);
    RouteModel::Node *node = open_list.back();
    open_list.pop_back();
    return node;
}


// Construct the path found by following each nodes parent
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while(current_node != start_node) 
    {
        distance += + current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // initialize starting node
    start_node->h_value = CalculateHValue(start_node);
    start_node->g_value = 0;
    start_node->visited = true;
    open_list.push_back(start_node);
    current_node = start_node;

    while (open_list.size() > 0) 
    {
        current_node = NextNode();
        AddNeighbors(current_node);

        if (current_node->x == end_node->x && current_node->y == end_node->y)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
    }

}
