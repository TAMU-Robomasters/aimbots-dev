#include "graph.hpp"
#include <iostream>

WeightedSquareGraph::WeightedSquareGraph(float width_meters, float height_meters, float precision_meters)
:   edges(),
    WIDTH_NODES(ceilf(width_meters/precision_meters)), 
    HEIGHT_NODES(ceilf(height_meters/precision_meters)),
    WIDTH_METERS(width_meters),
    HEIGHT_METERS(height_meters),
    NODE_WIDTH_METERS(WIDTH_METERS/WIDTH_NODES),
    NODE_HEIGHT_METERS(HEIGHT_METERS/HEIGHT_NODES)
{
    // Initialize cell coordinates and meter coordinates
    for (short x = 0; x < WIDTH_NODES; x++)
        for (short y = 0; y < HEIGHT_NODES; y++)
            edges[{x, y}] = {x*NODE_WIDTH_METERS, y*NODE_HEIGHT_METERS};
}

Vector2f WeightedSquareGraph::get_location(const Vector2i pos){
    return edges.at(pos);
}

Vector2i WeightedSquareGraph::snap_to_grid(const Vector2f& pos_meters) const
{
    return {
        (short) round(pos_meters[0]/NODE_WIDTH_METERS),
        (short) round(pos_meters[1]/NODE_HEIGHT_METERS)
    };
}

void WeightedSquareGraph::remove_region(const Vector2f& bottom_left_meters, const Vector2f& top_right_meters)
{
    auto BL = snap_to_grid(bottom_left_meters);
    auto TR = snap_to_grid(top_right_meters);

    auto BL_x = BL[0];
    auto BL_y = BL[1];
    auto TR_x = TR[0];
    auto TR_y = TR[1];

    for (auto x = BL_x; x <= TR_x; x++)
        for (auto y = BL_y; y <= TR_y; y++)
            edges.erase({x, y});
}

vector<Vector2i> WeightedSquareGraph::get_neighbors(const Vector2i& node) const
{
    vector<Vector2i> neighbors;
    size_t index = 0;
    
    for (short x = -1; x <= 1; x++)
    {
        for (short y = -1; y <= 1; y++)
        {
            Vector2i offset {x, y};
            Vector2i neighbor = node + offset;

            bool isCenter = x == 0 && y == 0;
            bool isUntraversable = edges.find(neighbor) == edges.end();

            if (isCenter || isUntraversable) continue;

            neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

float WeightedSquareGraph::get_distance_meters(const Vector2i& a, const Vector2i& b) const
{
    auto ax = a[0];
    auto ay = a[1];
    auto bx = b[0];
    auto by = b[1];

    auto dx = (ax - bx) * NODE_WIDTH_METERS;
    auto dy = (ay - by) * NODE_HEIGHT_METERS;

    return sqrtf(dx*dx + dy*dy);
}

float WeightedSquareGraph::heuristic(const Vector2i& a, const Vector2i& b) const
{
    return get_distance_meters(a, b);
}

void a_star_search
  (const WeightedSquareGraph& graph,
   const Vector2i& start,
   const Vector2i& goal,
   unordered_map<Vector2i, Vector2i>& came_from,
   unordered_map<Vector2i, float>& aggregate_cost)
{
    PriorityQueue<Vector2i, float> frontier;
    frontier.put(start, 0);

    came_from[start] = start;
    aggregate_cost[start] = 0;
  
    while (!frontier.empty()) {
        Vector2i current = frontier.get();
        auto all_neighbors = graph.get_neighbors(current);

        if (current == goal) break;

        for (auto neighbor : all_neighbors)
        {
            float added_cost = graph.get_distance_meters(current, neighbor);
            float new_cost = aggregate_cost[current] + added_cost;
            
            if (aggregate_cost.find(neighbor) == aggregate_cost.end() || new_cost < aggregate_cost[neighbor])
            {
                aggregate_cost[neighbor] = new_cost;
                double priority = new_cost + graph.heuristic(neighbor, goal);
                frontier.put(neighbor, priority);

                came_from[neighbor] = current;
            }
        }
    }
}

// Translates the unordered map -> vector<Vector2f> of all nodes
vector<Vector2f> WeightedSquareGraph::get_path(Vector2i start, Vector2i goal){
    unordered_map<Vector2i, Vector2i> came_from;
    unordered_map<Vector2i, float> aggregate_cost;
    a_star_search(*this, start, goal, came_from, aggregate_cost);

    vector<Vector2f> path;
    Vector2i current_vector = goal;
    path.insert(path.begin(), this->get_location(current_vector));
    while (current_vector != start){
        current_vector = came_from.at(current_vector);
        path.insert(path.begin(), this->get_location(current_vector));
    }
    return path;
}