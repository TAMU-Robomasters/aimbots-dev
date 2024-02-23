#pragma once

#include <unordered_map>
#include "priority_queue.hpp"
#include "utils/common_types.hpp"
#include <cmath>
#include <array>
#include <queue>
#include <functional>
//#include "Ramer_Douglas_peucker.hpp"

//I stole all of this because I have not taken DSA
//https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp

//represents a location in a graph as apposed to an ID, because I think this is more scalable, this may be a bad idea

using namespace std;
using namespace modm;

//defines hash function for Vector2i so unordered map can work
//reference: https://en.cppreference.com/w/cpp/utility/hash
template <>
struct std::hash<Vector2i>
{
    size_t operator()(const Vector2i& k) const
    {
        size_t h1 = hash<int>{}(k[0]);
        size_t h2 = hash<int>{}(k[0]);
        return (h1 ^ (h2 << 1)); // Uses built in hash function on both ints, then combines the hashes uses bitwise magic i dont understand
    }
};

struct WeightedSquareGraph {
    //coordinate system
    //   ^ +y
    //   |
    //  origin --> +x
    unordered_map<Vector2i, Vector2f> edges;

    const int WIDTH_NODES;  // width of grid in nodes
    const int HEIGHT_NODES; // height of grid in nodes

    const float WIDTH_METERS;  // width of grid in meters
    const float HEIGHT_METERS; // height of grid in meters

    const float NODE_WIDTH_METERS;  // width of each node in meters
    const float NODE_HEIGHT_METERS; // height of each node in meters

    // He was a modm::Vector2i. She was a Graph_loc. Will they fall in love?

    WeightedSquareGraph(float width_meters, float height_meters, float precision_meters);

    // Gets nearest hypothetical node from a given position, no guarantee that node will exist
    Vector2i snap_to_grid(const Vector2f&) const;

    // Removes all nodes within a given rectangle area. Will remove nodes that dont exist. 
    // Rounds to nearest node so will slightly under or overestimate area. 
    void remove_region(const Vector2f& bottom_left_meters, const Vector2f& top_right_meters);

    // Gets all 8 grid neighbors of the given node
    array<Vector2i, 8> get_neighbors(const Vector2i&) const;
    
    // Returns the distance between two nodes
    float get_distance_meters(const Vector2i&, const Vector2i&) const;

    // Calculates the heuristic cost of travelling between two nodes
    float heuristic(const Vector2i&, const Vector2i&) const;
};