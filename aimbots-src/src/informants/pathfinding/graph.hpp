#pragma once
#include <unordered_map>
#include "utils/common_types.hpp"
#include <cmath>
#include <array>
#include <queue>
#include <functional>

//I stole all of this because I have not taken DSA
//https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp

//represents a location in a graph as apposed to an ID, because I think this is more scalable, this may be a bad idea


//defines hash function for Vector2i so unordered map can work
//reference: https://en.cppreference.com/w/cpp/utility/hash
template <>
struct std::hash<modm::Vector2i>
{
    std::size_t operator()(const modm::Vector2i& k) const
    {
        std::size_t h1 = std::hash<int>{}(k[0]);
        std::size_t h2 = std::hash<int>{}(k[0]);
        return (h1 ^ (h2 << 1)); //uses built in hash function on both ints, then combines the hashes uses bitwise magic i dont understand
    }
};

struct WeightedSquareGraph {
    //coordinate system
    //   ^ +y
    //   |
    //  origin --> +x
    std::unordered_map<modm::Vector2i, modm::Vector2i> edges;
    double dx;
    double dy;
    int graph_width;
    int graph_height;
    double width_meters;
    double height_meters;

    // He was a modm::Vector2i. She was a Graph_loc. Will they fall in love?
    WeightedSquareGraph (double width_meters, double height_meters, double precision_cm) {
        this->width_meters = width_meters;
        this->height_meters = height_meters;
        graph_width = std::ceil(width_meters/(precision_cm/100)); //width of square grid measured in nodes
        graph_height = std::ceil(height_meters/(precision_cm/100)); //height of square grid measured in nodes
        dx = width_meters/graph_width; //this grid is evenly spaced
        dy = height_meters/graph_height;

        for (int i = 0; i < graph_width; i++){
            for (int j = 0; j < graph_height; j++){
                modm::Vector2i loc = {i, j};
                modm::Vector2i pos = {i*dx, j*dy};
                edges[loc] = pos;
            }
        }
    }

    void with_obstacle(modm::Vector2i bottom_left_meters, modm::Vector2i top_right_meters){
        //removes all nodes within a given square area. Will remove nodes that dont exist. Rounds to nearest node so will slightly under or overestimate area. 
        modm::Vector2i bottom_left = snap_to_grid(bottom_left_meters);
        modm::Vector2i top_right = snap_to_grid(top_right_meters);
        for (int i = bottom_left[0]; i < top_right[0]; i++){
            for (int j = bottom_left[1]; j < top_right[1]; j++){
                modm::Vector2i to_erase = {i, j};
                edges.erase(to_erase);
            }
        }
    }

    modm::Vector2i snap_to_grid (modm::Vector2i pos) {
        // gets nearest hypothetical node from a given position, no guarantee that node will exist
        int x = std::round(pos[0]/dx);
        int y = std::round(pos[1]/dy);
        modm::Vector2i loc = {x, y};
        return loc;
    }

    void get_neighbors(modm::Vector2i loc, std::array<modm::Vector2i, 8>* neighbors) {
        // Generates up all neighbors on the grid relative to a specific node
        //   Up to 4 orthogonal & 4 diagonal, similar to Minesweeper
        int k = 0;
        for (int i = -1; i < 2; i++){
            for (int j = -1; j <2; j++){
                if (j == 0 && i == 0) {continue;}
                modm::Vector2i neighbor;
                neighbor[0] = i + loc[0];
                neighbor[1] = j + loc[1];
                (*neighbors)[k] = neighbor;
                
                k++;
            }
        }
        }
    
    // Returns the distance between two nodes
    double get_cost(modm::Vector2i from_node, modm::Vector2i to_node){
        return std::sqrt(pow2(edges[to_node][0] - edges[from_node][0]) + pow2(edges[to_node][0] - edges[from_node][0])); // tengo bomba :steam_happy:
    }

    double heuristic (modm::Vector2i other_node, modm::Vector2i goal) {
        return get_cost(other_node, goal);
    }


    }; // boo

//completely copied from redlob. Yoink.
template<typename T, typename priority_t>
struct PriorityQueue {
  typedef std::pair<priority_t, T> PQElement;
  std::priority_queue<PQElement, std::vector<PQElement>,
                 std::greater<PQElement>> elements;

  inline bool empty() const {
     return elements.empty();
  }

  inline void put(T item, priority_t priority) {
    elements.emplace(priority, item);
  }

  T get() {
    T best_item = elements.top().second;
    elements.pop();
    return best_item;
  }
};




template<typename Location, typename Graph>

void a_star_search
  (WeightedSquareGraph graph,
   modm::Vector2i start,
   modm::Vector2i goal,
   std::unordered_map<modm::Vector2i, modm::Vector2i>& came_from,
   std::unordered_map<modm::Vector2i, double>& cost_so_far)
{
  PriorityQueue<modm::Vector2i, double> frontier;
  frontier.put(start, 0);

  came_from[start] = start;
  cost_so_far[start] = 0;
  
  while (!frontier.empty()) {
    modm::Vector2i current = frontier.get();

    if (current == goal) {
      break;
    }
    std::array<modm::Vector2i, 8> neighbors;
    for (Location next : graph.get_neighbors(current, &neighbors)) {
      double new_cost = cost_so_far[current] + graph.get_cost(current, next);
      if (cost_so_far.find(next) == cost_so_far.end()
          || new_cost < cost_so_far[next]) {
        cost_so_far[next] = new_cost;
        double priority = new_cost + heuristic(next, goal);
        frontier.put(next, priority);
        came_from[next] = current;
      }
    }
  }
}



