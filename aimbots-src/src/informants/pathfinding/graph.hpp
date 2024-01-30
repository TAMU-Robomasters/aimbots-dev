#pragma once
#include <unordered_map>
#include "utils/common_types.hpp"
#include <cmath>
#include <vector>

using namespace std;

//I stole all of this because I have not taken DSA
//https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp

//represents a location in a graph as apposed to an ID, because I think this is more scalable, this may be a bad idea


struct Graph_Loc { 
    int x;
    int y;
};

struct WeightedSquareGraph {
    unordered_map<Graph_Loc, Graph_Loc> edges;

    Graph_Loc* get_neighbors(Graph_Loc loc) {
        // Generates 8 neighbors on the grid
        //   4 orthogonal & 4 diagonal, similar to Minesweeper
        vector<Graph_Loc> neighbors;
        int x = loc.x;
        int y = loc.y;
        int k = 0;
        for (int i = -1; i < 2; i++){
            for (int j = -1; j <2; j++){
                if (j == 0 && i == 0) {continue;}
                Graph_Loc neighbor;
                neighbor.x = i + x;
                neighbor.y = j + y;
                neighbors[k] = neighbor;
                k++;
            }
        }
            return &neighbors;
        }
    
    // Returns the distance between two nodes
    double get_cost(Graph_loc from_node, Graph_Loc to_node){
        return std::sqrt(pow2(to_node.x - from_node.x) + pow2(to_node.y - from_node.y));
    }
        

    }; // boo
