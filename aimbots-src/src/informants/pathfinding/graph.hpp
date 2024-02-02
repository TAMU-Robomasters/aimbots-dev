#pragma once
#include <unordered_map>
#include "utils/common_types.hpp"
#include <cmath>
#include <array>

//I stole all of this because I have not taken DSA
//https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp

//represents a location in a graph as apposed to an ID, because I think this is more scalable, this may be a bad idea


struct WeightedSquareGraph {
    std::unordered_map<modm::Vector2i, modm::Vector2i> edges;

    // He was a modm::Vector2i. She was a Graph_loc. Will they fall in love?

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
   /* double get_cost(Vector<> from_node, Graph_Loc to_node){
        return std::sqrt(pow2(to_node.x - from_node.x) + pow2(to_node.y - from_node.y)); // tengo bomba :steam_happy:
    }
    */

    }; // boo
