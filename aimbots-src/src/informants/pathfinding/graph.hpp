#pragma once
using std::vector, std::unordered_map;
//I stole all of this because I have not taken DSA
//https://www.redblobgames.com/pathfinding/a-star/implementation.html#cpp

//represents a location in a graph as apposed to an ID, because I think this is more scalable, this may be a bad idea
struct Graph_Loc { 
    int x;
    int y;
};
struct WeightedSquareGraph {
    unordered_map<Graph_Loc, vector<Graph_Loc> > edges;

    vector<Graph_Loc> neighbors(Graph_loc loc) {
        return edges[loc]
    }
};