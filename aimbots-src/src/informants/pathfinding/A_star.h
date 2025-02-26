#pragma once
#include <unordered_map>
#include <vector>
#include <functional>
#include "prio_queue.h"
#include <iostream>
using std::unordered_map, std::vector, std::pair;
template<typename NodeType>
class A_star {
    private:
    std::function<double(NodeType*, NodeType*)> heuristic;
    std::function<double(NodeType*, NodeType*)> travel_cost;

    public:
    unordered_map<NodeType*, vector<pair<NodeType*, double>>> neighbors;

        A_star(unordered_map<NodeType*, vector<pair<NodeType*, double>>> neighbors,
           std::function<double(NodeType*, NodeType*)> heuristic,
           std::function<double(NodeType*, NodeType*)> travel_cost)
        : neighbors(neighbors), heuristic(heuristic), travel_cost(travel_cost) {}

        A_star(A_star<NodeType>& other) : neighbors(other.neighbors), heuristic(other.heuristic), travel_cost(other.travel_cost)
        {}

        A_star& operator=(A_star<NodeType>& other) {
            if (this != &other) {
                neighbors = other.neighbors;
                heuristic = other.heuristic;
                travel_cost = other.travel_cost;
            }
            return *this;

        }

        vector<NodeType*> search(NodeType* start, NodeType* goal) {
            vector<NodeType*> path;
            PrioQueue<NodeType*, double> open_set;

            unordered_map<NodeType*, NodeType*> came_from;
            unordered_map<NodeType*, double> gScore; //cost of cheapest path from start to node
            gScore[start] = 0;

            unordered_map<NodeType*, double> fScore; //estimated cost of the path through node from start to goal
            fScore[start] = heuristic(start, goal);
            open_set.put(start, fScore[start]);

            while(!open_set.empty()) {
                NodeType* curr = open_set.get();
                if (curr == goal) {
                    path.push_back(curr);
                    while (came_from.find(curr) != came_from.end()) {
                        curr = came_from[curr];
                        path.insert(path.begin(), curr);
                    }
                    return path;
                }

                for (auto it = neighbors[curr].begin(); it != neighbors[curr].end(); it++) {
                    double tentative_gScore = gScore[curr] + it->second;
                    if (gScore.find(it->first) == gScore.end() || tentative_gScore < gScore[it->first]) {
                        came_from[it->first] = curr;
                        gScore[it->first] = tentative_gScore;
                        fScore[it->first] = tentative_gScore + heuristic(it->first, goal);
                        open_set.put(it->first, fScore[it->first]);
                    }
                }
            }
            return path;
        }
};

