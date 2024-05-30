#pragma once
#include <unordered_map>
#include <vector>
#include <functional>
#include "prioQueue.h"
using std::unordered_map, std::vector;
template<typename NodeType>
class A_star {
    private:
    std::function<double(NodeType*, NodeType*)> heuristic;
    std::function<double(NodeType*, NodeType*)> travel_cost;

    public:
    unordered_map<NodeType*, vector<NodeType*>> neighbors;

        A_star(unordered_map<NodeType*, vector<NodeType*>> neighbors,
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
                    double tentative_gScore = gScore[curr] + travel_cost(curr, *it);
                    if (gScore.find(*it) == gScore.end() || tentative_gScore < gScore[*it]) {
                        came_from[*it] = curr;
                        gScore[*it] = tentative_gScore;
                        fScore[*it] = tentative_gScore + heuristic(*it, goal);
                        open_set.put(*it, fScore[*it]);
                    }
                }
            }
            return path;
        }
};

