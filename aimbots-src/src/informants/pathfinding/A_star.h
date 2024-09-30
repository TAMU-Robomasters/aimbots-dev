#pragma once
#include <functional>
#include <unordered_map>
#include <vector>

#include "prio_queue.h"
using std::unordered_map, std::vector;
template <typename NodeType>
class AStar {
public:
    AStar(
        unordered_map<NodeType*, vector<NodeType*>> neighbors,
        std::function<double(NodeType*, NodeType*)> heuristic,
        std::function<double(NodeType*, NodeType*)> travel_cost)
        : neighbors(neighbors),
          heuristic(heuristic),
          travel_cost(travel_cost) {}

    AStar(AStar<NodeType>& other) : neighbors(other.neighbors), heuristic(other.heuristic), travel_cost(other.travel_cost) {}

    AStar& operator=(AStar<NodeType>& other) {
        if (this != &other) {
            neighbors = other.neighbors;
            heuristic = other.heuristic;
            travel_cost = other.travel_cost;
        }
        return *this;
    }

    unordered_map<NodeType*, vector<NodeType*>> neighbors;

    vector<NodeType*> search(NodeType* start, NodeType* goal) {
        vector<NodeType*> path;
        PrioQueue<NodeType*, double> open_set;

        unordered_map<NodeType*, NodeType*> came_from;
        unordered_map<NodeType*, double> g_score;  // cost of cheapest path from start to node
        g_score[start] = 0;

        unordered_map<NodeType*, double> f_score;  // estimated cost of the path through node from start to goal
        f_score[start] = heuristic(start, goal);
        open_set.put(start, f_score[start]);

        while (!open_set.empty()) {
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
                double tentative_gScore = g_score[curr] + travel_cost(curr, *it);
                if (g_score.find(*it) == g_score.end() || tentative_gScore < g_score[*it]) {
                    came_from[*it] = curr;
                    g_score[*it] = tentative_gScore;
                    f_score[*it] = tentative_gScore + heuristic(*it, goal);
                    open_set.put(*it, f_score[*it]);
                }
            }
        }
        return path;
    }

private:
    std::function<double(NodeType*, NodeType*)> heuristic;
    std::function<double(NodeType*, NodeType*)> travel_cost;
};
