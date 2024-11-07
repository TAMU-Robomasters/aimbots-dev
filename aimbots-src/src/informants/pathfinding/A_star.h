#pragma once
#include <functional>
#include <unordered_map>
#include <vector>

#include "prio_queue.h"
using std::unordered_map, std::vector, std::pair;
template <typename NodeType>
class AStar {
public:
    AStar(
        unordered_map<NodeType*, vector<std::pair<NodeType*, double>>> neighbors,
        std::function<double(NodeType*, NodeType*)> heuristic)
        : neighbors(neighbors),
          heuristic(heuristic) {}

    AStar(AStar<NodeType>& other) : neighbors(other.neighbors), heuristic(other.heuristic) {}

    AStar& operator=(AStar<NodeType>& other) {
        if (this != &other) {
            neighbors = other.neighbors;
            heuristic = other.heuristic;
        }
        return *this;
    }

    unordered_map<NodeType*, vector<std::pair<NodeType*, double>>> neighbors;

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
                double tentative_gScore = g_score[curr] + it->second;
                if (g_score.find(it->first) == g_score.end() || tentative_gScore < g_score[it->first]) {
                    came_from[it->first] = curr;
                    g_score[it->first] = tentative_gScore;
                    f_score[it->first] = tentative_gScore + heuristic(it->first, goal);
                    open_set.put(it->first, f_score[it->first]);
                }
            }
        }
        return path;
    }

private:
    std::function<double(NodeType*, NodeType*)> heuristic;    
};
