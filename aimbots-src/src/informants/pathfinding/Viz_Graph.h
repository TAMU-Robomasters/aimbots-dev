#pragma once

#include <vector>

#include "A_star.h"
#include "Point.h"
using std::vector;

class VizGraph {
public:
    AStar<Point> pathfinder;
    vector<vector<Point>>& polygons;

public:
    vector<Point> search(double x1, double y1, double x2, double y2);
    VizGraph(unordered_map<Point*, vector<Point*>> neighbors, vector<vector<Point>>& polygons);
};

VizGraph constructVizGraph(vector<vector<Point>>& polygons);
bool has_LOS(Point a, Point b, const vector<vector<Point>>& polygons);
