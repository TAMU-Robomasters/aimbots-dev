#pragma once

#include "A_star.h"
#include "Point.h"
#include <vector>
using std::vector;

class VizGraph {
    public:
    A_star<Point> pathfinder;
    vector<vector<Point>>& polygons;

    public:
    vector<Point> search(double x1, double y1, double x2, double y2);
    VizGraph(unordered_map<Point*, vector<Point*>> neighbors, vector<vector<Point>>& polygons);
};

VizGraph constructVizGraph(vector<vector<Point>>& polygons);
bool hasLOS(Point a, Point b, const vector<vector<Point>>& polygons);

