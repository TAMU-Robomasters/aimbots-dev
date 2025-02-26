#pragma once

#include "A_star.h"
#include "Point.h"
#include <vector>
using std::vector;

class VizGraph {
    public:
    A_star<Point> pathfinder;
    vector<vector<Point>>& polygons;
    vector<pair<vector<Point>, double>> cost_zones;

    public:
    vector<Point> search(double x1, double y1, double x2, double y2);
    VizGraph(unordered_map<Point*, vector<pair<Point*, double>>> neighbors, vector<vector<Point>>& polygons, vector<pair<vector<Point>, double>>& cost_zones);
};

VizGraph constructVizGraph(vector<vector<Point>>& polygons, vector<pair<vector<Point>, double>>& cost_zones);
bool hasLOS(Point a, Point b, const vector<vector<Point>>& polygons);
double shoelace(const vector<Point>& polygon);
void normalize(Point& pt);
double cost(Point a, Point b, vector<pair<vector<Point>, double>>& cost_zones);

