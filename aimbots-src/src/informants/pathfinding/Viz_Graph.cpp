#include "Viz_Graph.h"
#include <cmath>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <algorithm>

using std::sqrt, std::vector, std::unordered_map, std::cout, std::endl, std::remove;
VizGraph::VizGraph(unordered_map<Point*, vector<Point*>> neighbors, vector<vector<Point>>& polygons)
 : pathfinder 
 (
    neighbors,
    [](Point* a, Point* b) -> double {
        return sqrt((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
    },
    [](Point* a, Point* b) -> double {
        return sqrt((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
    }
 ), polygons(polygons)
 {}

bool hasLOS(Point a, Point b, const vector<vector<Point>>& polygons) {

    //arbitrarily define a as our base point
    double x1 = a.x;
    double y1 = a.y;

    //line is defined as basepoint + dr*t where t is an arbitrary parameter.
    //if t > 1 or t < 0 then point lays on line but not line segment.
    //if t = 0, or t = 1 then point lays on vertex of a or b respectively

    //deltas
    double x2 = b.x;
    double y2 = b.y;

    for (auto it = polygons.begin(); it != polygons.end(); it++) {

        //for every point find the line connecting it to the "next" point
        for (auto jt = it->begin(); jt != it->end(); jt++) {
            auto next = jt+1;

            if (next == it->end()) {
                next = it->begin();
            }

            double x3 = jt->x;
            double y3 = jt->y;

            double x4 = next->x;
            double y4 = next->y;

            float detirminant = ((x2-x1)*(y3-y4) - (x3-x4)*(y2-y1));
            if (detirminant == 0) continue; //lines never intersect

            float s = (1.0f/detirminant)*((x3-x1)*(y3-y4) - (y3-y1)*(x3-x4));
            float t = (1.0f/detirminant)*((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));

            // cout << "The line from " << x1 << " " << y1 << " to " << b.x << " " << b.y << " does " << ((s > 0 && s < 1 && t >= 0 && t <= 1) ? "" : "not ");
            // cout << "intersect with the line from " << x3 << " " << y3 << " to " << next->x << " " << next->y << endl;
            // cout << "\ts " << s << " t " << t << endl;

            //if intersection falls on both line segments and is not at either of line 1s vertices, then no LOS
            if (s > 0 && s < 1 && t >= 0 && t <= 1) return false;
        }
    }
    return true;
}


 VizGraph constructVizGraph(vector<vector<Point>>& polygons) {
    unordered_map<Point*, vector<Point*>> neighbors;

    for (auto it = polygons.begin(); it != polygons.end(); it++) {
        //iterate through all polygons, it is an iterator to a vector of vectors of points, *it is a vector of points

        //make consecutive points neighbors NOTE DOES NOT CHECK NON-CONSECUTIVE POINTS, CONCAVE POLYGONS WILL NOT WORK CORRECTLY
        for (auto jt = it->begin(); jt != it->end(); jt++) {
            //iterate through individual polygon, jt is an iterator to a vector of points, *jt is a point
            auto next = jt+1; //next point
            if (next == it->end()) { //if last point
                neighbors[&*jt].push_back(&*(it->begin())); //address of first point
                neighbors[&*(it->begin())].push_back(&*jt);
            }

            //not the end
            else {
                neighbors[&*jt].push_back(&*next);
                neighbors[&*next].push_back(&*jt);
            }

            // add points on other polygons, LOS is two way so dont have to double check polygons so start at it not begin
            for (auto xt = it+1; xt != polygons.end(); xt++) {
                for (auto yt = xt->begin(); yt != xt->end(); yt++) {
                    if (hasLOS(*jt, *yt, polygons)) {
                        neighbors[&*jt].push_back(&*yt);
                        neighbors[&*yt].push_back(&*jt);
                    }
                }
            }
        } 
    }

    return VizGraph(neighbors, polygons);
 }

 vector<Point> VizGraph::search(double x1, double y1, double x2, double y2) {
    Point start(x1, y1);
    Point goal(x2, y2);
    vector<Point> path;

    if (hasLOS(start, goal, polygons)) {
        return {start, goal};
    }

    for (auto it = polygons.begin(); it != polygons.end(); it++) {
        for (auto jt = it->begin(); jt != it->end(); jt++) {
            if (hasLOS(start, *jt, polygons)) {
                pathfinder.neighbors[&start].push_back(&*jt);
            }

            if (hasLOS(goal, *jt, polygons)) {
                pathfinder.neighbors[&*jt].push_back(&goal);
                pathfinder.neighbors[&goal].push_back(&*jt); //used to remove goal after search
            }
        }
    }

    vector<Point*> ptrPath = pathfinder.search(&start, &goal);
    for (auto it = ptrPath.begin(); it != ptrPath.end(); it++) {
        path.push_back(**it);
    }

    for (auto it = pathfinder.neighbors[&goal].begin(); it != pathfinder.neighbors[&goal].end(); it++) {
        vector<Point*>& vec = pathfinder.neighbors[*it];
        vec.erase(remove(vec.begin(), vec.end(), &goal), vec.end());
    }

    pathfinder.neighbors.erase(pathfinder.neighbors.find(&start));
    pathfinder.neighbors.erase(pathfinder.neighbors.find(&goal));


    
    return path;
 }