#include "viz_graph.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

using std::sqrt, std::vector, std::unordered_map, std::cout, std::endl, std::remove, std::pair;




VizGraph::VizGraph(unordered_map<Point*, vector<Point*>> neighbors, vector<vector<Point>>& polygons)
    : pathfinder(
          neighbors,
          [](Point* a, Point* b) -> double { return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y)); },
          [](Point* a, Point* b) -> double { return sqrt((a->x - b->x) * (a->x - b->x) + (a->y - b->y) * (a->y - b->y)); }),
      polygons(polygons) {}

bool has_LOS(Point a, Point b, const vector<vector<Point>>& polygons) {
    // arbitrarily define a as our base point
    double x1 = a.x;
    double y1 = a.y;

    // line is defined as basepoint + dr*t where t is an arbitrary parameter.
    // if t > 1 or t < 0 then point lays on line but not line segment.
    // if t = 0, or t = 1 then point lays on vertex of a or b respectively

    // deltas
    double x2 = b.x;
    double y2 = b.y;

    for (auto it = polygons.begin(); it != polygons.end(); it++) {
        // for every point find the line connecting it to the "next" point
        for (auto jt = it->begin(); jt != it->end(); jt++) {
            auto next = jt + 1;

            if (next == it->end()) {
                next = it->begin();
            }

            double x3 = jt->x;
            double y3 = jt->y;

            double x4 = next->x;
            double y4 = next->y;

            double detirminant = ((x2 - x1) * (y3 - y4) - (x3 - x4) * (y2 - y1));
            if (detirminant == 0) continue;  // lines never intersect

            double s = (static_cast<double>(1.0) / detirminant) * ((x3 - x1) * (y3 - y4) - (y3 - y1) * (x3 - x4));
            double t = (static_cast<double>(1.0) / detirminant) * ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));

            // cout << "The line from " << x1 << " " << y1 << " to " << b.x << " " << b.y << " does " << ((s > 0 && s < 1 &&
            // t >= 0 && t <= 1) ? "" : "not "); cout << "intersect with the line from " << x3 << " " << y3 << " to " <<
            // next->x << " " << next->y << endl; cout << "\ts " << s << " t " << t << endl;

            // if intersection falls on both line segments and is not at either of line 1s vertices, then no LOS
            if (s > 0 && s < 1 && t >= 0 && t <= 1) return false;
        }
    }
    return true;
}

VizGraph constructVizGraph(vector<vector<Point>>& polygons, vector<pair<vector<Point>, double>>& cost_zones) {
    unordered_map<Point*, vector<Point*>> neighbors;

    for (auto it = polygons.begin(); it != polygons.end(); it++) {
        // iterate through all polygons, it is an iterator to a vector of vectors of points, *it is a vector of points

        // make consecutive points neighbors NOTE DOES NOT CHECK NON-CONSECUTIVE POINTS, CONCAVE POLYGONS WILL NOT WORK
        // CORRECTLY
        
        //find the overall ccw/cw direction of the polygon using the shoelace formula, store this as a z unit vector
        double z_dir = shoelace(*it);
        z_dir = z_dir < 0 ? -1 : 1;
        
        for (auto jt = it->begin(); jt != it->end(); jt++) {
            Point last = jt == it->begin() ? it->back() : *(jt - 1);
            Point next = jt == it->end() - 1 ? it->front() : *(jt + 1);

            // vector from last to current
            Point V1 = Point(jt->x - last.x, jt->y - last.y);

            // vector from current to next
            Point V2 = Point(next.x - jt->x, next.y - jt->y);

            normalize(V1);
            normalize(V2);
            //vectors normal to V1 and V2, pointing outside of the polygon
            Point normal1 = Point(V1.y*z_dir, -V1.x*z_dir);
            Point normal2 = Point(V2.y*z_dir, -V2.x*z_dir);

            //vector will be in the middle of the the two vectors starting at current and pointing to next and last
            //vector will be pointing outside of the polygon
            Point mid = Point(normal1.x + normal2.x, normal1.y + normal2.y);

            normalize(mid);

            //any vector with a dot product with the middle smaller then the rejection dot will pass through the area of polygon, reject it
            double rejection_dot = mid.x * V2.x + mid.y * V2.y;

            for (auto kt = jt + 1; kt != it->end(); kt++) {
                if (has_LOS(*jt, *kt, polygons)) {
                    Point los_vector = Point(kt->x - jt->x, kt->y - jt->y);
                    normalize(los_vector);
                    if (los_vector.x * mid.x + los_vector.y * mid.y >= rejection_dot) {
                        neighbors[&*jt].push_back(&*kt);
                        neighbors[&*kt].push_back(&*jt);
                }
            }

            // add points on other polygons, LOS is two way so dont have to double check polygons so start at it not begin
            for (auto xt = it + 1; xt != polygons.end(); xt++) {
                for (auto yt = xt->begin(); yt != xt->end(); yt++) {
                    if (has_LOS(*jt, *yt, polygons)) {
                        neighbors[&*jt].push_back(&*yt);
                        neighbors[&*yt].push_back(&*jt);
                    }
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

    if (has_LOS(start, goal, polygons)) {
        return {start, goal};
    }

    for (auto it = polygons.begin(); it != polygons.end(); it++) {
        for (auto jt = it->begin(); jt != it->end(); jt++) {
            if (has_LOS(start, *jt, polygons)) {
                pathfinder.neighbors[&start].push_back(&*jt);
            }

            if (has_LOS(goal, *jt, polygons)) {
                pathfinder.neighbors[&*jt].push_back(&goal);
                pathfinder.neighbors[&goal].push_back(&*jt);  // used to remove goal after search
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

double shoelace(const vector<Point>& polygon) {
    double sum = 0;
    for (size_t i = 0; i < polygon.size() - 1; i++) {
        sum += polygon[i].x * polygon[i + 1].y - polygon[i + 1].x * polygon[i].y;
    }
    sum += polygon.back().x * polygon.front().y - polygon.front().x * polygon.back().y;
    return sum / 2;
}

void normalize(Point& pt) {
    double mag = sqrt(pt.x * pt.x + pt.y * pt.y);
    pt.x /= mag;
    pt.y /= mag;
}

double cost(Point a, Point b, vector<pair<vector<Point>, double>>& cost_zones) {
    for (auto it = cost_zones.begin(); it != cost_zones.end(); it++) {

        // find the max and min x and y values of the zone to check if the line is fully or partially in the zone
        double max_x = 0;
        double min_x = 0;
        double max_y = 0;
        double min_y = 0;
        for (auto jt = it->first.begin(); jt != it->first.end(); jt++) {
            max_x = jt->x > max_x ? jt->x : max_x;
            min_x = jt->x < min_x ? jt->x : min_x;
            max_y = jt->y > max_y ? jt->y : max_y;
            min_y = jt->y < min_y ? jt->y : min_y;
        }

        vector<Point> intersections;
        for (auto jt = it->first.begin(); jt != it->first.end(); jt++) {

            double x3 = jt->x;
            double y3 = jt->y;

            double x4 = next->x;
            double y4 = next->y;

            double detirminant = ((x2 - x1) * (y3 - y4) - (x3 - x4) * (y2 - y1));
            if (detirminant == 0) continue;  // lines never intersect

            double s = (static_cast<double>(1.0) / detirminant) * ((x3 - x1) * (y3 - y4) - (y3 - y1) * (x3 - x4));
            double t = (static_cast<double>(1.0) / detirminant) * ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));

            // cout << "The line from " << x1 << " " << y1 << " to " << b.x << " " << b.y << " does " << ((s > 0 && s < 1 &&
            // t >= 0 && t <= 1) ? "" : "not "); cout << "intersect with the line from " << x3 << " " << y3 << " to " <<
            // next->x << " " << next->y << endl; cout << "\ts " << s << " t " << t << endl;

            // if intersection falls on both line segments and is not at either of line 1s vertices, then no LOS
            if (s > 0 && s < 1 && t > 0 && t < 1){
                intersections.push_back(Point(x1 + s * (x2 - x1), y1 + s * (y2 - y1)));
            }
        }

        switch(intersections.size()){
            case 0:
                break;
            case 1:
                break;
            case 2:
            default: // more then 2 intersections should not be possible, if this happens just take the first 2 i guess?
                break;
        }
    }
    return 1;
}