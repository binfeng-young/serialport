/**
 * Created by bfyoung on 2019/2/14.
 */

#include <limits>
#include <stdlib.h>
#include <cmath>

#include "footprint.h"
#include "typedef.h"
#include "costmap_tools.h"

namespace bv {
namespace mapping {

void calculateMinAndMaxDistances(const std::vector<Point> &footprint, double &min_dist, double &max_dist)
{
    min_dist = std::numeric_limits<double>::max();
    max_dist = 0.0f;

    if (footprint.size() <= 2) {
        return;
    }

    for (unsigned short int i = 0; i < footprint.size() - 1; ++i) {
        // check the distance from the robot center point to the first vertex
        double vertex_dist = distance(0.0, 0.0, footprint[i].x, footprint[i].y);
        double edge_dist = distanceToLine(0.0, 0.0, footprint[i].x, footprint[i].y,
                                         footprint[i + 1].x, footprint[i + 1].y);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    // we also need to do the last vertex and the first vertex
    double vertex_dist = distance(0.0, 0.0, footprint.back().x, footprint.back().y);
    double edge_dist = distanceToLine(0.0, 0.0, footprint.back().x, footprint.back().y,
                                     footprint.front().x, footprint.front().y);
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
}

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1)
{
    double A = pX - x0;
    double B = pY - y0;
    double C = x1 - x0;
    double D = y1 - y0;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = dot / len_sq;

    double xx, yy;

    if (param < 0) {
        xx = x0;
        yy = y0;
    } else if (param > 1) {
        xx = x1;
        yy = y1;
    } else {
        xx = x0 + param * C;
        yy = y0 + param * D;
    }

    return distance(pX, pY, xx, yy);
}

std::vector<Point> makeFootprintFromRadius(const double& robot_radius)
{
    std::vector<Point> points;
    points.reserve(16);
    // Loop over 16 angles around a circle making a point each time
    int N = 16;
    for (int i = 0; i < N; ++i) {
        double angle = i * 2 * M_PI / N;
        points.push_back({cos(angle) * robot_radius, sin(angle) * robot_radius});
    }
    return points;
}

std::vector<Point> makeObstacleFootprintFromRadius(const double& robot_radius)
{
    std::vector<Point> points;
    points.reserve(16);
    double b = robot_radius / 2;
    double a2 = std::pow(robot_radius / 3, 2);
    double b2 = std::pow(b, 2);
    int N = 4;
    for (int i = 0; i <= N; ++i) {
        double angle = i * 2 * M_PI / 16;
        if (i == 4) {
            points.push_back({0, b});
            points.push_back({0, -b});
        } else {
            Point p = {sign0(cos(angle)) * std::sqrt(a2 * b2 / (b2 + a2 * std::pow(std::tan(angle), 2))),
                    sign0(sin(angle)) * std::sqrt(a2 * b2 / (a2 + b2 * std::pow(1 / std::tan(angle), 2)))};
            points.push_back(p);
            points.push_back({p.x, -p.y});
        }
    }
    return points;
}


void transformFootprint(const Point& offset, double theta, const std::vector<Point> &footprint_spec,
                        std::vector<Point> &oriented_footprint)
{
    // build the oriented footprint at a given location
    oriented_footprint.clear();
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    for (const auto& it : footprint_spec) {
        oriented_footprint.push_back(transformPoint(it, offset, cos_th, sin_th));
    }
}

void scaleFootprint(std::vector<Point> &footprint, double scale)
{
    // pad footprint in place
    for (auto &pt : footprint) {
        pt.x *= scale;
        pt.y *= scale;
    }
}

void padFootprint(std::vector<Point> &footprint, double width_padding, double length_padding)
{
    // pad footprint in place
    for (auto &pt : footprint) {
        pt.x += sign0(pt.x) * width_padding;
        pt.y += sign0(pt.y) * length_padding;
    }
}

}
}
