/**
 * Created by bfyoung on 2019/2/14.
 */

#ifndef _FOOTPRINT_H_
#define _FOOTPRINT_H_

#include <math.h>
#include <vector>
#include "typedef.h"

namespace bv {
namespace mapping {

inline double distance(double x0, double y0, double x1, double y1)
{
    return hypotf(x1 - x0, y1 - y0);
}

inline double sign0(double x)
{
    return x < 0.0f ? -1.0f : (x > 0.0f ? 1.0f : 0.0f);
}

void calculateMinAndMaxDistances(const std::vector<Point> &footprint, double &min_dist, double &max_dist);

double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);

void scaleFootprint(std::vector<Point> &footprint, double scale);

void padFootprint(std::vector<Point> &footprint, double width_padding, double length_padding);


std::vector<Point> makeFootprintFromRadius(const double& robot_radius);

std::vector<Point> makeObstacleFootprintFromRadius(const double& robot_radius);

void transformFootprint(const Point& offset, double theta, const std::vector<Point> &footprint_spec,
                        std::vector<Point> &oriented_footprint);

}
}

#endif /* _FOOTPRINT_H_ */
