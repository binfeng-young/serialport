/**
 * Created by bfyoung on 2019/2/13.
 */

#ifndef BVROBOT2_RANGEDATA_H
#define BVROBOT2_RANGEDATA_H

#include <vector>
#include <cstdint>
#include <cost_values.h>

#include "typedef.h"

namespace bv {

struct PointData {
    mapping::Pose2D pose;
    enum Type {
        NONE = 0,
        BUMP_LEFT,
        BUMP_FRONT,
        BUMP_RIGHT,
        WALL_LEFT,
        WALL_RIGHT,
    } type;
};

struct RangeData {
    mapping::Pose2D origin_pose;

    std::vector<PointData> points;
};

}

#endif //BVROBOT2_RANGEDATA_H
