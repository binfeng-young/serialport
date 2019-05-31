/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/
#ifndef _COST_VALUES_H_
#define _COST_VALUES_H_

#include <cstdint>

/** Provides a mapping for often used cost values */
namespace bv {
namespace mapping {


enum MapValue : unsigned char {
    NO_INFORMATION = 255,//255;
    LETHAL_OBSTACLE = 254,//254;
    INSCRIBED_INFLATED_OBSTACLE = 253,//253;
    FREE_SPACE = 0,
};
enum BoundType
{
    NORMAL_BOUND,
    SMALL_BOUND,
};

/**
 * @class CostValues
 * @brief MapValue and cost convert to each other
 * cost = (num_of(obstacle) << 8) | num_of(free)
 * MapValue = ratio of obstacle and free...
 */
class CostValues {
public:
    static MapValue cost2MapValue(const uint16_t& cost) {
        int obstacle = ((cost >> 8) & 0xff);
        int free = (cost & 0xff);
        int sum = obstacle + free;
        double obstacle_rate = (obstacle * 1.0) / sum;
        if (obstacle_rate > 0.5) {
            return LETHAL_OBSTACLE;
        }
        if (obstacle_rate > 0.1) {
            return INSCRIBED_INFLATED_OBSTACLE;
        }
        if (free) {
            return FREE_SPACE;
        }
        return NO_INFORMATION;
    }

    static uint16_t mapValue2Cost(const MapValue& value, const uint16_t& last_cost) {
        uint16_t cost = last_cost;
        switch (value) {
            case NO_INFORMATION: cost = 0; break;
            case LETHAL_OBSTACLE: cost += (1 << 8); break;
            case INSCRIBED_INFLATED_OBSTACLE: cost += (1 << 8); break;
            case FREE_SPACE: cost += 1; break;
        }
        if (((cost >> 8) & 0xff) >= 200 || (cost & 0xff) >= 200) {
            cost /= 2;
        }
        return cost;
    }
};

}
}
#endif  // _COST_VALUES_H_
