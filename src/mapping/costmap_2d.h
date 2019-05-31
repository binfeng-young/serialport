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
 *         David V. Lu!!
 *********************************************************************/


#ifndef _COSTMAP_2D_H_
#define _COSTMAP_2D_H_

#include <cmath>
#include <vector>
#include <map>
#include <mutex>
#include <memory>

#include "cost_values.h"
#include "typedef.h"
#include "cell_index.h"
#include "costmap_tools.h"

namespace bv {
namespace mapping {

/**
 * @class Costmap2D
 * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
 */
class Costmap2D {
public:
    Costmap2D();

    Costmap2D(const CellLimits &limits,
              double resolution,
              uint16_t default_value = MapValue::NO_INFORMATION,
              bool fixed = false);

    /**
   * @brief  Constructor for a costmap
   * @param  cells_size_x The x size of the map in cells
   * @param  cells_size_y The y size of the map in cells
   * @param  resolution The resolution of the map in meters/cell
   * @param  origin_x The x origin of the map
   * @param  origin_y The y origin of the map
   * @param  default_value Default Value
   */
    Costmap2D(const CellLimits &limits,
              double resolution,
              const Point &min_point,
              uint16_t default_value,
              bool fixed);
    /**
     * @brief  Destructor
     */
    virtual ~Costmap2D();

    virtual void setCost(const CellIndex &index, const uint16_t &cost)
    {
        if (!contains(index)){
            return;
        }
        cells_[toId(index)] = cost;
        if (known_cell_box_.isEmpty()) {
            known_cell_box_.min = index;
            known_cell_box_.max = index;
            return;
        }
        known_cell_box_.min.x = std::min(known_cell_box_.min.x, index.x);
        known_cell_box_.min.y = std::min(known_cell_box_.min.y, index.y);
        known_cell_box_.max.x = std::max(known_cell_box_.max.x, index.x);
        known_cell_box_.max.y = std::max(known_cell_box_.max.y, index.y);
    }

    virtual void updateCell(const CellIndex &index, const MapValue& value) {
        if (!contains(index)) {
            return;
        }
        uint16_t cost = CostValues::mapValue2Cost(value, cells_[toId(index)]);
        setCost(index, cost);
    }

    virtual MapValue getCellType(const CellIndex &index) const
    {
        if (!contains(index)) {
            return NO_INFORMATION;
        }
        return CostValues::cost2MapValue(cells_[toId(index)]);
    }

    //uint16_t getCost(const float& x, const float& y) const;

    virtual uint16_t getCost(const CellIndex &index) const
    {
        if (!contains(index)) {
            return default_value_;
        }
        return cells_[toId(index)];
    }

    CellIndex getCellIndex(const Point &point) const
    {
//        return {(int)std::lround((point.x - min_point_.x) / resolution_),
//                (int)std::lround((point.y - min_point_.y) / resolution_)};
        return {(int)std::floor((point.x - min_point_.x) / resolution_),
            (int)std::floor((point.y - min_point_.y) / resolution_)};
    }

    CellIndex getCellIndexEnforceBounds(const Point &point) const
    {
        auto cell_index = getCellIndex(point);
        auto bound = [=](int val, int min, int max) {
            if (val < min) {
                return min;
            }
            if (val > max) {
                return max;
            }
            return val;
        };
        cell_index.x = bound(cell_index.x, 0, limits_.size_x);
        cell_index.y = bound(cell_index.y, 0, limits_.size_y);
        return cell_index;
    }

    Point getWorld(const CellIndex &cell_index) const
    {
//        return {min_point_.x + (cell_index.x) * resolution_,
//                min_point_.y + (cell_index.y) * resolution_};
        return {min_point_.x + (cell_index.x + 0.5f) * resolution_,
            min_point_.y + (cell_index.y + 0.5f) * resolution_};
    }

    /**
     * @brief  Given an index... compute the associated map coordinates
     * @param  index The index
     * @return  cell_index Will be set to the xy coordinate
     */
    CellIndex indexToCellIndex(unsigned int index) const
    {
        CellIndex cell_index;
        cell_index.y = index / limits_.size_x;
        cell_index.x = index - (cell_index.y * limits_.size_x);
        return cell_index;
    }

    bool contains(const CellIndex &cell_index) const
    {
        return limits_.isContains(cell_index);
    }

    bool isKnown(const CellIndex &cell_index) const
    {
        return contains(cell_index) && getCost(cell_index) != default_value_;
    }

    std::string print() const;

    void cropped();

    void inflationForObstacle(float radius);

    /**
     * @brief  Move the origin of the costmap to a new location.... keeping data when it can
     * @param  offset_p The origin offset
     */
    void updateOriginByOffset(const Point& offset_p);

    std::unique_ptr<Costmap2D> transform(const Pose2D& origin, const Pose2D& goal);

    void grow(const Point& point);

    CellLimits getLimits() const {
        return limits_;
    }

    double getResolution() const
    {
        return resolution_;
    }

    /**
     * @brief  Sets the cost of a convex polygon to a desired value
     * @param polygon The polygon to perform the operation on,  [array]
     * @param cost_value The value to set costs to
     * @return True if the polygon was filled... false if it could not be filled
     */
    bool setConvexPolygonCost(const std::vector<Point> &polygon, MapValue cost_value);

    /**
     * @brief  Get the map cells that make up the outline of a polygon
     * @param polygon The polygon in map coordinates to rasterize, [array]
     * @param polygon_cells Will be set to the cells contained in the outline of the polygon
     * @param size polygon and polygon cell size
     */
    void polygonOutlineCells(const std::vector<CellIndex> &polygon, std::vector<CellIndex> &polygon_cells);

    /**
     * @brief  Get the map cells that fill a convex polygon
     * @param polygon The polygon in map coordinates to rasterize , [array]
     * @param polygon_cells Will be set to the cells that fill the polygon , [array]
     * @param size polygon and polygon cell size
     */
    void convexFillCells(const std::vector<CellIndex> &polygon, std::vector<CellIndex> &polygon_cells);

    void resetMap(const CellIndex& min, const CellIndex& max);

    /**
     * @brief  Given distance in the world... convert it to cells
     * @param  world_dist The world distance
     * @return The equivalent cell distance
     */
    int cellDistance(double world_dist);


    typedef std::recursive_mutex mutex_t;

    mutex_t *getMutex()
    {
        return access_;
    }

    /**
     * @brief  Deletes the costmap, static_map, and markers data structures
     */
    void deleteMaps();


    /**
     * @brief  Initializes the costmap, static_map, and markers data structures
     * @param size_x The x size to use for map initialization
     * @param size_y The y size to use for map initialization
     */
    void initMaps(unsigned int size_x, unsigned int size_y);

    /**
       * @brief  Raytrace a line and apply some action at each step
       * @param  at The action to take... a functor
       * @param  x0 The starting x coordinate
       * @param  y0 The starting y coordinate
       * @param  x1 The ending x coordinate
       * @param  y1 The ending y coordinate
       * @param  max_length The maximum desired length of the segment... allows you to not go all the way to the endpoint
       */
    template<class ActionType>
    void raytraceLine(ActionType at, unsigned int x0, unsigned int y0, unsigned int x1,
                      unsigned int y1,
                      unsigned int max_length = std::numeric_limits<unsigned int>::max())
    {
        int dx = x1 - x0;
        int dy = y1 - y0;

        unsigned int abs_dx = abs(dx);
        unsigned int abs_dy = abs(dy);

        int offset_dx = sign(dx);
        int offset_dy = sign(dy) * limits_.size_x;

        unsigned int offset = y0 * limits_.size_x + x0;

        // we need to chose how much to scale our dominant dimension, based on the maximum length of the line
        double dist = hypotf(dx, dy); // hypot
        double scale = (dist < 0.0) ? 1.0 : std::min(1.0, max_length / dist);

        // if x is dominant
        if (abs_dx >= abs_dy) {
            int error_y = abs_dx / 2;
            bresenham2D(at, abs_dx, abs_dy, error_y, offset_dx, offset_dy, offset,
                        (unsigned int) (scale * abs_dx));
            return;
        }

        // otherwise y is dominant
        int error_x = abs_dy / 2;
        bresenham2D(at, abs_dy, abs_dx, error_x, offset_dy, offset_dx, offset, (unsigned int) (scale * abs_dy));
    }

    void clearMaps(const Point& point, double reset_distance);

    bool enclosure(double wx, double wy, int thresholdSize);

    void setSizeFixed(bool fixed)
    {
        size_fixed_ = fixed;
    }

    bool isSizefixed()
    {
        return size_fixed_;
    }

protected:

    int toId(const CellIndex &index) const
    {
        return index.y * limits_.size_x + index.x;
    }

private:

    /**
     * @brief  Copy constructor for a costmap, creates a copy efficiently
     * @param map The costmap to copy
     */
    Costmap2D(const Costmap2D &map);

    /**
     * @brief  Overloaded assignment operator
     * @param  map The costmap to copy
     * @return A reference to the map after the copy has finished
     */
    Costmap2D &operator=(const Costmap2D &map);

    /**
     * @brief  Turn this costm ap into a copy of a window of a costmap passed in
     * @param  map The costmap to copy
     * @param win_origin_x The x origin (lower left corner) for the window to copy, in meters
     * @param win_origin_y The y origin (lower left corner) for the window to copy, in meters
     * @param win_size_x The x size of the window, in meters
     * @param win_size_y The y size of the window, in meters
     */
//    bool copyCostmapWindow(const Costmap2D& map, double win_origin_x, double win_origin_y, double win_size_x,
//                           double win_size_y);

    /**
     * @brief  A 2D implementation of Bresenham's raytracing algorithm... applies an action at each step
     */
    template<class ActionType>
    void bresenham2D(ActionType at, unsigned int abs_da, unsigned int abs_db, int error_b, int offset_a,
                     int offset_b, unsigned int offset, unsigned int max_length)
    {
        unsigned int end = std::min(max_length, abs_da);
        for (unsigned int i = 0; i < end; ++i) {
            at(offset);
            offset += offset_a;
            error_b += abs_db;
            if ((unsigned int) error_b >= abs_da) {
                offset += offset_b;
                error_b -= abs_da;
            }
        }
        at(offset);
    }

    inline int sign(int x)
    {
        return x > 0 ? 1.0 : -1.0;
    }

    mutex_t *access_;
protected:
    CellLimits limits_;
    double resolution_;
    Point min_point_;
    std::vector<uint16_t> cells_;
    unsigned char default_value_;
    bool size_fixed_;
    struct BoundBox {
        CellIndex max;
        CellIndex min;
        BoundBox() {
            max.x = 0;
            max.y = 0;
            min.x = max.x + 1;
            min.y = max.y + 1;
        }
        bool isEmpty() const {
            return min.x > max.x || min.y > max.y;
        }
    } known_cell_box_;

    //class PolygonOutlineCells
    class PolygonOutlineCells {
    public:
        PolygonOutlineCells(const Costmap2D &costmap, std::vector<CellIndex> &cells) :
            costmap_(costmap), cells_(cells)
        {
        }

        // just push the relevant cells back onto the list
        inline void operator()(unsigned int offset)
        {
            cells_.emplace_back(costmap_.indexToCellIndex(offset));
        }

    private:
        const Costmap2D &costmap_;
        std::vector<CellIndex> &cells_;
    };
};

}
}
#endif  // _COSTMAP_2D_H     