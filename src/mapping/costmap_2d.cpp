
#include <queue>
#include <list>
#include <cstring>
#include <iostream>
#include <algorithm>

#include "costmap_2d.h"
#include "cost_values.h"
#include "costmap_tools.h"
#include "footprint.h"

using namespace std;

namespace bv {
namespace mapping {
Costmap2D::Costmap2D()
    : limits_()
    , resolution_(0)
    , min_point_()
    , cells_()
    , default_value_(0)
    , size_fixed_(false)
    , known_cell_box_()
{
    // Create mutex
    access_ = new mutex_t;
}

Costmap2D::Costmap2D(const CellLimits &limits,
          double resolution,
          uint16_t default_value,
          bool fixed)
    : limits_(limits)
    , resolution_(resolution)
    , cells_(limits.size_x * limits.size_y, default_value)
    , default_value_(default_value)
    , size_fixed_(fixed)
    , known_cell_box_()
{
    min_point_ = {0 - 0.5 * limits.size_x * resolution, 0 - 0.5 * limits.size_y * resolution};
}

Costmap2D::Costmap2D(const CellLimits &limits,
                     double resolution,
                     const Point &min_point,
                     uint16_t default_value,
                     bool fixed)
    : limits_(limits)
    , resolution_(resolution)
    , min_point_(min_point)
    , cells_(limits.size_x * limits.size_y, default_value)
    , default_value_(default_value)
    , size_fixed_(fixed)
    , known_cell_box_()
{
    // Create mutex
    access_ = new mutex_t;
}
Costmap2D::Costmap2D(const Costmap2D& map)
{
    access_ = new mutex_t;
    *this = map;
}

Costmap2D& Costmap2D::operator=(const Costmap2D& map)
{
    // check for self assignement
    if (this == &map)
        return *this;

    limits_ = map.limits_;
    resolution_ = map.resolution_;
    min_point_ = map.min_point_;
    default_value_ = map.default_value_;
    known_cell_box_ = map.known_cell_box_;
    size_fixed_ = map.size_fixed_;
    cells_ = map.cells_;

    return *this;
}


void Costmap2D::deleteMaps()
{
    // clean up data
    std::unique_lock<mutex_t> lock(*access_);
    std::vector<uint16_t>().swap(cells_);
}


Costmap2D::~Costmap2D()
{
    deleteMaps();
    delete access_;
}

void Costmap2D::clearMaps(const Point& point, double reset_distance)
{
    Point start_point = {
        point.x - reset_distance / 2, point.y - reset_distance / 2
    };
    Point end_point = {
        start_point.x + reset_distance, start_point.y + reset_distance
    };

    CellIndex min = getCellIndexEnforceBounds(start_point);
    CellIndex max = getCellIndexEnforceBounds(end_point);

    //lock
    std::unique_lock<mutex_t> lock(*access_);
    for (const auto& index : CellIndexRangeIterator(min, max)) {
        setCost(index, FREE_SPACE); //default_value_ //should be NO_INFORMATION?
    }
}

void Costmap2D::resetMap(const CellIndex& min, const CellIndex& max)
{
    std::unique_lock<mutex_t> lock(*access_);
    for (const auto& index : CellIndexRangeIterator(min, max)) {
        setCost(index, default_value_);
    }
}

std::string Costmap2D::print() const
{
    std::string out;
    for (int i = 0; i < limits_.size_x; i++) {
        for (int j = 0; j < limits_.size_y; j++) {
            out += std::to_string((int)getCellType(CellIndex(i, j)));
            if (j != limits_.size_y - 1) {
                out += " ";
            }
        }

        if (i != limits_.size_x - 1) {
            out += '\n';
        }
    }
    return out;
}


void Costmap2D::inflationForObstacle(float radius)
{
    auto inflation = cells_;
    auto foot = makeFootprintFromRadius(radius);

    for (auto cell : CellIndexRangeIterator(limits_)) {
        if (getCost(cell) == MapValue::LETHAL_OBSTACLE || cell.x == 0 || cell.y == 0 || cell.y == limits_.size_y - 1 || cell.x == limits_.size_x - 1) {
            std::vector<Point> polygon;
            transformFootprint(getWorld(cell), 0, foot, polygon);


            std::vector<CellIndex> map_polygon;

            for (const auto& i : polygon) {
                auto index =getCellIndex(i);
                if (!limits_.isContains(index)) {
                    continue;
                }
                map_polygon.push_back(index);
            }

            if (map_polygon.empty()) {
                continue;
            }
            std::vector<CellIndex> polygon_cells;

            convexFillCells(map_polygon, polygon_cells);
            for (auto index : polygon_cells) {
                if (!limits_.isContains(index)) {
                    continue;
                }
                inflation[toId(index)] = MapValue::LETHAL_OBSTACLE;
            }
        }
    }
    cells_ = inflation;
}

void Costmap2D::cropped()
{
    CellIndex offset = known_cell_box_.min;
    CellLimits box = CellLimits(known_cell_box_.max.x - known_cell_box_.min.x + 1
        , known_cell_box_.max.y - known_cell_box_.min.y + 1);
    auto cropped_map_min = CellIndex(0, 0);
    auto cropped_map_max = CellIndex(box.size_x - 1, box.size_y - 1);
    std::vector<unsigned short> new_cells(box.size_x * box.size_y, default_value_);
    std::unique_lock<mutex_t> lock(*access_);
    for (const auto& new_cell_index : CellIndexRangeIterator(cropped_map_min, cropped_map_max)) {
        auto cell_index = new_cell_index + offset;
        if (!isKnown(cell_index)) continue;
        new_cells[new_cell_index.x + new_cell_index.y * box.size_x] = cells_[toId(cell_index)];
    }

    limits_ = box;
    min_point_.x = min_point_.x + offset.x * resolution_;
    min_point_.y = min_point_.y + offset.y * resolution_;
    cells_ = new_cells;
    known_cell_box_.min = cropped_map_min;
    known_cell_box_.max = cropped_map_max;
}
void Costmap2D::grow(const Point& point)
{

    for (CellIndex index = getCellIndex(point); !contains(index); index = getCellIndex(point)) {
        CellIndex offset (0, 0);
        CellLimits new_limits = limits_;
        if (index.x < 0) {
            offset.x += limits_.size_x / 2;
            new_limits.size_x += limits_.size_x / 2;
        }

        if (index.y < 0) {
            offset.y += limits_.size_y / 2;
            new_limits.size_y += limits_.size_y / 2;
        }

        if (index.x >= limits_.size_x) {
            new_limits.size_x += limits_.size_x / 2;
        }

        if (index.y >= limits_.size_y) {
            new_limits.size_y += limits_.size_y / 2;
        }
        //const int offset = x_offset + stride * y_offset;
        const int new_size = new_limits.size_x * new_limits.size_y;

        std::vector<unsigned short> new_cells(new_size, default_value_);
        std::unique_lock<mutex_t> lock(*access_);
        for (const auto& cell_index : CellIndexRangeIterator(limits_)) {
            auto new_cell_index = cell_index + offset;
            new_cells[new_cell_index.x + new_cell_index.y * new_limits.size_x] = cells_[toId(cell_index)];
        }
        limits_ = new_limits;
        min_point_.x = min_point_.x - offset.x * resolution_;
        min_point_.y = min_point_.y - offset.y * resolution_;
        cells_ = new_cells;
        if (!known_cell_box_.isEmpty()) {
            known_cell_box_.min = known_cell_box_.min + offset;
            known_cell_box_.max = known_cell_box_.max + offset;
        }
    }
}


void Costmap2D::updateOriginByOffset(const Point& offset_p)
{
    CellIndex offset = CellIndex((int)std::lround(offset_p.x / resolution_),
                                          (int)std::lround(offset_p.y / resolution_));
    CellIndex new_min = known_cell_box_.min + offset;
    CellIndex new_max = known_cell_box_.max + offset;
    if (!contains(new_min) || !contains(new_max) || offset == CellIndex(0, 0) || known_cell_box_.isEmpty()) {
        return;
    }
    std::unique_lock<mutex_t> lock(*access_);
    if (offset.y < 0 || (offset.y == 0 && offset.x < 0)) {
        for (const auto& index : CellIndexRangeIterator(known_cell_box_.min, known_cell_box_.max)) {
            auto new_cell_index = index + offset;
            cells_[toId(new_cell_index)] = cells_[toId(index)];
            cells_[toId(index)] = default_value_;
        }
    } else {
        for (int i = known_cell_box_.max.y; i >= known_cell_box_.min.y; i--) {
            for (int j = known_cell_box_.max.x; j >= known_cell_box_.min.x; j--) {
                auto index = CellIndex(j, i);
                auto new_cell_index = index + offset;
                cells_[toId(new_cell_index)] = cells_[toId(index)];
                cells_[toId(index)] = default_value_;
            }
        }
    }
    known_cell_box_.min = new_min;
    known_cell_box_.max = new_max;
    min_point_.x = min_point_.x - offset_p.x;
    min_point_.y = min_point_.y - offset_p.y;
}

std::unique_ptr<Costmap2D> Costmap2D::transform(const Pose2D& origin, const Pose2D& goal)
{
    double delta_theta = goal.theta - origin.theta;
    double cos_th = cos(delta_theta);
    double sin_th = sin(delta_theta);
    Point new_min_point = {goal.point.x - 0.5 * limits_.size_x * resolution_, goal.point.y - 0.5 * limits_.size_y * resolution_};
    std::unique_ptr<Costmap2D>
        new_map = std::make_unique<Costmap2D>(limits_, resolution_, new_min_point, default_value_, fixed);
    for (const auto index : CellIndexRangeIterator(limits_)) {
        if (!isKnown(index)) {
            continue;
        }
        auto cost = getCost(index);

        Point point = getWorld(index);
        Point new_point =
            transformPoint({point.x - origin.point.x, point.y - origin.point.y}, goal.point, cos_th, sin_th);
        auto new_index = new_map->getCellIndex(new_point);
        if (!new_map->contains(new_index)) {
            new_map->grow(new_point);
            new_index = new_map->getCellIndex(new_point);
        }
//        auto n4 = nhood4(new_index, new_map->size_x_, new_map->size_y_);
//        for (const auto nb: n4) {
//            if (!new_map->contains(nb)) {
//                continue;
//            }
//            new_map->setCost(nb, static_cast<const uint16_t &>(new_map->getCost(nb) + cost / 4));
//        }
        new_map->setCost(new_index, new_map->getCost(new_index) + cost);
    }
    new_map->cropped();
    return new_map;
}

int Costmap2D::cellDistance(double world_dist)
{
    return std::abs((int)std::lround(world_dist / resolution_));
}

bool Costmap2D::setConvexPolygonCost(const std::vector<Point>& polygon, MapValue cost_value)
{
    // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
    std::vector<CellIndex> map_polygon;

    for (const auto& i : polygon) {
        if (!contains(getCellIndex(i))) {
            if (size_fixed_) {
                // ("Polygon lies outside map bounds, so we can't fill it");
                return false;
            }
            grow(i);
        }
    }

    for (const auto& i : polygon) {
        map_polygon.push_back(getCellIndex(i));
    }

    if (map_polygon.empty()) {
        return false;
    }
    std::vector<CellIndex> polygon_cells;

    convexFillCells(map_polygon, polygon_cells);

    // set the cost of those cells
    std::unique_lock<mutex_t> lock(*access_);
    for (auto &polygon_cell : polygon_cells) {
        //std::cout << polygon_cell.x << " " << polygon_cell.y << ", ";
        updateCell(polygon_cell, cost_value);
    }
    //

//  DEBUG_LOG(DEBUG_THIS_FILE,"raw map(%d) = %d \r\n",map_index_.size(), polygon_cells.size());
    return true;
}

void Costmap2D::polygonOutlineCells(const std::vector<CellIndex>& polygon, std::vector<CellIndex>& polygon_cells)
{
    PolygonOutlineCells cell_gatherer(*this, polygon_cells);
    for (unsigned int i = 0; i < polygon.size() - 1; ++i)
    {
        raytraceLine(cell_gatherer, polygon[i].x, polygon[i].y, polygon[i + 1].x, polygon[i + 1].y);
    }
    if (!polygon.empty())
    {
        unsigned int last_index = polygon.size() - 1;
        // we also need to close the polygon by going from the last point to the first
        raytraceLine(cell_gatherer, polygon[last_index].x, polygon[last_index].y, polygon[0].x, polygon[0].y);
    }
}

void Costmap2D::convexFillCells(const std::vector<CellIndex>& polygon, std::vector<CellIndex>& polygon_cells)
{
    // we need a minimum polygon of a triangle
    if (polygon.size() < 3)
        return;

    // first get the cells that make up the outline of the polygon
    polygonOutlineCells(polygon, polygon_cells);


    // quick bubble sort to sort points by x
    CellIndex swap;
    unsigned int i = 0;
    while (i < polygon_cells.size() - 1)
    {
        if (polygon_cells[i].x > polygon_cells[i + 1].x)
        {
            swap = polygon_cells[i];
            polygon_cells[i] = polygon_cells[i + 1];
            polygon_cells[i + 1] = swap;

            if (i > 0)
                --i;
        }
        else
            ++i;
    }
    auto iter = std::unique(polygon_cells.begin(), polygon_cells.end());
    polygon_cells.erase(iter, polygon_cells.end());

    i = 0;
    CellIndex min_pt;
    CellIndex max_pt;
    unsigned int min_x = polygon_cells[0].x;
    unsigned int max_x = polygon_cells[polygon_cells.size() - 1].x;

    // walk through each column and mark cells inside the polygon
    for (unsigned int x = min_x; x <= max_x; ++x)
    {
        if (i >= polygon_cells.size() - 1)
            break;

        if (polygon_cells[i].y < polygon_cells[i + 1].y)
        {
            min_pt = polygon_cells[i];
            max_pt = polygon_cells[i + 1];
        }
        else
        {
            min_pt = polygon_cells[i + 1];
            max_pt = polygon_cells[i];
        }

        i += 2;
        while (i < polygon_cells.size() && polygon_cells[i].x == x)
        {
            if (polygon_cells[i].y < min_pt.y)
                min_pt = polygon_cells[i];
            else if (polygon_cells[i].y > max_pt.y)
                max_pt = polygon_cells[i];
            ++i;
        }

        CellIndex pt;
        // loop though cells in the column
        for (unsigned int y = min_pt.y; y < max_pt.y; ++y)
        {
            pt.x = x;
            pt.y = y;
            polygon_cells.push_back(pt);
        }
    }
}

bool Costmap2D::enclosure(double wx, double wy, int thresholdSize)
{
    CellIndex cell_index = getCellIndex({wx, wy});
    if (!contains(cell_index)) {
        return false;
    }
    unsigned int lethal_obstacle_size = 0;
    std::vector<CellIndex> nhood_list = nhood8(cell_index, limits_);
    for (const auto& k : nhood_list) {
        if (!contains(k)) {
            continue;
        }
        if (getCost(k) == LETHAL_OBSTACLE || getCost(k) == LETHAL_OBSTACLE)
            lethal_obstacle_size++;
    }
    return lethal_obstacle_size >= thresholdSize;
}

}
}
