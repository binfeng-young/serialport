//
// Created by binfeng.yang on 2019/4/22.
//

#ifndef BVROBOT2_REGION_H
#define BVROBOT2_REGION_H
#include <cell_index.h>
#include <typedef.h>
#include <vector>
#include <cmath>
namespace bv {
namespace mapping {
enum FoldCleanDirection {
    FOLD_CLEAN_DIR_LEFT,
    FOLD_CLEAN_DIR_RIGHT,
};

enum FoldHeadDirection {
    FOLD_HEAD_DIR_UP,
    FOLD_HEAD_DIR_DOWN,
    FOLD_HEAD_DIR_LEFT,
    FOLD_HEAD_DIR_RIGHT,
};

class Region {
public:
    Region() : max_pt_({0,0}), min_pt_({0,0}){}

    Region(const Point& max_point, const Point& min_point)
    : max_pt_(max_point)
    , min_pt_(min_point) {};

    Region (const Region &region)
    {
        *this = region;
    }

    Region& operator = (const Region& region)
    {
        if (this == &region) {
            return *this;
        }

        max_pt_ = region.max_pt_;
        min_pt_ = region.min_pt_;
        return *this;
    }
    /// \brief Return distance can move
    float getMoveDistance(const Point& point, FoldHeadDirection head_dir)
    {
        switch (head_dir) {
            case FOLD_HEAD_DIR_UP: return max_pt_.x - point.x;
            case FOLD_HEAD_DIR_DOWN: return point.x - min_pt_.x;
            case FOLD_HEAD_DIR_LEFT: return max_pt_.y - point.y;
            case FOLD_HEAD_DIR_RIGHT: return point.y - min_pt_.y;
        }
        //TODO: Judging how much has not been swept in front
    }

    float getFoldCleanDirection() {

    }

    bool isOnBorder(const Point& point) {
        return false;
    }

    bool isOutSide(const Point& point) {
        return false;
    }



    //bool isCleanOver();
    //float getMoveDistance();
    //Point getNearWall();
    // get clean dir c
    Point max_pt_;
    Point min_pt_;
};

class RegionMap {
public:
    RegionMap(CellLimits limits, float length)
        : limits_(limits)
        , length_(length)
        , regions_(limits.size_x * limits.size_y)
    {
        // the origin point is (0, 0)
        min_point_ =  {-0.5f * limits_.size_x * length_,
                       -0.5f * limits_.size_y * length_};
        //init regions
        for (auto index : CellIndexRangeIterator(limits_)) {
            auto center = getRegionalCenterPoint(index);
            auto id = toId(index);
            Point min_pt = {center.x - 0.5 * length_, center.y - 0.5 * length_};
            Point max_pt = {center.x + 0.5 * length_, center.y + 0.5 * length_};
            regions_[id] = Region(min_pt, max_pt);
        }
    }

    CellIndex getIndex(const Point& point)
    {
        return {(int)std::floor((point.x - min_point_.x) / length_),
            (int)std::floor((point.y - min_point_.y) / length_)};
    }

    Point getRegionalCenterPoint(const CellIndex& index) {
        return {min_point_.x + (index.x + 0.5f) * length_,
            min_point_.y + (index.y + 0.5f) * length_};
    }

    Region getRegion(const CellIndex& index)
    {
        return regions_[toId(index)];
    }

    Point getNearWall(const Point& point)
    {

    }

    bool contains(const CellIndex &cell_index) const
    {
        return limits_.isContains(cell_index);
    }

    int toId(const CellIndex& index) {
        return index.y * limits_.size_x + index.x;
    }

private:

    CellLimits limits_;
    float length_;
    Point min_point_;
    std::vector<Region> regions_;
};

}
}
#endif //BVROBOT2_REGION_H
