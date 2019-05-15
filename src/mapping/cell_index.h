//
// Created by binfeng.yang on 2019/3/4.
//

#ifndef CELL_INDEX_H
#define CELL_INDEX_H

#include <memory>

namespace bv {
namespace mapping {

// convenient for storing x/y point pairs
struct CellIndex {
    int x;
    int y;
    CellIndex() { x = 0; y = 0; }

    CellIndex(int x, int y)
    {
        this->x = x;
        this->y = y;
    }
    bool operator==(const CellIndex& other) const {
        return x == other.x && y == other.y;
    }

    bool operator!=(const CellIndex& other) const {
        return !operator==(other);
    }

    CellIndex operator+(const CellIndex& other) const {
        return {x + other.x, y + other.y};
    }

    CellIndex operator-(const CellIndex& other) const {
        return {x - other.x, y - other.y};
    }
};

class CellLimits {
public:
    CellLimits() = default;
    CellLimits(int init_size_x, int init_size_y)
        : size_x(init_size_x), size_y(init_size_y) {}

    bool isContains(const CellIndex &cell_index) const
    {
        return (0 <= cell_index.x && 0 <= cell_index.y) && (cell_index.x < size_x && cell_index.y < size_y);
    }

    int toId(const CellIndex &index) const
    {
        return index.y * size_x + index.x;
    }
    int size_x = 0;
    int size_y = 0;
};

// Iterates in row-major order through a range of xy-indices.
class CellIndexRangeIterator
    : public std::iterator<std::input_iterator_tag, CellIndex> {
public:
    // Constructs a new iterator for the specified range.
    CellIndexRangeIterator(const CellIndex& min_xy_index,
                           const CellIndex& max_xy_index)
        : min_xy_index_(min_xy_index),
          max_xy_index_(max_xy_index),
          xy_index_(min_xy_index) {}

    // Constructs a new iterator for everything contained in 'cell_limits'.
    explicit CellIndexRangeIterator(const CellLimits& cell_limits)
        : CellIndexRangeIterator(CellIndex(0, 0),
                                 CellIndex(cell_limits.size_x - 1,
                                           cell_limits.size_y - 1)) {}

    CellIndexRangeIterator& operator++() {
        // This is a necessary evil. Bounds checking is very expensive and needs to
        // be avoided in production. We have unit tests that exercise this check
        // in debug mode.
        if (xy_index_.x < max_xy_index_.x) {
            ++xy_index_.x;
        } else {
            xy_index_.x = min_xy_index_.x;
            ++xy_index_.y;
        }
        return *this;
    }

    CellIndex& operator*() { return xy_index_; }

    bool operator==(const CellIndexRangeIterator& other) const {
        return xy_index_ == other.xy_index_;
    }

    bool operator!=(const CellIndexRangeIterator& other) const {
        return !operator==(other);
    }

    CellIndexRangeIterator begin() {
        return CellIndexRangeIterator(min_xy_index_, max_xy_index_);
    }

    CellIndexRangeIterator end() {
        CellIndexRangeIterator it = begin();
        it.xy_index_ = CellIndex(min_xy_index_.x, max_xy_index_.y + 1);
        return it;
    }

private:
    CellIndex min_xy_index_;
    CellIndex max_xy_index_;
    CellIndex xy_index_;
};

}
}
#endif //CELL_INDEX_H
