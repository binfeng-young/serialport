//
// Created by binfeng.yang on 2019/5/21.
//
#include "cell_index.h"

namespace bv {
namespace mapping {
static const int direction[8][2] = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

std::vector<CellIndex> CellLimits::nhood4(const CellIndex &idx, bool inc_out_side)
{
    //get 4-connected neighbourhood indexes, check for edge of map
    std::vector<CellIndex> out;
    out.reserve(4);
    for (int i = 0; i < 4; i++) {
        CellIndex next = CellIndex(idx.x + direction[i][0], idx.y + direction[i][1]);
        if (!inc_out_side && !isContains(next)) {
            continue;
        }
        out.push_back(next);
    }
    return out;
}

std::vector<CellIndex> CellLimits::nhood8(const CellIndex &idx, bool inc_out_side)
{
    //get 8-connected neighbourhood indexes, check for edge of map
    std::vector<CellIndex> out;
    out.reserve(8);
    for (const auto &i : direction) {
        CellIndex next = CellIndex(idx.x + i[0], idx.y + i[1]);
        if (!inc_out_side && !isContains(next)) {
            continue;
        }
        out.push_back(next);
    }
    return out;
}

std::vector<CellIndex> CellLimits::nhoodRadius(const CellIndex &idx, bool inc_out_side)
{

}

}
}