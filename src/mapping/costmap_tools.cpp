/**
 * Created by bfyoung on 2019/2/14.
 */
#include <cmath>
#include <vector>
#include <queue>

#include "costmap_tools.h"

namespace bv {
namespace mapping {
static short direction[8][2] = {0, 1, 1, 0, -1, 0, 0, -1, 1, 1, 1, -1, -1, 1, -1, -1};

void getDirCoord(int i, short &dx, short &dy)
{
    dx = direction[i][0];
    dy = direction[i][1];
}

bool isContains(const CellIndex &cell_index, const CellLimits& limits)
{
    return (0 <= cell_index.x && 0 <= cell_index.y) && (cell_index.x < limits.size_x && cell_index.y < limits.size_y);
}

/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap size_x_
 * @param costmap size_y_
 * @return neighbour cell indexes
 */
std::vector<CellIndex> nhood4(const CellIndex &idx, const CellLimits& limits)
{
    //get 4-connected neighbourhood indexes, check for edge of map
    std::vector<CellIndex> out;
    out.reserve(4);
    for (int i = 0; i < 4; i++) {
        CellIndex next = CellIndex(idx.x + direction[i][0], idx.y + direction[i][1]);
        if (isContains(next, limits)) {
            out.push_back(next);
        }
    }
    return out;
}

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap size_x_
 * @param costmap size_y_
 * @return neighbour cell indexes
 */
std::vector<CellIndex> nhood8(const CellIndex &idx, const CellLimits& limits)
{
    //get 8-connected neighbourhood indexes, check for edge of map
    std::vector<CellIndex> out;
    out.reserve(8);
    for (const auto &i : direction) {
        CellIndex next = CellIndex(idx.x + i[0], idx.y + i[1]);
        if (isContains(next, limits)) {
            out.push_back(next);
        }
    }
    return out;
}

/**
 * @brief Determine 24-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap size_x_
 * @param costmap size_y_
 * @return neighbour cell indexes
 */
std::vector<CellIndex> nhood24(const CellIndex &idx, const CellLimits& limits)
{
    //get 24-connected neighbourhood indexes, check for edge of map
    std::vector<CellIndex> out;
    out.reserve(24);
    auto min_cell = CellIndex(std::max(idx.x - 2, 0), std::max(idx.y - 2, 0));
    auto max_cell = CellIndex(std::min(idx.x + 2, limits.size_x) - 1, std::min(idx.y + 2, limits.size_y) - 1);
    /*    ROS_INFO("Size:(%d\t%d):(%d\t%d)--(%d\t%d)",size_x_, size_y_, x1, y1, x2, y2);*/
    for (const auto &index : CellIndexRangeIterator(min_cell, max_cell)) {
        if (index != idx)
            out.push_back(index);
    }
    return out;
}

//std::vector<unsigned int> side(unsigned int idx, char direction)
//{
//    auto mapI = CMAPI;
//    unsigned short size_x = mapI->getSizeInCellsX();
//    unsigned short size_y = mapI->getSizeInCellsY();
//    std::vector<unsigned int> out;
//    out.reserve(8);
//    if (idx > size_x * size_y - 1) {
//        return out;
//    }
//    if (direction) {
//        if (idx % size_x > 0) {
//            out.push_back(idx - 1);
//            if (idx >= size_x) {
//                out.push_back(idx - size_x - 1);
//            }
//            if (idx < size_x * (size_y - 1)) {
//                out.push_back(idx + size_x - 1);
//            }
//        }
//
//    } else {
//        if (idx % size_x < size_x - 1) {
//            out.push_back(idx + 1);
//            if (idx < size_x * (size_y - 1)) {
//                out.push_back(idx + size_x + 1);
//            }
//            if (idx < size_x * (size_y - 1)) {
//                out.push_back(idx + size_x + 1);
//            }
//        }
//    }
//    return out;
//}

////8邻域内空闲位置
//bool nearest8Cell(unsigned int &result, unsigned int start)
//{
//    std::vector<CellIndex> neighbor = nhood8(start);
//    for (unsigned short i : neighbor) {
//        if ((CMAPI->getCost(i) == FREE_SPACE)) {
//            result = i;
//            return true;
//        }
//    }
//    return false;
//}

//bool isNearest8ValCell(unsigned char val, unsigned int start)
//{
//    std::vector<CellIndex> neighbor = nhood8(start);
//    uint8_t num = 0;
//    for (auto nh : neighbor) {
//        if (CMAPI->getCost(nh) == val) {
//            num++;
//        }
//    }
//    return num >= 3;
//}

/*
 * function: nearestClearCell
 * describe: 搜索离指定点最近的空闲格子
 * parameter:
 *          in  -- target_cell   目标格子(cel)
 *          in  -- size 搜索队列大小
 * return:   搜索结果(cell)
 * */
//uint16_t nearestClearCell(uint16_t target_cell, uint8_t queue_size)
//{
//    auto mapI = CMAPI;
//    auto mapFlag = MFI;
//
//    if (target_cell >= mapI->getSize()) {
//        return target_cell;
//    }
//
//    //initialize breadth first search
//    std::queue<unsigned int> bfs;
//    //push initial cell
//    bfs.push(target_cell);
//    mapFlag->clear();
//    mapFlag->setFlag(target_cell, 1);
//    //search for neighbouring cell matching value
//    while ((!bfs.empty())) {
//        if (bfs.size() > queue_size) {
//            break;
//        }
//        unsigned int idx = bfs.front();
//        bfs.pop();
//
//        //return if cell of correct value is found
//        if (mapI->getCost(idx) == FREE_SPACE) {
//            return idx;
//        }
//        auto neighbour = nhood8(idx);
//        //iterate over all adjacent unvisited cells
//        for (auto nbr : neighbour) {
//            if (!mapFlag->getFlag(nbr)) {
//                bfs.push(nbr);
//                mapFlag->setFlag(nbr, 1);
//            }
//        }
//    }
//    return target_cell;
//}
Point transformPoint(const Point &point, const Point &offset, double cos, double sin)
{
    return {
        offset.x + point.x * cos - point.y * sin,
        offset.y + point.y * cos + point.x * sin
    };
}

Point padPoint(const Point &offset, double length, double theta)
{
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    return transformPoint({(length * 1.2) * cos_th, (length * 1.2) * sin_th}, offset, cos_th, sin_th);
}

}
}
