/**
 * Created by bfyoung on 2019/2/14.
 */
#ifndef _COSTMAP_TOOLS_H_
#define _COSTMAP_TOOLS_H_

#include "cost_values.h"
#include "typedef.h"
#include "cell_index.h"

namespace bv {
namespace mapping {

void getDirCoord(int i, short &dx, short &dy);

bool isContains(const CellIndex &cell_index, const CellLimits& limits);

/**
 * @brief Determine 4-connected neighbourhood of an input cell, checking for map edges
 * @param costmap size_x
 * @param costmap size_y
 * @param idx input cell index
 * @return neighbour cell indexes
 */
std::vector<CellIndex> nhood4(const CellIndex& idx, const CellLimits& limits);

/**
 * @brief Determine 8-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap size_x
 * @param costmap size_y
 * @return neighbour cell indexes
 */
std::vector<CellIndex> nhood8(const CellIndex& idx, const CellLimits& limits);

/**
 * @brief Determine 24-connected neighbourhood of an input cell, checking for map edges
 * @param idx input cell index
 * @param costmap size_x
 * @param costmap size_y
 * @return neighbour cell indexes
 */
std::vector<CellIndex> nhood24(const CellIndex& idx, const CellLimits& limits);

/**
 * @brief Find nearest cell of a specified value
 * @param result Index of located cell
 * @param start Index initial cell to search from
 * @param val Specified value to search for
 * @param costmap char map
 * @param costmap size_x
 * @param costmap size_y
 * @return True if a cell with the requested value was found
 */
bool nearest8Cell(unsigned int &result, unsigned int start);

bool isNearest8ValCell(unsigned char val, unsigned int start);

uint16_t nearestClearCell(uint16_t target_cell, uint8_t queue_size);

Point transformPoint(const Point& point, const Point& offset, double cos, double sin);

Point padPoint(const Point& point, double length, double theta);
}
}

#endif /* _COSTMAP_TOOLS_H_ */
