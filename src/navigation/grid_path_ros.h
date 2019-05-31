
#ifndef _GRID_PATH_H
#define _GRID_PATH_H

#include<vector>
#include "traceback.h"
#include "cell_index.h"

class GridPath_ros : public Traceback
{
public:
    GridPath_ros(PotentialCalculator *p_calc) : Traceback(p_calc)
    {}

    bool getPath(const std::vector<float> &potential,
                 int start_x, int start_y, int end_x, int end_y,
                 std::vector<std::pair<float, float> > &path);
};

#endif
