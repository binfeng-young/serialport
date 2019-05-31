//
// Created by peter on 4/2/19.
//

#include <cstddef>
#include <cell_index.h>
#include "inflation.h"
#include <costmap_2d.h>

std::vector<uint8_t> inflation_for_obstacle(const std::vector<uint8_t> &state_grid, int width)
{
    if(state_grid.empty())
        return std::vector<uint8_t>();

    size_t size = state_grid.size();
    int inflation_r = 2;

    std::vector<uint8_t> inflation_costs = state_grid;
    for(size_t i = 0; i < size; i++)
    {
        if(state_grid[i] == 254)// obstacle
        {
            for(int row = -inflation_r; row <= inflation_r; row++)
            {
                for(int col = -inflation_r; col <= inflation_r; col++)
                {
                    size_t index = i + row * width + col;
                    if(index < 0 || index >= size)
                        continue;

                    inflation_costs.at(index) = 254;
                }
            }
        }
    }

    return inflation_costs;
}

std::vector<uint8_t> inflation_for_obstacle_except_target(const std::vector<uint8_t> &state_grid,
                                                          int width, const CellIndex &target_cell,
                                                          const CellLimits & limits)
{
    if(state_grid.empty())
        return std::vector<uint8_t>();

    auto size = state_grid.size();
    int inflation_r = 2;

    auto target_i = limits.toId(target_cell);

    std::vector<uint8_t> inflation_costs = state_grid;
    for(size_t i = 0; i < size; i++)
    {
        if(state_grid[i] != 254 || i == target_i)// obstacle
        {
            continue;
        }
        for(int row = -inflation_r; row <= inflation_r; row++)
        {
            for(int col = -inflation_r; col <= inflation_r; col++)
            {
                size_t index = i + row * width + col;
                if(index < 0 || index >= size)
                    continue;

                inflation_costs.at(index) = 254;
            }
        }
    }

    return inflation_costs;
}