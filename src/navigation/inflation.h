//
// Created by peter on 4/2/19.
//

#pragma once

#include <cstdint>
#include <vector>

std::vector<uint8_t> inflation_for_obstacle(const std::vector<uint8_t> &state_grid, int width);


std::vector<uint8_t> inflation_for_obstacle_except_target(const std::vector<uint8_t> &state_grid,
                                                          int width, const CellIndex &target_cell,
                                                          const CellLimits & limits);
