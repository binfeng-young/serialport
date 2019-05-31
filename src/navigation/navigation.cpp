#include "navigation.h"
#include "costmap_2d.h"
#include "global_planner.h"
#include "frontier_search.h"

#include "opencv_test.h"
#include "inflation.h"
#include "region.h"

using std::cout;
using std::endl;

namespace
{
std::vector<CellIndex> calc_simple_path(const CellIndex &start_cell,
                                        const CellIndex &target_cell)
{
    std::vector<CellIndex> path;
    int x_diff = target_cell.x - start_cell.x;
    int y_diff = target_cell.y - start_cell.y;
    CellIndex path_index = start_cell;

    path.push_back(path_index);

    if (x_diff < 0)
    {
        while (path_index.x != target_cell.x)
        {
            path_index.x -= 1;
            path.push_back(path_index);
        }
    }
    else
    {
        while (path_index.x != target_cell.x)
        {
            path_index.x += 1;
            path.push_back(path_index);
        }
    }

    if (y_diff < 0)
    {
        while (path_index.y != target_cell.y)
        {
            path_index.y -= 1;
            path.push_back(path_index);
        }
    }
    else
    {
        while (path_index.y != target_cell.y)
        {
            path_index.y += 1;
            path.push_back(path_index);
        }
    }

    path.push_back(target_cell);

    return path;
}

void change_grid(std::vector<uint8_t> &state_grid, int width, int height)
{
    std::vector<uint8_t> grid(state_grid.size(), 255);
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            int old_index = (width - 1 - x) + y * width;
            //int local_index = x + y * width;
            uint8_t state = state_grid[old_index];
            int new_index = x + y * width;
            grid[new_index] = state;
        }
    }
    state_grid = grid;
}
}
void Navigation::update_global_map()
{
    std::unique_lock<std::mutex> lck(*map_mutex_);
    CellLimits map_limits{map_->width, map_->height};
    costmap_ = std::make_shared<Costmap2D>(map_limits, map_->resolution, map_->origin);
    costmap_->set_cells(map_->state_grid);
}


bool Navigation::find_target(const CellIndex &start_cell, CellIndex &target_cell, FrontierType type)
{
	update_global_map();

    std::vector<uint8_t> &costs = costmap_->get_cells();
    costs = inflation_for_obstacle_except_target(costs, costmap_->getLimits().size_x,
                                                 target_cell, costmap_->getLimits());

	FrontierSearch frontier_search(costmap_);
	return frontier_search.getGoalFromFrontier(start_cell, target_cell, type);
}


std::vector<CellIndex> Navigation::frontiers(const CellIndex &start_cell)
{
    update_global_map();

    std::vector<uint8_t> &costs = costmap_->get_cells();
    costs = inflation_for_obstacle(costs, costmap_->getLimits().size_x);


    FrontierSearch frontier_search(costmap_);
	std::vector<FrontierSearch::PositionInfo> infos = frontier_search.searchFrom(start_cell, FrontierType::WALL);
	std::vector<CellIndex> cells;
	for (const auto &info : infos) {
		cells.emplace_back(info.index);
	}
	return cells;
}

std::vector<CellIndex> Navigation::find_path(const CellIndex &start_cell,
                                             const CellIndex &target_cell)
{
    update_global_map();

    std::vector<uint8_t> &costs = costmap_->get_cells();
    costs = inflation_for_obstacle_except_target(costs, costmap_->getLimits().size_x,
                                                 target_cell, costmap_->getLimits());

    //show global map on opencv
    //show_global_map(global_map_.state_grid.data(), global_map_.width, global_map_.height, start_cell, target_cell);

    GlobalPlanner globalPlanner(costmap_);

    std::vector<CellIndex> path;
    globalPlanner.makePlan(start_cell, target_cell, path);

    //test
    //path = calc_simple_path(start_cell, target_cell);

    return path;
}

void Navigation::add_passed_cells_to_costs(const std::vector<CellIndex> &passed_cells, int width,
                                           std::vector<uint8_t> &costs)
{
    if (passed_cells.empty())
        return;

    size_t size = costs.size();
    int inflation_r = 2;

    std::vector<uint8_t> costs_new = costs;
    for (const auto &cell : passed_cells)
    {
        for (int y = -inflation_r; y <= inflation_r; y++)
        {
            for (int x = -inflation_r; x <= inflation_r; x++)
            {
                int y_new = cell.y + y;
                int x_new = cell.x + x;
                size_t index = y_new * width + x_new ;
                if (index < 0 || index >= size)
                    continue;

                costs_new.at(index) = 0;
                //cout << "passed cell y: " << y_new << " x: " << x_new << endl;
            }
        }
    }

    costs = costs_new;
}