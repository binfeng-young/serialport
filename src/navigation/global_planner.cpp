/*
 * planner_core.cpp
 *
 *  Created on: 2016年10月7日
 *      Author: leslie
 */

#include "global_planner.h"

#include "bv_type.h"
#include "costmap_tools.h"
#include <algorithm>
#include <iostream>
#include "astar.h"
#include "opencv_test.h"

using std::cout;
using std::endl;

GlobalPlanner::GlobalPlanner(std::shared_ptr<Costmap2D> costmap) : allow_unknown_(true)
{
    costmap_ = costmap;
    size_x_ = costmap_->getLimits().size_x;
    size_y_ = costmap_->getLimits().size_y;
    //std::cout << "map width: " << size_x_ << " height: " << size_y_ << std::endl;
    //planner_ = new AStar();
    p_calc_ = new PotentialCalculator(size_x_, size_y_);
    planner_ = new AStarExpansion(p_calc_, size_x_, size_y_);
    //path_maker_ = new GridPath();
    path_maker_ = new GridPath_ros(p_calc_);
    //set global planner parameters
    path_maker_->setLethalCost(254);
}

GlobalPlanner::~GlobalPlanner()
{
    delete planner_;
    delete path_maker_;
}

bool GlobalPlanner::makePlan(const CellIndex &start,
                             const CellIndex &goal,
                             std::vector<CellIndex> &plan)
{
    plan.clear();
    int nx = size_x_;
    int ny = size_y_;
    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);
    potential_array_.reserve(nx * ny);
    costmap_->setCost(goal, MapValue::FREE_SPACE);

    bool found_legal = planner_->calculatePotentials(costmap_->get_cells(), start.x, start.y,
                                                     goal.x, goal.y,
                                                     nx * ny * 2, potential_array_);



    if (found_legal)
    {
        std::vector<std::pair<float, float>> path;
        bool succ = path_maker_->getPath(potential_array_, start.x, start.y,
                                         goal.x, goal.y, path);
        if (succ) //getPlanFromPotential(*mapFlag, start, goal, plan)
        {
            plan.reserve(path.size());
            for(const auto& pair : path)
            {
                plan.emplace_back(pair.first, pair.second);
            }
        }
        else
        {
            plan.clear();
        }
    }

    return !plan.empty();
}


/*bool GlobalPlanner::getPlanFromPotential(const MapFlag &mapFlag, const CellIndex &start, const CellIndex &goal,
                                         std::vector<CellIndex> &plan)
{
    //DEBUG_LOG(DEBUG_THIS_FILE,"Try To Get Plan From Potential\r\n");
    if (!initialized_)
    {
        //DEBUG_LOG(DEBUG_THIS_FILE, "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }
    //std::vector<unsigned short> path;
//    path.reserve(100);
    //clear the plan, just in case
    plan.clear();
    //DEBUG_LOG(DEBUG_THIS_FILE,"Try to get path,potential_ size = %d!\r\n", potential_.size());
    //if (!path_maker_->getPath(*costmap_, mapFlag, start, goal, plan))
    {
        //DEBUG_LOG(DEBUG_THIS_FILE, "NO PATH!");
        plan.clear();
        return false;
    }
    if (plan.size() > 1)
    {
        plan.pop_back();
        std::reverse(plan.begin(), plan.end());
    }
    return !plan.empty();
}*/


