/*
 * planner_core.h
 *
 *  Created on: 2016年10月7日
 *      Author: leslie
 */

#ifndef MODULES_NAVIGATION_INC_GLOBAL_PLANNER_H_
#define MODULES_NAVIGATION_INC_GLOBAL_PLANNER_H_

#include <vector>
#include <map>
#include "bv_type.h"
#include "mapping/costmap_2d.h"

#include "expander.h"
#include "potential_calculator.h"
#include "grid_path_ros.h"

/**
 * @class PlannerCore
 * @brief Provides a ROS wrapper for the global_planner planner which runs a fast, interpolated navigation function on a costmap.
 */

class GlobalPlanner {
public:

    /**
     * @brief  Constructor for the PlannerCore object
     * @param  costmap A pointer to the costmap to use
     */
    GlobalPlanner(std::shared_ptr<Costmap2D> costmap);

    /**
     * @brief  Default deconstructor for the PlannerCore object
     */
    ~GlobalPlanner();

    /**
     * @brief  Initialization function for the PlannerCore object
     * @param  name The name of this planner
     * @param  costmap to use for planning
     */

    void initialize(Costmap2D *costmap);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start The start index
     * @param goal The goal index
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const CellIndex &start, const CellIndex &goal,
                  std::vector<CellIndex> &plan);

    /**
     * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
     * @param start_x
     * @param start_y
     * @param end_x
     * @param end_y
     * @param goal The goal pose to create a plan to
     * @param plan The plan... filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    //bool getPlanFromPotential(const MapFlag &mapFlag, const CellIndex &start, const CellIndex &goal, std::vector<CellIndex> &plan);


protected:
    PotentialCalculator* p_calc_;
    /**
     * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
     */
    bool initialized_, allow_unknown_, visualize_potential_;
private:
    std::shared_ptr<Costmap2D> costmap_;
    int size_x_;
    int size_y_;
    Expander *planner_;
    GridPath_ros *path_maker_;
    std::vector<float> potential_array_;
};



#endif /* MODULES_NAVIGATION_INC_GLOBAL_PLANNER_H_ */
