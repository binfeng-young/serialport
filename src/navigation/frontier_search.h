/*
 * frontier_search.h
 *
 *  Created on: 2016年10月7日
 *      Author: leslie
 */

#ifndef MODULES_NAVIGATION_INC_FRONTIER_SEARCH_H_
#define MODULES_NAVIGATION_INC_FRONTIER_SEARCH_H_
#include "costmap_2d.h"
#include "cell_index.h"
#include <vector>
#include <queue>
#include <list>
#include <map>

struct Score {
    float distence;
    char unknown_num;
};

struct Frontier {
    unsigned short int size;
    float min_distance;
    Point_2f initial;
//    struct Point centroid;
    Point_2f middle;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input costmap.
 */
class FrontierSearch {

public:
    struct PositionInfo {
        PositionInfo(const CellIndex& _index, unsigned short int _dis)
        {
            index = _index;
            dis = _dis;
        }

        CellIndex index;
        unsigned short int dis;
    };

    FrontierSearch(std::shared_ptr<Costmap2D> costmap);

    float distance2D(float ax, float ay, float bx, float by);

    /**
     * @brief Runs search implementation, outward from the start position
     * @param position Initial position to search from
     * @return List of frontiers, if any
     */
    std::vector<PositionInfo> searchFrom(const CellIndex& position, FrontierType type);

    bool isValidFrontier(const CellIndex& frontier);

    bool isExistValidFrontier();

    /**
     * @brief     from frontier points , get one of them as next goal
     * @param current_pose  the robot current pose
     * @param goal output, the goal we find
     * @return
     */
    bool getGoalFromFrontier(const CellIndex& current_position, CellIndex& goal, FrontierType type = FrontierType::NO_PASS);

    /**
     * @brief    if we reach the goal , determin which side should we go
     * @param goal  the goal we want to reach
     * @param direction output,the side
     * @return
     */
    char judgeDirection(const CellIndex& goal, int8_t direction_x_y);

    char judgeTowards(const CellIndex& goal, int8_t direction_x_y);

    char getNeighbourUnknow(const CellIndex& index);

    bool isInList(const CellIndex& idx, const std::vector<CellIndex> &frontier_flag);

protected:
    CellIndex buildNewFrontier(const CellIndex& initial_cell, const CellIndex& reference, FrontierType type, std::vector<uint8_t>& map_flag);

    /**
    * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
    * @param idx Index of candidate cell
    * @param frontier_flag Flag vector indicating which cells are already marked as frontiers
    * @return
    */
    bool isFrontierCell(const CellIndex& idx, FrontierType type);

private:
    std::shared_ptr<Costmap2D> costmap_;
    int size_x_, size_y_;
    int map_size_;
    CellIndex last_goal_;
    CellIndex before_last_goal_;
};
#endif /* MODULES_NAVIGATION_INC_FRONTIER_SEARCH_H_ */
