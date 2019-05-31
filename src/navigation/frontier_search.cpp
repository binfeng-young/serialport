/*
 * frontier_search.cpp
 *
 *  Created on: 2016年10月7日
 *      Author: leslie
 */

#include "frontier_search.h"
#include "bv_type.h"
#include "costmap_tools.h"
#include <iostream>

#define ERROR_INDEX (CellIndex(-1, -1))

FrontierSearch::FrontierSearch(std::shared_ptr<Costmap2D> costmap)
{
    costmap_ = costmap;
    size_x_ = costmap->getLimits().size_x;
    size_y_ = costmap->getLimits().size_y;
    map_size_ = size_x_ * size_y_;
    //frontier_extraction::outlineMap(map_, size_x_, size_y_, costmap_2d::OBSTACLE);
    //memset(map_flag_, 0, size_x_ * size_y_ * sizeof(uint8_t));
    last_goal_ = before_last_goal_ = ERROR_INDEX;
}

std::vector<FrontierSearch::PositionInfo> FrontierSearch::searchFrom(const CellIndex& position, FrontierType type)
{
    unsigned short int iter_count = 0;
    std::vector<PositionInfo> frontier_list;
	std::vector<uint8_t> map_flag(size_x_ * size_y_, 0);

    //find closest clear cell to start search
    {
        auto n8 = nhood8(position, costmap_->getLimits());
        for (const auto &n : n8) {
            costmap_->setCost(n, MapValue::PASSED);
        }
    }
    std::queue<PositionInfo> bfs;
    bfs.emplace(position, 0);

	auto start_id = costmap_->getLimits().toId(bfs.front().index);
	map_flag[start_id] = 1;
//    uint32_t current_time = 0, last_time = 0;
	/*cv::Mat img(size_y_, size_x_, CV_8UC1, cv::Scalar::all(0));
	cv::Mat color_img;
	cv::cvtColor(img, color_img, CV_GRAY2BGR);
	cv::namedWindow("frontier", cv::WINDOW_KEEPRATIO);*/
    while (!bfs.empty()/* && iter_count < SEARCH_TIMES*/)  //600
    {
        iter_count++;
        auto idx = bfs.front().index;
        auto dis = bfs.front().dis;
        bfs.pop();
        // iterate over 4-connected neighbourhood
        std::vector<CellIndex> neighbour = nhood8(idx, costmap_->getLimits());
        for (const auto &nbr: neighbour) {
            auto id = costmap_->getLimits().toId(nbr);
            if (map_flag[id]) {
                continue;
            }
            map_flag[id] = 1;

            if (costmap_->getCost(nbr) == MapValue::PASSED || (type == FrontierType::WALL && costmap_->getCost(nbr) == MapValue ::FREE_SPACE)) {
                bfs.emplace(nbr, dis + 1);
                continue;
            }

            if (!isFrontierCell(nbr, type)) {
                continue;
            }
            auto new_frontier = buildNewFrontier(nbr, position, type, map_flag);
            if (new_frontier != ERROR_INDEX) {
                frontier_list.emplace_back(new_frontier, dis + 1);
                if (type == FrontierType::WALL) {
                    return frontier_list;
                }
            }
        }
		//cv::circle(color_img, { idx.x, idx.y }, 5, {255, 0 ,0});
        //cv::imshow("frontier", color_img);
        //cv::waitKey(1);
    }
    //cv::waitKey(0);
    return frontier_list;
}

CellIndex
FrontierSearch::buildNewFrontier(const CellIndex& initial_cell, const CellIndex& reference, FrontierType type, std::vector<uint8_t>& map_flag)
{
    int minDistance = abs(reference.x - initial_cell.x) + abs(reference.y - initial_cell.y);
    auto frontier = initial_cell;
    if (type == FrontierType::NO_PASS) {
        if (!isValidFrontier(initial_cell)) {
            minDistance = std::numeric_limits<uint16_t>::max();
            frontier = ERROR_INDEX;
        }
    }
    CellIndex max_cell = initial_cell;
    CellIndex min_cell = initial_cell;
    //record initial contact point for frontier
    //push initial gridcell onto queue
    std::queue<CellIndex> bfs;
    bfs.push(initial_cell);
    int size = 0;
    while (!bfs.empty()) {
        size++;
        auto idx = bfs.front();
        bfs.pop();
        auto neighbour = nhood4(idx, costmap_->getLimits());
        // try adding cells in 8-connected neighborhood to frontier
        for (const auto &nbr : neighbour) {
            auto id = costmap_->getLimits().toId(nbr);
            //check that cell is type and not already marked as frontier
            if (map_flag[id]) {
                continue;
            }
            map_flag[id] = 1;
            if (!isFrontierCell(nbr, type)) {
                continue;
            }
            max_cell.x = std::max(max_cell.x, nbr.x);
            max_cell.y = std::max(max_cell.y, nbr.y);
            min_cell.x = std::min(min_cell.x, nbr.x);
            min_cell.y = std::min(min_cell.y, nbr.y);
            // mark cell as frontier
            //determine frontier's distance from robot, going by closest gridcell to robot
            if (type == FrontierType::NO_PASS) {
                int distance = abs(reference.x - nbr.x) + abs(reference.y - nbr.y);
                if (isValidFrontier(nbr) && distance < minDistance && distance < std::numeric_limits<uint16_t>::max()) {
                    minDistance = distance;
                    frontier = nbr;
                }
            }
            // add to queue for breadth first search
            bfs.push(nbr);
        }
    }
    if (type == FrontierType::NO_PASS) {
        return size > 3 ? frontier : ERROR_INDEX;
    } else {
        return  (max_cell.x - min_cell.x + max_cell.y - min_cell.y) > 50 ? frontier : ERROR_INDEX;
    }
    //return frontier;
}

bool FrontierSearch::isFrontierCell(const CellIndex& idx, FrontierType type)
{
    auto cost = costmap_->getCost(idx);
    if (type == FrontierType::NO_PASS && cost != MapValue::UNKNOWN && cost != MapValue::FREE_SPACE) {
        return false;
    } else if (type == FrontierType::WALL && cost != MapValue::OBSTACLE && cost != MapValue::LOOP_OBSTACLE) {
        return false;
    }

    auto nhoods = nhood8(idx, costmap_->getLimits());
    for (const auto &n : nhoods) {
        if (costmap_->getCost(n) == MapValue::PASSED || (type == FrontierType::WALL && costmap_->getCost(n) == MapValue::FREE_SPACE)) {
            return true;
        }
    }
    return false;
}

bool FrontierSearch::isValidFrontier(const CellIndex& frontier)
{
    if (frontier == last_goal_ || frontier == before_last_goal_) {
        return false;
    }
    unsigned short int no_information_size = 0;
    unsigned short int lethal_obstacle_size = 0;
    unsigned short int passed_size = 0;
    auto nhood24s = nhood24(frontier, costmap_->getLimits());
    for (const auto &nbr : nhood24s) {
        auto cost = costmap_->getCost(nbr);
        if (cost == MapValue::UNKNOWN || cost == MapValue::FREE_SPACE) {
            no_information_size++;
        } else if (cost == MapValue::OBSTACLE
                   || cost == MapValue::LOOP_OBSTACLE) {
            lethal_obstacle_size++;
        } else if (cost == MapValue::PASSED) {
            passed_size++;
        }
    }
    //DEBUG_LOG(DEBUG_THIS_FILE,"24 neighborhood num(%d) = %d,%d,%d\r\n",  nhood24_list.size(), no_information_size, lethal_obstacle_size, free_space_size);

    return (lethal_obstacle_size < 10 && no_information_size > 5 && passed_size > 2)
           || (no_information_size > 10 && lethal_obstacle_size == 0 && passed_size >= 4)
           || no_information_size > 12;
}

bool FrontierSearch::isExistValidFrontier()
{
	std::vector<uint8_t> map_flag(size_x_ * size_y_, 0);
    auto searchFrom_ = [&](CellIndex position) {
        if (map_flag[costmap_->getLimits().toId(position)]) {
            return false;
        }
        //DEBUG_LOG(DEBUG_THIS_FILE, "searchFrom\r\n");
        bool exist = false;
        unsigned short int iter_count = 0;
        std::queue<CellIndex> bfs;
		map_flag[costmap_->getLimits().toId(position)] = 1;
        bfs.push(position);
        while (!bfs.empty()) {
            iter_count++;
            auto idx = bfs.front();
            bfs.pop();
            auto neighbour = nhood8(idx, costmap_->getLimits());
            for (const auto &nbr: neighbour) {
                auto id = costmap_->getLimits().toId(nbr);
                if (map_flag[id]) {
                    continue;
                }
				map_flag[id] = 1;
                if (costmap_->getCost(nbr) == MapValue::FREE_SPACE) {
                    bfs.push(nbr);
                } else if (!exist && isFrontierCell(nbr, FrontierType::NO_PASS)) {
                    auto new_frontier = buildNewFrontier(nbr, position, FrontierType::NO_PASS, map_flag);
                    if (new_frontier != ERROR_INDEX) {
                        exist = true;
                    }
                }
            }
        }
        return exist && iter_count > 50;
    };
  

    for (const auto &index : CellIndexRangeIterator(costmap_->getLimits())) {
        if (costmap_->getCost(index) == MapValue::FREE_SPACE && searchFrom_(index)) {
            return true;
        }
    }
    return false;
}


float FrontierSearch::distance2D(float ax, float ay, float bx, float by)
{
//    return sqrtf(powf(ax - bx, 2) + powf(ay - by, 2));
    return sqrtf((ax - bx) * (ax - bx) + (ay - by) * (ay - by));
}


bool FrontierSearch::getGoalFromFrontier(const CellIndex& current_position, CellIndex &goal, FrontierType type)
{
    //DEBUG_LOG(DEBUG_THIS_FILE, " getGoalFromFrontier\r\n");
    std::vector<PositionInfo> frontier_list_ = searchFrom(current_position, type);
    if (frontier_list_.empty()) {
        //DEBUG_LOG(DEBUG_THIS_FILE, " frontier points size is zero , can not find a goal\r\n");
        return false;
    }

    if (type == FrontierType::WALL) {
        goal = frontier_list_[0].index;
        return goal != ERROR_INDEX;
    }
//    float cx, cy;
//    mapI->indexToWorld(current_position, cx, cy);
//    auto getCandidate = [&](unsigned short idx) {
//        frontier_extraction::Score candidate{};
//        float x, y;
//        mapI->indexToWorld(idx, x, y);
//        candidate.distence = distance2D(cx, cy, x, y);
//        //边界点周围未知区域个数
//        candidate.unknown_num = getNeighbourUnknow(idx);
//        return candidate;
//    };
    unsigned short int ix, iy;
    float distance_sum = 0;
    int unknown_num_sum = 0;
    float max_score = 10e-6;
    auto temp = ERROR_INDEX;
    for (auto frontier : frontier_list_) {
        //当前位置距离边界点位置
        //frontier_extraction::Score tem_candidate = getCandidate(frontier);
        distance_sum += frontier.dis;
        unknown_num_sum += getNeighbourUnknow(frontier.index);
    }
    //DEBUG_LOG(DEBUG_THIS_FILE, " current pose = %d , %d  \r\n", ix, iy);
    for (auto frontier : frontier_list_) {
        if (fabsf(distance_sum) <= 10e-6 || unknown_num_sum == 0) {
            temp = frontier_list_[0].index;
            break;
        }
        //frontier_extraction::Score tem_candidate = getCandidate(frontier);
        float score = 0.8f * (1 - (frontier.dis / distance_sum)) + 0.2f * getNeighbourUnknow(frontier.index) / unknown_num_sum;
        if (score > max_score) {
            max_score = score;
            temp = frontier.index;
        }
    }
    goal = temp;
    // Convert the Euler angle to quaternion
    //update last goal
    before_last_goal_ = last_goal_;
    last_goal_ = goal;
    //DEBUG_LOG(LOG_DEBUG, " goal pose = %d , %d  \r\n", ix, iy);
    return goal != ERROR_INDEX;
}

char FrontierSearch::getNeighbourUnknow(const CellIndex& index)
{
    char no_information_size = 0;
    auto nhood24_list = nhood24(index, costmap_->getLimits());
    for (const auto& k : nhood24_list) {
        if (costmap_->getCost(k) == MapValue::UNKNOWN) {
            no_information_size++;
        }
    }
    return no_information_size;
}

char FrontierSearch::judgeDirection(const CellIndex& goal, int8_t direction_x_y)
{
    // Judge the run fold direction
    int no_information_size_up = 0;
    int no_information_size_down = 0;
    int size_x = size_x_, size_y = size_y_;

    int ixx_min, iyy_min, ixx_max, iyy_max;
    int index_iter;
    ixx_min = goal.x - 2;
    ixx_max = goal.x + 3;
    iyy_min = goal.y - 2;
    iyy_max = goal.y + 3;
    if (ixx_min < 0) ixx_min = 0;
    if (ixx_max > size_x) ixx_max = size_x;
    if (iyy_min < 0) iyy_min = 0;
    if (iyy_max > size_y) iyy_max = size_y;

    if (direction_x_y) {
        for (int j = iyy_min; j < goal.y; j++) {
            for (int i = ixx_min; i < ixx_max; i++) {
                if (costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_up++;
            }
        }
        index_iter = 0;
        for (int j = goal.y + 1; j < iyy_max; j++) {
            for (int i = ixx_min; i < ixx_max; i++) {
                if(costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_down++;
            }
        }
    } else {
        for (int j = iyy_min; j < iyy_max; j++) {
            for (int i = ixx_min; i < goal.x; i++) {
                if(costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_up++;
            }
        }
        index_iter = 0;
        for (int j = iyy_min; j < iyy_max; j++) {
            for (int i = goal.x + 1; i < ixx_max; i++) {
                if(costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_down++;
            }
        }
    }
    if (no_information_size_up > no_information_size_down)
        return -1;
    else
        return 1;
}

char FrontierSearch::judgeTowards(const CellIndex& goal, int8_t direction_x_y)
{
    // Judge the run fold direction
    int no_information_size_back = 0;
    int no_information_size_front = 0;
    int size_x = size_x_, size_y = size_y_;
    int ixx_min, iyy_min, ixx_max, iyy_max;
    int index_iter;
    if (direction_x_y) {
        ixx_min = goal.x - 5;
        ixx_max = goal.x + 6;
        iyy_min = goal.y - 2;
        iyy_max = goal.y + 3;
    } else {
        ixx_min = goal.x - 2;
        ixx_max = goal.x + 3;
        iyy_min = goal.y - 5;
        iyy_max = goal.y + 6;
    }

    if (ixx_min < 0) ixx_min = 0;
    if (ixx_max > size_x) ixx_max = size_x;
    if (iyy_min < 0) iyy_min = 0;
    if (iyy_max > size_y) iyy_max = size_y;

    if (direction_x_y) {
        for (int j = ixx_min; j < goal.x; j++) {
            for (int i = iyy_min; i < iyy_max; i++) {
                if(costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_back++;
            }
        }
        index_iter = 0;
        for (int j = goal.x + 1; j < ixx_max; j++) {
            for (int i = iyy_min; i < iyy_max; i++) {
                if(costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_front++;
            }
        }
    } else {
        for (int j = ixx_min; j < ixx_max; j++) {
            for (int i = iyy_min; i < (int) goal.y; i++) {
                if(costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_front++;
            }
        }
        index_iter = 0;
        for (int j = ixx_min; j < ixx_max; j++) {
            for (int i = goal.y + 1; i < iyy_max; i++) {
                if(costmap_->getCost(CellIndex(i, j)) == MapValue::UNKNOWN)
                    no_information_size_back++;
            }
        }
    }
    //DEBUG_LOG(DEBUG_THIS_FILE, "towards front , back =  %d,%d\r\n", no_information_size_front,
              //no_information_size_back);
    if (no_information_size_front < no_information_size_back)
        return -1;
    else
        return 1;
}

bool FrontierSearch::isInList(const CellIndex&idx, const std::vector<CellIndex> &frontier_flag)
{
    for (auto i : frontier_flag) {
        if (idx == i) {
            return true;
        }
    }
    return false;
}